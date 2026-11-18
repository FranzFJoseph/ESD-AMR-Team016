#include "nav.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define PI_F 3.14159265359f
#define TWO_PI_F (2.0f * PI_F)
#define CROSS_JUNCTION_Y_M 0.70f
#define CROSS_BRANCH_X_M 0.70f
#define COORDINATE_MATCH_M 0.001f
#define WAYPOINT_TOLERANCE_M 0.025f
#define MAX_LINEAR_MPS 0.20f
#define MAX_WHEEL_MPS 0.35f
#define MAX_YAW_RATE_RAD_S 1.8f
#define TURN_IN_PLACE_RAD 0.70f
#define BRAKING_GAIN_PER_S 2.4f
#define IMU_CORRECTION 0.20f
#define LIDAR_FULL_TURN_Q6 (360u * 64u)
#define LIDAR_COVERAGE_SECTOR_Q6 (15u * 64u)
#define LIDAR_FULL_COVERAGE ((1u << 24u) - 1u)
#define LIDAR_MIN_MM 50u
#define LIDAR_MAX_MM 12000u
#define LIDAR_SINGLE_STOP_MM 250u
#define LIDAR_NEAR_STOP_MM 280u
#define LIDAR_PAIR_STOP_MM 400u
#define LIDAR_CORRIDOR_HALF_WIDTH_MM 145.0f
#define LIDAR_CORRIDOR_ANGLE_Q6 (45u * 64u)
#define LIDAR_PAIR_GAP_Q6 (4u * 64u)
#define LIDAR_PAIR_MAX_MS 25u
#define LIDAR_STALE_MS 300u

void NavLidarResetSweep(NavLidarGuard *guard)
{
    guard->coverage = 0u;
    guard->usable_returns = 0u;
    guard->tracking_sweep = false;
    guard->sweep = (NavLidarStatus){0};
    memset(guard->previous, 0, sizeof guard->previous);
}

static void record_closest(NavLidarHit *closest, NavLidarHit hit)
{
    if (hit.distance_mm != 0u &&
        (closest->distance_mm == 0u || hit.distance_mm < closest->distance_mm)) {
        *closest = hit;
    }
}

static unsigned lidar_angle_gap(unsigned a, unsigned b)
{
    const unsigned gap = a > b ? a - b : b - a;
    return gap > LIDAR_FULL_TURN_Q6 / 2u ? LIDAR_FULL_TURN_Q6 - gap : gap;
}

static void lidar_check_zone(NavLidarGuard *guard, NavLidarZone zone,
                            NavLidarHit hit, uint32_t tick_ms, bool wide_pair)
{
    const NavLidarCandidate previous = guard->previous[zone];
    const unsigned gap_limit = wide_pair ? LIDAR_CORRIDOR_ANGLE_Q6 * 2u : LIDAR_PAIR_GAP_Q6;
    const bool pair = previous.hit.distance_mm != 0u && tick_ms - previous.tick_ms <= LIDAR_PAIR_MAX_MS &&
        lidar_angle_gap(hit.angle_q6, previous.hit.angle_q6) <= gap_limit;
    guard->previous[zone] = (NavLidarCandidate){ .hit = hit, .tick_ms = tick_ms };
    if (hit.distance_mm <= LIDAR_SINGLE_STOP_MM || pair) {
        if (pair) record_closest(&hit, previous.hit);
        record_closest(&guard->sweep.hazards[zone], hit);
        record_closest(&guard->status.hazards[zone], hit);
    }
}

void NavLidarObserve(NavLidarGuard *guard, uint16_t angle_q6, uint16_t distance_mm,
                     uint8_t quality, bool new_sweep, uint32_t tick_ms, int16_t front_q6)
{
    if (new_sweep) {
        if (guard->tracking_sweep && guard->coverage == LIDAR_FULL_COVERAGE &&
            guard->usable_returns >= 3u) {
            guard->sweep.scan_tick = tick_ms;
            guard->sweep.ready = true;
            guard->status = guard->sweep;
        }
        guard->tracking_sweep = true;
        guard->coverage = 0u;
        guard->usable_returns = 0u;
        guard->sweep = (NavLidarStatus){0};
    }
    guard->coverage |= 1u << (angle_q6 / LIDAR_COVERAGE_SECTOR_Q6);
    if (quality == 0u || distance_mm < LIDAR_MIN_MM || distance_mm > LIDAR_MAX_MM) return;
    if (guard->usable_returns < 3u) ++guard->usable_returns;
    const NavLidarHit hit = { .distance_mm = distance_mm, .angle_q6 = angle_q6 };
    /* The rear shell is closer than the room, so it must not become "nearest". */
    if (!NavLidarBodyShell(angle_q6, distance_mm, front_q6)) {
        record_closest(&guard->sweep.nearest, hit);
        record_closest(&guard->status.nearest, hit);
    }

    const bool calibrated = front_q6 >= 0 && (unsigned)front_q6 < LIDAR_FULL_TURN_Q6;
    if (distance_mm <= (calibrated ? LIDAR_NEAR_STOP_MM : LIDAR_PAIR_STOP_MM)) {
        lidar_check_zone(guard, LIDAR_NEAR, hit, tick_ms, false);
    }
    if (!calibrated || distance_mm > LIDAR_PAIR_STOP_MM) return;

    const unsigned from_front = lidar_angle_gap(angle_q6, (unsigned)front_q6);
    const bool behind = from_front > LIDAR_FULL_TURN_Q6 / 4u;
    const unsigned gap = behind ? LIDAR_FULL_TURN_Q6 / 2u - from_front : from_front;
    if (gap <= LIDAR_CORRIDOR_ANGLE_Q6 &&
        distance_mm * sinf((float)gap * (TWO_PI_F / LIDAR_FULL_TURN_Q6)) <= LIDAR_CORRIDOR_HALF_WIDTH_MM) {
        /* The forward corridor accepts any two returns inside it. A real
           object is wider than the 4 degree noise pair used around the body. */
        lidar_check_zone(guard, behind ? LIDAR_REAR : LIDAR_FRONT, hit, tick_ms, !behind);
    }
}

bool NavLidarFresh(const NavLidarStatus *status, uint32_t tick_ms)
{
    return status->ready && tick_ms - status->scan_tick <= LIDAR_STALE_MS;
}

NavLidarHit NavLidarHazard(const NavLidarStatus *status, NavMotion motion)
{
    NavLidarHit obstacle = status->hazards[LIDAR_NEAR];
    if (motion == NAV_MOTION_FORWARD || motion == NAV_MOTION_REVERSE) {
        record_closest(&obstacle, status->hazards[motion == NAV_MOTION_FORWARD ? LIDAR_FRONT : LIDAR_REAR]);
    }
    return obstacle;
}

static int lidar_signed_q6(unsigned raw, unsigned front)
{
    int relative = (int)raw - (int)front;
    while (relative > (int)(LIDAR_FULL_TURN_Q6 / 2u)) relative -= (int)LIDAR_FULL_TURN_Q6;
    while (relative < -(int)(LIDAR_FULL_TURN_Q6 / 2u)) relative += (int)LIDAR_FULL_TURN_Q6;
    return relative;
}

bool NavLidarInForwardCorridor(uint16_t angle_q6, uint16_t distance_mm, int16_t front_q6, uint16_t limit_mm)
{
    unsigned from_front;
    if (front_q6 < 0 || (unsigned)front_q6 >= LIDAR_FULL_TURN_Q6) return false;
    if (distance_mm == 0u || distance_mm > limit_mm) return false;
    from_front = lidar_angle_gap(angle_q6, (unsigned)front_q6);
    if (from_front > LIDAR_FULL_TURN_Q6 / 4u) return false;
    return from_front <= LIDAR_CORRIDOR_ANGLE_Q6 &&
        distance_mm * sinf((float)from_front * (TWO_PI_F / LIDAR_FULL_TURN_Q6)) <= LIDAR_CORRIDOR_HALF_WIDTH_MM;
}

bool NavLidarBodyShell(uint16_t angle_q6, uint16_t distance_mm, int16_t front_q6)
{
    int relative;
    if (front_q6 < 0 || (unsigned)front_q6 >= LIDAR_FULL_TURN_Q6) return false;
    if (distance_mm < 280u || distance_mm > 450u) return false;
    relative = lidar_signed_q6(angle_q6, (unsigned)front_q6);
    if (relative < 0) relative = -relative;
    return relative >= 100 * 64;
}

float NavAvoidYawRate(const NavLidarHit *front, int16_t front_q6, int8_t *pass_side)
{
    if (front == NULL || pass_side == NULL || front->distance_mm == 0u ||
        front_q6 < 0 || (unsigned)front_q6 >= LIDAR_FULL_TURN_Q6) {
        return 0.0f;
    }
    if (*pass_side == 0) {
        const int relative = lidar_signed_q6(front->angle_q6, (unsigned)front_q6);
        const int deadband = 8 * 64;
        if (relative < -deadband) *pass_side = -1;
        else *pass_side = 1;
    }
    return *pass_side > 0 ? 0.55f : -0.55f;
}

bool NavLidarBlocksPass(const NavLidarHit *hit, int16_t front_q6, bool passing)
{
    int relative;
    if (hit == NULL || hit->distance_mm == 0u) return false;
    if (hit->distance_mm <= 180u || !passing) return true;
    if (front_q6 < 0 || (unsigned)front_q6 >= LIDAR_FULL_TURN_Q6) return true;
    relative = lidar_signed_q6(hit->angle_q6, (unsigned)front_q6);
    if (relative < 0) relative = -relative;
    return relative <= 60 * 64;
}

static float clamp(float value, float low, float high)
{
    return fminf(high, fmaxf(low, value));
}

static float angle_wrap(float angle)
{
    float wrapped = fmodf(angle, TWO_PI_F);
    if (wrapped > PI_F) {
        wrapped -= TWO_PI_F;
    } else if (wrapped < -PI_F) {
        wrapped += TWO_PI_F;
    }
    return wrapped;
}

static bool valid_pose(const NavPose *pose)
{
    return pose != NULL && isfinite(pose->x_m) && isfinite(pose->y_m) &&
           isfinite(pose->yaw_rad);
}

static bool valid_elapsed(float dt_s)
{
    return isfinite(dt_s) && dt_s >= 0.001f && dt_s <= 0.050f;
}

static bool append_segment(NavPath *path, NavPoint from, NavPoint to, float spacing_m)
{
    const float distance = hypotf(to.x_m - from.x_m, to.y_m - from.y_m);
    const float divisions = fmaxf(1.0f, ceilf(distance / spacing_m));
    if (!isfinite(distance) || divisions > (float)(NAV_MAX_WAYPOINTS - path->count)) {
        return false;
    }

    const uint16_t steps = (uint16_t)divisions;
    for (uint16_t i = 1; i < steps; ++i) {
        const float fraction = (float)i / (float)steps;
        path->points[path->count++] = (NavPoint){
            from.x_m + fraction * (to.x_m - from.x_m),
            from.y_m + fraction * (to.y_m - from.y_m),
        };
    }
    path->points[path->count++] = to;
    return true;
}

bool NavGeneratePath(NavPath *path, const NavPose *start,
                     float x_m, float y_m, float spacing_m)
{
    if (path == NULL) {
        return false;
    }
    path->count = 0;
    if (!valid_pose(start) || !isfinite(x_m) || !isfinite(y_m) ||
        !isfinite(spacing_m) || spacing_m <= 0.0f) {
        return false;
    }

    NavPoint from = {start->x_m, start->y_m};
    const NavPoint target = {x_m, y_m};
    const NavPoint junction = {0.0f, CROSS_JUNCTION_Y_M};
    const bool cross_branch = fabsf(y_m - CROSS_JUNCTION_Y_M) <= COORDINATE_MATCH_M &&
                              fabsf(fabsf(x_m) - CROSS_BRANCH_X_M) <= COORDINATE_MATCH_M;
    if (cross_branch && hypotf(from.x_m - junction.x_m, from.y_m - junction.y_m) >
                            COORDINATE_MATCH_M &&
        hypotf(from.x_m - target.x_m, from.y_m - target.y_m) > NAV_ENDPOINT_TOLERANCE_M) {
        if (!append_segment(path, from, junction, spacing_m)) {
            return false;
        }
        from = junction;
    }

    if (!append_segment(path, from, target, spacing_m)) {
        path->count = 0;
        return false;
    }
    return true;
}

bool NavUpdatePose(NavPose *pose, float left_delta_m, float right_delta_m,
                   float wheelbase_m, float imu_yaw_rad, bool imu_valid)
{
    if (!valid_pose(pose) || !isfinite(left_delta_m) || !isfinite(right_delta_m) ||
        !isfinite(wheelbase_m) || wheelbase_m <= 0.0f) {
        return false;
    }

    const float previous_yaw = angle_wrap(pose->yaw_rad);
    const float encoder_turn = (right_delta_m - left_delta_m) / wheelbase_m;
    if (!isfinite(encoder_turn)) {
        return false;
    }
    float yaw = angle_wrap(previous_yaw + angle_wrap(encoder_turn));
    if (imu_valid && isfinite(imu_yaw_rad)) {
        const float innovation = angle_wrap(angle_wrap(imu_yaw_rad) - yaw);
        yaw = angle_wrap(yaw + IMU_CORRECTION * innovation);
    }

    const float midpoint_yaw = previous_yaw + 0.5f * angle_wrap(yaw - previous_yaw);
    const float travel = 0.5f * left_delta_m + 0.5f * right_delta_m;
    const NavPose next = {
        pose->x_m + travel * cosf(midpoint_yaw),
        pose->y_m + travel * sinf(midpoint_yaw),
        yaw,
    };
    if (!valid_pose(&next)) {
        return false;
    }
    *pose = next;
    return true;
}

void NavResetTracker(NavTracker *tracker)
{
    *tracker = (NavTracker){0};
}

void NavResetPid(NavPid *pid)
{
    *pid = (NavPid){0};
}

static float pid_step(NavPid *pid, float error, float dt_s, bool angular,
                      float kp, float ki, float kd, float feedforward,
                      float lower, float upper)
{
    if (pid->initialized) {
        const float difference = error - pid->previous_error;
        const float derivative = (angular ? angle_wrap(difference) : difference) / dt_s;
        pid->derivative += (dt_s / (0.04f + dt_s)) * (derivative - pid->derivative);
    }
    pid->previous_error = error;
    pid->initialized = true;

    const float possible_integral = clamp(pid->integral + ki * error * dt_s, -0.45f, 0.45f);
    const float base = feedforward + kp * error + kd * pid->derivative;
    const float possible_output = base + possible_integral;
    if ((possible_output <= upper || error < 0.0f) &&
        (possible_output >= lower || error > 0.0f)) {
        pid->integral = possible_integral;
    }
    return clamp(base + pid->integral, lower, upper);
}

static bool braking_waypoint(const NavPath *path, uint16_t index)
{
    if (index + 1U == path->count) {
        return true;
    }
    if (index == 0U) {
        return false;
    }

    const NavPoint previous = path->points[index - 1U];
    const NavPoint current = path->points[index];
    const NavPoint next = path->points[index + 1U];
    const float ax = current.x_m - previous.x_m;
    const float ay = current.y_m - previous.y_m;
    const float bx = next.x_m - current.x_m;
    const float by = next.y_m - current.y_m;
    /* Brake before a substantial turn so the turn does not cut across the corridor. */
    return ax * bx + ay * by < 0.85f * hypotf(ax, ay) * hypotf(bx, by);
}

NavTrackStatus NavTrackStep(NavTracker *tracker, const NavPath *path,
                           const NavPose *pose, float wheelbase_m, float dt_s,
                           float *left_mps, float *right_mps)
{
    if (left_mps == NULL || right_mps == NULL) {
        return NAV_TRACK_INVALID;
    }
    *left_mps = 0.0f;
    *right_mps = 0.0f;
    if (tracker == NULL || path == NULL || path->count == 0U ||
        path->count > NAV_MAX_WAYPOINTS || tracker->waypoint_index > path->count ||
        !valid_pose(pose) || !valid_elapsed(dt_s) ||
        !isfinite(wheelbase_m) || wheelbase_m <= 0.0f) {
        return NAV_TRACK_INVALID;
    }

    while (tracker->waypoint_index < path->count) {
        const uint16_t index = tracker->waypoint_index;
        const float dx = path->points[index].x_m - pose->x_m;
        const float dy = path->points[index].y_m - pose->y_m;
        const float distance = hypotf(dx, dy);
        if (!isfinite(distance)) {
            return NAV_TRACK_INVALID;
        }
        const float tolerance = index + 1U == path->count ?
                                NAV_ENDPOINT_TOLERANCE_M : WAYPOINT_TOLERANCE_M;
        if (distance <= tolerance) {
            ++tracker->waypoint_index;
            NavResetPid(&tracker->heading);
            continue;
        }

        const float yaw_error = angle_wrap(atan2f(dy, dx) - angle_wrap(pose->yaw_rad));
        const float yaw_rate = pid_step(&tracker->heading, yaw_error, dt_s, true,
                                        3.0f, 0.12f, 0.035f, 0.0f,
                                        -MAX_YAW_RATE_RAD_S, MAX_YAW_RATE_RAD_S);
        float speed = MAX_LINEAR_MPS;
        if (braking_waypoint(path, index)) {
            speed = fminf(speed, BRAKING_GAIN_PER_S * distance);
        }
        const float forward = speed * fmaxf(0.0f, 1.0f - fabsf(yaw_error) / TURN_IN_PLACE_RAD);
        const float turn = yaw_rate * 0.5f * wheelbase_m;
        const float left = forward - turn;
        const float right = forward + turn;
        const float peak = fmaxf(fabsf(left), fabsf(right));
        const float scale = peak > MAX_WHEEL_MPS ? MAX_WHEEL_MPS / peak : 1.0f;
        *left_mps = scale * left;
        *right_mps = scale * right;
        return NAV_TRACK_RUNNING;
    }
    return NAV_TRACK_ARRIVED;
}

float NavMotorStep(NavPid *pid, float target_mps, float actual_mps, float dt_s)
{
    if (pid == NULL) {
        return 0.0f;
    }
    if (!valid_elapsed(dt_s) || !isfinite(target_mps) || !isfinite(actual_mps) ||
        fabsf(target_mps) < 0.001f) {
        NavResetPid(pid);
        return 0.0f;
    }

    const float target = clamp(target_mps, -MAX_WHEEL_MPS, MAX_WHEEL_MPS);
    const float error = target - actual_mps;
    const float feedforward = 1.8f * target + copysignf(0.08f, target);
    const float lower = target > 0.0f ? 0.0f : -1.0f;
    const float upper = target > 0.0f ? 1.0f : 0.0f;
    return pid_step(pid, error, dt_s, false, 1.1f, 2.2f, 0.012f, feedforward, lower, upper);
}
