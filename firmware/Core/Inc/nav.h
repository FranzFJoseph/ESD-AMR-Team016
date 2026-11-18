#ifndef CODING_DAY4_NAV_H
#define CODING_DAY4_NAV_H

#include <stdbool.h>
#include <stdint.h>

#define NAV_MAX_WAYPOINTS 64U
#define NAV_INITIAL_YAW_RAD 1.57079632679f
#define NAV_ENDPOINT_TOLERANCE_M 0.040f

typedef struct {
    float x_m;
    float y_m;
} NavPoint;

/* The start faces +y. Heading is measured counterclockwise from +x. */
typedef struct {
    float x_m;
    float y_m;
    float yaw_rad;
} NavPose;

/* Points exclude the start and include the exact destination. */
typedef struct {
    uint16_t count;
    NavPoint points[NAV_MAX_WAYPOINTS];
} NavPath;

typedef struct {
    float integral;
    float previous_error;
    float derivative;
    bool initialized;
} NavPid;

typedef struct {
    uint16_t waypoint_index;
    NavPid heading;
} NavTracker;

typedef enum {
    NAV_TRACK_INVALID = 0,
    NAV_TRACK_RUNNING,
    NAV_TRACK_ARRIVED,
} NavTrackStatus;

bool NavGeneratePath(NavPath *path, const NavPose *start,
                     float x_m, float y_m, float spacing_m);

/* Positive wheel travel moves the car forward. The IMU yaw uses the pose frame. */
bool NavUpdatePose(NavPose *pose, float left_delta_m, float right_delta_m,
                   float wheelbase_m, float imu_yaw_rad, bool imu_valid);

void NavResetTracker(NavTracker *tracker);
NavTrackStatus NavTrackStep(NavTracker *tracker, const NavPath *path,
                           const NavPose *pose, float wheelbase_m, float dt_s,
                           float *left_mps, float *right_mps);

void NavResetPid(NavPid *pid);
/* A zero target returns zero effort immediately. Nonzero duty is in [-1, 1]. */
float NavMotorStep(NavPid *pid, float target_mps, float actual_mps, float dt_s);

typedef enum {
    NAV_MOTION_IDLE, NAV_MOTION_FORWARD, NAV_MOTION_REVERSE, NAV_MOTION_LEFT, NAV_MOTION_RIGHT
} NavMotion;

typedef struct {
    uint16_t distance_mm;
    uint16_t angle_q6;
} NavLidarHit;

typedef enum { LIDAR_NEAR, LIDAR_FRONT, LIDAR_REAR, LIDAR_ZONE_COUNT } NavLidarZone;

typedef struct {
    uint32_t scan_tick;
    NavLidarHit nearest;
    NavLidarHit hazards[LIDAR_ZONE_COUNT];
    bool ready;
} NavLidarStatus;

typedef struct {
    NavLidarHit hit;
    uint32_t tick_ms;
} NavLidarCandidate;

typedef struct {
    NavLidarStatus status;
    NavLidarStatus sweep;
    NavLidarCandidate previous[LIDAR_ZONE_COUNT];
    uint32_t coverage;
    uint8_t usable_returns;
    bool tracking_sweep;
} NavLidarGuard;

/* Discard a partial sweep without clearing a confirmed obstacle. */
void NavLidarResetSweep(NavLidarGuard *guard);
/* Q6 front angle of the chassis, or -1 when uncalibrated. */
void NavLidarObserve(NavLidarGuard *guard, uint16_t angle_q6, uint16_t distance_mm,
                     uint8_t quality, bool new_sweep, uint32_t tick_ms, int16_t front_q6);
/* Close returns more than 100 degrees off the nose are the chassis, not the map. */
bool NavLidarBodyShell(uint16_t angle_q6, uint16_t distance_mm, int16_t front_q6);
/* True when the return is ahead of the nose, inside the driving width, and not farther than limit_mm. */
bool NavLidarInForwardCorridor(uint16_t angle_q6, uint16_t distance_mm, int16_t front_q6, uint16_t limit_mm);
bool NavLidarFresh(const NavLidarStatus *status, uint32_t tick_ms);
NavLidarHit NavLidarHazard(const NavLidarStatus *status, NavMotion motion);
/* Positive yaw rate turns left, away from a static obstacle on the right.
   pass_side is +1 to keep the obstacle on the right, -1 to keep it on the left,
   and 0 until the first front-corridor hit chooses a side. A later hit does not
   flip the side. Returns 0 when the front corridor is clear. */
float NavAvoidYawRate(const NavLidarHit *front, int16_t front_q6, int8_t *pass_side);
/* Near-zone hits stop the car. While a pass is in progress, a hit beside the
   car and farther than 180 mm is the obstacle being passed, not a new stop. */
bool NavLidarBlocksPass(const NavLidarHit *hit, int16_t front_q6, bool passing);

#endif
