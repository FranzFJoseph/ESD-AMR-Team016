#include "imu_heading.h"

#include <math.h>

#define PI_F 3.14159265359f

static bool near(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static bool near_angle(float actual, float expected, float tolerance)
{
    return near(remainderf(actual - expected, 2.0f * PI_F), 0.0f, tolerance);
}

static void calibrate(ImuHeading *heading)
{
    for (unsigned i = 0u; i < 200u; ++i) {
        ImuHeadingUpdate(heading, 0.25f, 1.0f, 10000u, true, true);
    }
}

int main(void)
{
    ImuHeading heading;
    ImuHeadingInit(&heading, 0.5f * PI_F);
    for (unsigned i = 0u; i < 199u; ++i) {
        ImuHeadingUpdate(&heading, 0.25f, 1.0f, 10000u, true, true);
    }
    if (heading.calibrated) return 1;
    ImuHeadingUpdate(&heading, 0.25f, 1.0f, 10000u, true, false);
    if (heading.calibration_samples != 0u) return 2;
    calibrate(&heading);
    if (!heading.calibrated || !near(heading.bias_dps, 0.25f, 0.0001f) ||
        !near_angle(heading.yaw_rad, 0.5f * PI_F, 0.0001f)) return 3;

    /* One second of a real left turn retains the original world frame and bias. */
    for (unsigned i = 0u; i < 100u; ++i) {
        ImuHeadingUpdate(&heading, 90.25f, 1.0f, 10000u, false, false);
    }
    if (!near_angle(heading.yaw_rad, PI_F, 0.001f) || heading.stationary ||
        !near(heading.bias_dps, 0.25f, 0.0001f)) return 4;

    /* A stopped dwell holds yaw while a small temperature drift is absorbed. */
    for (unsigned i = 0u; i < 100u; ++i) {
        ImuHeadingUpdate(&heading, 0.65f, 1.0f, 10000u, true, true);
    }
    if (!heading.stationary) return 5;
    const float stopped_yaw = heading.yaw_rad;
    for (unsigned i = 0u; i < 1000u; ++i) {
        ImuHeadingUpdate(&heading, 0.65f, 1.0f, 10000u, true, true);
    }
    if (heading.yaw_rad != stopped_yaw || !near(heading.bias_dps, 0.65f, 0.005f)) return 6;
    const float stopped_bias = heading.bias_dps;
    ImuHeadingUpdate(&heading, -89.35f, 1.0f, 10000u, false, false);
    if (heading.stationary || !near_angle(heading.yaw_rad, stopped_yaw - 0.005f * PI_F, 0.0001f) ||
        heading.bias_dps != stopped_bias) return 7;

    /* Rate variation and invalid inputs cannot complete a startup window. */
    ImuHeadingInit(&heading, 0.5f * PI_F);
    for (unsigned i = 0u; i < 200u; ++i) {
        ImuHeadingUpdate(&heading, (i & 1u) ? 3.0f : -3.0f, 1.0f, 10000u, true, true);
    }
    if (heading.calibrated) return 8;
    ImuHeadingUpdate(&heading, 0.25f, 1.0f, 999u, true, true);
    if (heading.calibration_samples != 0u) return 9;
    ImuHeadingUpdate(&heading, 0.25f, 1.0f, 10000u, false, true);
    if (heading.calibration_samples != 0u) return 10;
    ImuHeadingUpdate(&heading, 0.25f, NAN, 10000u, true, true);
    if (heading.calibration_samples != 0u) return 11;
    calibrate(&heading);
    if (!heading.calibrated) return 12;
    const float initial_yaw = heading.yaw_rad;
    const float initial_bias = heading.bias_dps;
    ImuHeadingUpdate(&heading, INFINITY, 1.0f, 10000u, true, true);
    ImuHeadingUpdate(&heading, 30.0f, 1.0f, 50001u, false, false);
    if (heading.yaw_rad != initial_yaw || heading.bias_dps != initial_bias || heading.stationary) return 13;

    /* Wheel motion alone breaks stationary hold even at a small yaw rate. */
    for (unsigned i = 0u; i < 100u; ++i) {
        ImuHeadingUpdate(&heading, 0.25f, 1.0f, 10000u, true, true);
    }
    if (!heading.stationary) return 14;
    ImuHeadingUpdate(&heading, 0.5f, 1.0f, 10000u, true, false);
    if (heading.stationary || heading.stopped_us != 0u || heading.yaw_rad <= initial_yaw) return 15;
    const float moving_yaw = heading.yaw_rad;
    ImuHeadingUpdate(&heading, 30.25f, 1.0f, 40000u, false, false);
    if (!near_angle(heading.yaw_rad, moving_yaw + 1.2f * PI_F / 180.0f, 0.0001f)) return 16;
    return 0;
}
