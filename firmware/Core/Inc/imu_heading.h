#ifndef PART2_IMU_HEADING_H
#define PART2_IMU_HEADING_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float yaw_rad;
    float bias_dps;
    bool calibrated;
    bool stationary;
    uint16_t calibration_samples;
    float calibration_mean_dps;
    float calibration_m2;
    uint32_t stopped_us;
} ImuHeading;

/* Initialization establishes the caller's heading frame. A nonfinite heading uses zero. */
void ImuHeadingInit(ImuHeading *heading, float initial_yaw_rad);

/*
 * rate_dps follows the caller's yaw sign. elapsed_us is the time between samples.
 * stationary_allowed should require a stopped controller, and wheels_still should
 * represent an encoder observation window rather than a single missing pulse.
 * Keep the vehicle physically still during startup calibration.
 */
void ImuHeadingUpdate(ImuHeading *heading, float rate_dps, float accel_norm2,
                      uint32_t elapsed_us, bool stationary_allowed, bool wheels_still);

#endif
