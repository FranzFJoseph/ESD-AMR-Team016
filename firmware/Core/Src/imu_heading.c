#include "imu_heading.h"

#include <math.h>
#include <stddef.h>

#define PI_F 3.14159265359f
#define TWO_PI_F (2.0f * PI_F)
#define DEG_TO_RAD_F (PI_F / 180.0f)
#define CALIBRATION_SAMPLES 200u
/* This module's resting gyro noise is well above 0.2 dps. The mean of the
   200-sample window is still a usable bias; only reject a window that is
   actually swinging. */
#define CALIBRATION_MAX_VARIANCE_DPS2 4.0f
#define MIN_SAMPLE_US 1000u
#define MAX_SAMPLE_US 50000u
#define MAX_STATIONARY_SAMPLE_US 30000u
#define STOPPED_SETTLE_US 1000000u
#define BIAS_TIME_CONSTANT_US 2000000.0f

static float wrap_yaw(float yaw_rad)
{
    float angle = fmodf(yaw_rad, TWO_PI_F);
    if (angle > PI_F) angle -= TWO_PI_F;
    if (angle < -PI_F) angle += TWO_PI_F;
    return angle;
}

void ImuHeadingInit(ImuHeading *heading, float initial_yaw_rad)
{
    if (heading == NULL) return;
    *heading = (ImuHeading){
        .yaw_rad = isfinite(initial_yaw_rad) ? wrap_yaw(initial_yaw_rad) : 0.0f,
    };
}

void ImuHeadingUpdate(ImuHeading *heading, float rate_dps, float accel_norm2,
                      uint32_t elapsed_us, bool stationary_allowed, bool wheels_still)
{
    if (heading == NULL) return;
    const bool valid_sample = isfinite(rate_dps) && isfinite(accel_norm2) &&
                              elapsed_us >= MIN_SAMPLE_US && elapsed_us <= MAX_SAMPLE_US;
    const bool at_rest = valid_sample && elapsed_us <= MAX_STATIONARY_SAMPLE_US &&
                         stationary_allowed && wheels_still &&
                         accel_norm2 >= 0.81f && accel_norm2 <= 1.21f;
    if (!heading->calibrated) {
        if (!at_rest || fabsf(rate_dps) > 5.0f) {
            heading->calibration_samples = 0u;
            heading->calibration_mean_dps = 0.0f;
            heading->calibration_m2 = 0.0f;
            return;
        }

        ++heading->calibration_samples;
        const float delta = rate_dps - heading->calibration_mean_dps;
        heading->calibration_mean_dps += delta / heading->calibration_samples;
        heading->calibration_m2 += delta * (rate_dps - heading->calibration_mean_dps);
        if (heading->calibration_samples == CALIBRATION_SAMPLES) {
            const float variance = heading->calibration_m2 / (CALIBRATION_SAMPLES - 1u);
            if (variance <= CALIBRATION_MAX_VARIANCE_DPS2) {
                heading->bias_dps = heading->calibration_mean_dps;
                heading->calibrated = true;
            }
            heading->calibration_samples = 0u;
            heading->calibration_mean_dps = 0.0f;
            heading->calibration_m2 = 0.0f;
        }
        return;
    }

    const float corrected_rate = rate_dps - heading->bias_dps;
    if (at_rest && fabsf(corrected_rate) <= 0.8f) {
        if (heading->stopped_us < STOPPED_SETTLE_US) {
            heading->stopped_us += elapsed_us;
        }
        heading->stationary = heading->stopped_us >= STOPPED_SETTLE_US;
    } else {
        heading->stopped_us = 0u;
        heading->stationary = false;
    }
    if (!valid_sample) return;

    if (heading->stationary) {
        const float alpha = (float)elapsed_us / (BIAS_TIME_CONSTANT_US + elapsed_us);
        heading->bias_dps += alpha * corrected_rate;
    } else {
        const float delta_yaw = corrected_rate * DEG_TO_RAD_F * (float)elapsed_us * 1.0e-6f;
        heading->yaw_rad = wrap_yaw(heading->yaw_rad + delta_yaw);
    }
}
