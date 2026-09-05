#include <math.h>
#include <string.h>
#include <stdint.h>

#include "torque_profile.h"

#define PARAM_COUNT 6

static float profileStart[PARAM_COUNT] = {0};
static float profileTarget[PARAM_COUNT] = {0};
static float profileUpdateDuration = 5.0f;
static float profileUpdateStartTime = 0.0f;
static uint8_t isProfileUpdating = 0;

void torque_profile_init(TorqueProfile *profile)
{
    // // Initial Profile
    profile->extensionPeakTorque = -12.0f;
    profile->flexionPeakTorque = 8.0f;
    profile->extensionPeakTime = 20.0f;
    profile->flexionPeakTime = 72.0f;
    profile->midTime = 38.0f;
    profile->extensionRiseTime = 25.0f;

    memset(profile->d, 0, sizeof(profile->d));

    torque_profile_update(profile);
}

void torque_profile_update(TorqueProfile *profile)
{
    profile->x[0] = profile->extensionPeakTime - profile->extensionRiseTime;
    profile->x[1] = profile->extensionPeakTime;
    profile->x[2] = profile->midTime;
    profile->x[3] = profile->flexionPeakTime;
    profile->x[4] = 100 - profile->extensionRiseTime + profile->extensionPeakTime;

    profile->y[0] = 0;
    profile->y[1] = profile->extensionPeakTorque;
    profile->y[2] = 0;
    profile->y[3] = profile->flexionPeakTorque;
    profile->y[4] = 0;

    // Calculate Torque Profile Coefficients for each segment
    for (int i = 0; i < NUM_SEGMENTS; i++)
    {
        hermite_cubic_to_power_cubic(profile->x[i], profile->y[i], profile->d[i], profile->x[i + 1], profile->y[i + 1], profile->d[i + 1], &profile->c0[i], &profile->c1[i], &profile->c2[i], &profile->c3[i]);
    }
}

void torque_profile_set_target(float *params)
{
    for (int i = 0; i < PARAM_COUNT; i++)
    {
        profileTarget[i] = params[i];
    }
}

void torque_profile_update_trigger(TorqueProfile *profile, float currentTimeSec)
{
    profileStart[0] = profile->extensionPeakTorque;
    profileStart[1] = profile->flexionPeakTorque;
    profileStart[2] = profile->extensionPeakTime;
    profileStart[3] = profile->flexionPeakTime;
    profileStart[4] = profile->midTime;
    profileStart[5] = profile->extensionRiseTime;

    profileUpdateStartTime = currentTimeSec;
    isProfileUpdating = 1;
}

void torque_profile_update_continuous(TorqueProfile *profile, float currentTimeSec)
{
    if (!isProfileUpdating)
        return;

    float alpha = (currentTimeSec - profileUpdateStartTime) / profileUpdateDuration;
    if (alpha >= 1.0f)
    {
        alpha = 1.0f;
        isProfileUpdating = 0;
    }

    float interpolated[PARAM_COUNT];
    for (int i = 0; i < PARAM_COUNT; i++)
    {
        interpolated[i] = profileStart[i] + alpha * (profileTarget[i] - profileStart[i]);
    }

    profile->extensionPeakTorque = interpolated[0];
    profile->flexionPeakTorque = interpolated[1];
    profile->extensionPeakTime = interpolated[2];
    profile->flexionPeakTime = interpolated[3];
    profile->midTime = interpolated[4];
    profile->extensionRiseTime = interpolated[5];

    torque_profile_update(profile);
}

void hermite_cubic_to_power_cubic(float x1, float f1, float d1, float x2, float f2, float d2, float *c0, float *c1, float *c2, float *c3)
{
    float h = x2 - x1;
    float df = (f2 - f1) / h;

    *c0 = f1;
    *c1 = d1;
    *c2 = -(2.0 * d1 - 3.0 * df + d2) / h;
    *c3 = (d1 - 2.0 * df + d2) / (h * h);
}

float evaluate_hermite(float x, float c0, float c1, float c2, float c3)
{
    return c0 + c1 * x + c2 * x * x + c3 * x * x * x;
}
