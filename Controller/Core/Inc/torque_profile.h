#ifndef INC_TORQUE_PROFILE_H_
#define INC_TORQUE_PROFILE_H_

#include "globals.h"

void torque_profile_init(TorqueProfile *profile);
void torque_profile_set_target(float* params);

void torque_profile_update(TorqueProfile *profile);
void torque_profile_update_trigger(TorqueProfile* profile, float currentTimeSec);
void torque_profile_update_continuous(TorqueProfile* profile, float currentTimeSec);

void hermite_cubic_to_power_cubic(float x1, float f1, float d1, float x2, float f2, float d2, float *c0, float *c1, float *c2, float *c3);
float evaluate_hermite(float x, float c0, float c1, float c2, float c3);

#endif /* INC_TORQUE_PROFILE_H_ */
