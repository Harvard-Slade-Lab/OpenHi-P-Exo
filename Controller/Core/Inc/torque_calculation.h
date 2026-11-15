#ifndef INC_TORQUE_CALCULATION_H_
#define INC_TORQUE_CALCULATION_H_

void get_torques(float loadLeft, float loadRight,
                 float meanVoltageLeft, float meanVoltageRight,
                 float *torqueMeasuredLeft, float *torqueMeasuredRight);

#endif /* INC_TORQUE_CALCULATION_H_ */