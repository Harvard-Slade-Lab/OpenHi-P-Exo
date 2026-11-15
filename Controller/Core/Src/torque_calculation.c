#include <math.h>

#include "torque_calculation.h"

void get_torques(float loadLeft, float loadRight,
                 float meanVoltageLeft, float meanVoltageRight,
                 float *torqueMeasuredLeft, float *torqueMeasuredRight)
{
    // -------- Static filter state --------
    static float loadLeftPrev1 = 0.0f, loadLeftPrev2 = 0.0f;
    static float loadLeftFiltPrev1 = 0.0f, loadLeftFiltPrev2 = 0.0f;

    static float loadRightPrev1 = 0.0f, loadRightPrev2 = 0.0f;
    static float loadRightFiltPrev1 = 0.0f, loadRightFiltPrev2 = 0.0f;

    // -------- Filter parameters (2nd-order Butterworth) --------
    const float freqSampling = 1000.0f;
    const float freqCutoff = 5.0f;

    const float wc = 2.0f * M_PI * freqCutoff / freqSampling;
    const float K = tanf(wc / 2.0f);
    const float norm = K * K + sqrtf(2.0f) * K + 1.0f;

    const float a0 = (K * K) / norm;
    const float a1 = 2.0f * a0;
    const float a2 = a0;
    const float b1 = 2.0f * (K * K - 1.0f) / norm;
    const float b2 = (K * K - sqrtf(2.0f) * K + 1.0f) / norm;

    // -------- Sensor parameters --------
    const float gain = 375.53f;
    const float excitation = 3.3f;

    const float sensitivityLeft = 2.1662f;  // [mV/V]
    const float sensitivityRight = 2.1674f; // [mV/V]

    const float maxLoad = 18.1437f; // [kgf]
    const float kgf_to_N = 9.80665f;
    const float maxLoad_N = maxLoad * kgf_to_N;

    const float sensitivityLeft_V_per_N = (sensitivityLeft / 1000.0f) * gain * excitation / maxLoad_N;
    const float sensitivityRight_V_per_N = (sensitivityRight / 1000.0f) * gain * excitation / maxLoad_N;

    const float momentArm = 0.36f; // [m]

    // -------- Left Channel Filtering --------
    float loadLeftFilt =
        a0 * loadLeft +
        a1 * loadLeftPrev1 +
        a2 * loadLeftPrev2 -
        b1 * loadLeftFiltPrev1 -
        b2 * loadLeftFiltPrev2;

    float outputVoltLeft = loadLeftFilt * 3.3f / 4095.0f;
    float forceLeft = (outputVoltLeft - meanVoltageLeft) / sensitivityLeft_V_per_N;
    *torqueMeasuredLeft = forceLeft * momentArm;

    // -------- Right Channel Filtering --------
    float loadRightFilt =
        a0 * loadRight +
        a1 * loadRightPrev1 +
        a2 * loadRightPrev2 -
        b1 * loadRightFiltPrev1 -
        b2 * loadRightFiltPrev2;

    float outputVoltRight = loadRightFilt * 3.3f / 4095.0f;
    float forceRight = (outputVoltRight - meanVoltageRight) / sensitivityRight_V_per_N;
    *torqueMeasuredRight = forceRight * momentArm;

    // -------- Update filter state --------
    loadLeftPrev2 = loadLeftPrev1;
    loadLeftPrev1 = loadLeft;

    loadLeftFiltPrev2 = loadLeftFiltPrev1;
    loadLeftFiltPrev1 = loadLeftFilt;

    loadRightPrev2 = loadRightPrev1;
    loadRightPrev1 = loadRight;

    loadRightFiltPrev2 = loadRightFiltPrev1;
    loadRightFiltPrev1 = loadRightFilt;
}