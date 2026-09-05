#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#include "control_logic.h"
#include "torque_profile.h"

void update_control_mode(uint8_t *controlMode)
{
	if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8))
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		*controlMode = 0;
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		*controlMode = 1;
	}
}

#define IMU_SMOOTH_NUM 30 // 100 Hz - 0.3 s

#define STEP_TRACK_NUM 3

// Left
static float angleRawArrayLeft[IMU_SMOOTH_NUM] = {0};
static float velocityRawArrayLeft[IMU_SMOOTH_NUM] = {0};
static float angleFiltArrayLeft[2] = {0};
static float velocityFiltArrayLeft[2] = {0};
static float phaseAngleFiltLeft = 0.0f;
static float phaseAngleFiltPrevLeft = 0.0f;
static float angleSmoothArrayLeft[3] = {0};
static float velocitySmoothArrayLeft[3] = {0};
static float phaseAngleSmoothLeft = 0.0f;
static float phaseAngleSmoothPrevLeft = 0.0f;

static uint16_t istrideLeft = 0;
static float stepPrevTimeLeft = 0.0f;
static float stepDurationsLeft[STEP_TRACK_NUM] = {0};
static bool startMoveFlagLeft = false;
static float walkingStopTimeLeft = 0.0f;
static bool noMoveFlagLeft = false;
static float noMoveTimeLeft = 0.0f;
static bool phaseFiltDropFlagLeft = false;
static float phaseFiltDropTimeLeft = 0.0f;

static bool angleMinFlagLeft = true;
static bool angleMaxFlagLeft = false;
static bool velocityMinFlagLeft = true;
static bool velocityMaxFlagLeft = false;

static float angleMinLeft = 0.0f;
static float angleMaxLeft = 0.0f;
static float angleCenterFactorLeft = 0.0f;
static float angleSizeFactorLeft = 100.0f;
static float angleNormLeft = 0.0f;

static float velocityMinLeft = 0.0f;
static float velocityMaxLeft = 0.0f;
static float velocityCenterFactorLeft = 0.0f;
static float velocitySizeFactorLeft = 100.0f;
static float velocityNormLeft = 0.0f;

static float phaseAngleNormLeft = 0.0f;
static float phaseRadiusLeft = 0.0f;

// Right
static float angleRawArrayRight[IMU_SMOOTH_NUM] = {0};
static float velocityRawArrayRight[IMU_SMOOTH_NUM] = {0};
static float angleFiltArrayRight[2] = {0};
static float velocityFiltArrayRight[2] = {0};
static float phaseAngleFiltRight = 0.0f;
static float phaseAngleFiltPrevRight = 0.0f;
static float angleSmoothArrayRight[3] = {0};
static float velocitySmoothArrayRight[3] = {0};
static float phaseAngleSmoothRight = 0.0f;
static float phaseAngleSmoothPrevRight = 0.0f;

static uint16_t istrideRight = 0;
static float stepPrevTimeRight = 0.0f;
static float stepDurationsRight[STEP_TRACK_NUM] = {0};

static bool startMoveFlagRight = false;
static float walkingStopTimeRight = 0.0f;
static bool noMoveFlagRight = false;
static float noMoveTimeRight = 0.0f;
static bool phaseFiltDropFlagRight = false;
static float phaseFiltDropTimeRight = 0.0f;

static bool angleMinFlagRight = true;
static bool angleMaxFlagRight = false;
static bool velocityMinFlagRight = true;
static bool velocityMaxFlagRight = false;

static float angleMinRight = 0.0f;
static float angleMaxRight = 0.0f;
static float angleCenterFactorRight = 0.0f;
static float angleSizeFactorRight = 100.0f;
static float angleNormRight = 0.0f;

static float velocityMinRight = 0.0f;
static float velocityMaxRight = 0.0f;
static float velocityCenterFactorRight = 0.0f;
static float velocitySizeFactorRight = 100.0f;
static float velocityNormRight = 0.0f;

static float phaseAngleNormRight = 0.0f;
static float phaseRadiusRight = 0.0f;

void process_leg(float timeSec, float imuMean, IMUData *imuData, float *gaitCycleNorm,
				 bool *walkingFlag, TorqueProfile *profile, float *torqueDesired,
				 bool *startMoveFlag, bool *noMoveFlag,
				 bool *angleMinFlag, bool *angleMaxFlag,
				 bool *velocityMinFlag, bool *velocityMaxFlag,
				 float *walkingStopTime, float *noMoveTime,
				 float *angleRawArray, float *velocityRawArray,
				 float *angleFiltArray, float *velocityFiltArray,
				 float *phaseAngleFilt, float *phaseAngleFiltPrev,
				 float *angleSmoothArray, float *velocitySmoothArray,
				 float *phaseAngleSmooth, float *phaseAngleSmoothPrev,
				 float *angleMin, float *angleMax,
				 float *angleCenterFactor, float *angleSizeFactor,
				 float *velocityMin, float *velocityMax,
				 float *velocityCenterFactor, float *velocitySizeFactor,
				 float *angleNorm, float *velocityNorm,
				 float *phaseAngleNorm, float *phaseRadius,
				 float *stepPrevTime, float *stepDurations,
				 bool *phaseFiltDropFlag, float *phaseFiltDropTime,
				 uint16_t *istride)
{

	const float startMoveVelThreshold = 0.7f;
	const float noMoveVelThreshold = 0.3f;
	const float noMoveTimeThreshold = 0.5f; // [s]
	const float velocityPeakThreshold = 1.0f;
	const float radiusThreshold = 0.4f;
	const float pauseTimeThreshold = 1.0f; // [s]

	const uint16_t freqSampling = 100;
	const uint16_t freqCutoff = 7;
	const float T = 1.0f / freqSampling;
	const float cutoff = 2.0f * (float)M_PI * freqCutoff;
	const float a = cutoff / (2.0f / T + cutoff);
	const float b = a;
	const float c = (2.0f / T - cutoff) / (2.0f / T + cutoff);

	float angleRaw, velocityRaw;

	angleRaw = imuData->roll - imuMean;	
	velocityRaw = -imuData->gyroX;

	// Shifting Raw Data
	for (uint8_t i = 0; i < IMU_SMOOTH_NUM - 1; i++)
	{
		angleRawArray[i] = angleRawArray[i + 1];
		velocityRawArray[i] = velocityRawArray[i + 1];
	}

	angleRawArray[IMU_SMOOTH_NUM - 1] = angleRaw;
	velocityRawArray[IMU_SMOOTH_NUM - 1] = velocityRaw;

	angleFiltArray[1] = a * angleRawArray[IMU_SMOOTH_NUM - 1] + b * angleRawArray[IMU_SMOOTH_NUM - 2] + c * angleFiltArray[0];
	velocityFiltArray[1] = a * velocityRawArray[IMU_SMOOTH_NUM - 1] + b * velocityRawArray[IMU_SMOOTH_NUM - 2] + c * velocityFiltArray[0];

	angleFiltArray[0] = angleFiltArray[1];
	velocityFiltArray[0] = velocityFiltArray[1];

	*phaseAngleFilt = atan2f(velocityFiltArray[1], angleFiltArray[1] - *angleCenterFactor);
	if (*phaseAngleFilt < 0.0f)
		*phaseAngleFilt += 2.0f * (float)M_PI;

	// Summing Raw Data
	float sumAng = 0.0f;
	float sumVel = 0.0f;

	for (uint8_t i = 0; i < IMU_SMOOTH_NUM; i++)
	{
		sumAng += angleRawArray[i];
		sumVel += velocityRawArray[i];
	}

	// Smoothing - Average
	float angleSmooth = sumAng / (float)IMU_SMOOTH_NUM;
	float velocitySmooth = sumVel / (float)IMU_SMOOTH_NUM;

	for (uint8_t i = 0; i < 2; i++)
	{
		angleSmoothArray[i] = angleSmoothArray[i + 1];
		velocitySmoothArray[i] = velocitySmoothArray[i + 1];
	}

	angleSmoothArray[2] = angleSmooth;
	velocitySmoothArray[2] = velocitySmooth;

	*phaseAngleSmooth = atan2f(velocitySmooth, angleSmooth - *angleCenterFactor);

	if (*phaseAngleSmooth < 0.0f)
		*phaseAngleSmooth += 2.0f * (float)M_PI;

	// Walking Start Detection
	if (fabsf(velocitySmooth) > startMoveVelThreshold && (timeSec - *walkingStopTime) > pauseTimeThreshold)
	{
		*startMoveFlag = true;
	}

	// Real Walking (Gait) Detection
	if (*startMoveFlag && (timeSec - *stepPrevTime) > 0.5f)
	{
		if (!*phaseFiltDropFlag && (*phaseAngleFiltPrev - *phaseAngleFilt) > 4.0f)
		{
			*phaseFiltDropFlag = true;
			*phaseFiltDropTime = timeSec;
		}
		if ((*phaseFiltDropFlag && (timeSec - *phaseFiltDropTime) > 0.25f) || (*phaseAngleSmoothPrev - *phaseAngleSmooth) > 4.0f)
		{
			*walkingFlag = true;

			if (*istride >= 1)
			{
				for (uint8_t i = 0; i < STEP_TRACK_NUM - 1; i++)
				{
					stepDurations[i] = stepDurations[i + 1];
				}
				stepDurations[STEP_TRACK_NUM - 1] = timeSec - *stepPrevTime;
			}

			(*istride)++;
			*stepPrevTime = timeSec;

			*phaseFiltDropFlag = false;
		}
	}

	*phaseAngleFiltPrev = *phaseAngleFilt;
	*phaseAngleSmoothPrev = *phaseAngleSmooth;

	// Calculating Normalization Parameters
	if (*angleMaxFlag)
	{
		if ((angleSmoothArray[1] < angleSmoothArray[2]) && (angleSmoothArray[1] < angleSmoothArray[0]) && (angleSmoothArray[1] < *angleMax - 5.0f))
		{
			*angleMin = angleSmoothArray[1];

			*angleMinFlag = true;
			*angleMaxFlag = false;
		}
	}
	if (*angleMinFlag)
	{
		if ((angleSmoothArray[1] > angleSmoothArray[2]) && (angleSmoothArray[1] > angleSmoothArray[0]) && (angleSmoothArray[1] > *angleMin + 5.0f))
		{
			*angleMax = angleSmoothArray[1];

			*angleMinFlag = false;
			*angleMaxFlag = true;
		}
	}
	if (*velocityMaxFlag)
	{
		if ((velocitySmoothArray[1] < velocitySmoothArray[2]) && (velocitySmoothArray[1] < velocitySmoothArray[0]) && (velocitySmoothArray[1] < -velocityPeakThreshold))
		{
			*velocityMin = velocitySmoothArray[1];

			*velocityMinFlag = true;
			*velocityMaxFlag = false;
		}
	}
	if (*velocityMinFlag)
	{
		if ((velocitySmoothArray[1] > velocitySmoothArray[2]) && (velocitySmoothArray[1] > velocitySmoothArray[0]) && (velocitySmoothArray[1] > velocityPeakThreshold))
		{
			*velocityMax = velocitySmoothArray[1];

			*velocityMinFlag = false;
			*velocityMaxFlag = true;
		}
	}

	// Update Normalization Parameter
	if (*istride > 1)
	{
		*angleCenterFactor = (*angleMax + *angleMin) / 2.0f;
		*velocityCenterFactor = (*velocityMax + *velocityMin) / 2.0f;
		*angleSizeFactor = (*angleMax - *angleMin) / 2.0f;
		*velocitySizeFactor = (*velocityMax - *velocityMin) / 2.0f;

		if (*angleSizeFactor < 0.1f)
			*angleSizeFactor = 1.0f;
		if (*velocitySizeFactor < 0.1f)
			*velocitySizeFactor = 1.0f;
	}

	*angleNorm = (angleSmooth - *angleCenterFactor) / *angleSizeFactor;
	*velocityNorm = (velocitySmooth - *velocityCenterFactor) / *velocitySizeFactor;

	*phaseAngleNorm = atan2f(*velocityNorm, *angleNorm);
	if (*phaseAngleNorm < 0.0f)
		*phaseAngleNorm += 2.0f * (float)M_PI;

	// No Movement Detection
	if (fabsf(velocitySmooth) < noMoveVelThreshold)
	{
		if (!*noMoveFlag)
		{
			*noMoveTime = timeSec;
		}
		else
		{
			if ((timeSec - *noMoveTime) > noMoveTimeThreshold)
			{
				*angleNorm = 0.0f;
				*velocityNorm = 0.0f;
			}
		}
		*noMoveFlag = true;
	}
	else
	{
		*noMoveFlag = false;
	}

	*phaseRadius = sqrtf((*angleNorm) * (*angleNorm) + (*velocityNorm) * (*velocityNorm));

	// Gait Cycle Estimation
	if (*walkingFlag && *istride > 1)
	{
		if (*istride > STEP_TRACK_NUM)
		{
			float sumStepDuration = 0.0f;
			for (uint8_t i = 0; i < STEP_TRACK_NUM; i++)
			{
				sumStepDuration += stepDurations[i];
			}
			float meanStepDuration = sumStepDuration / (float)STEP_TRACK_NUM;

			if ((timeSec - *stepPrevTime) > meanStepDuration)
			{
				*gaitCycleNorm = 100.0f;
			}
			else
			{
				*gaitCycleNorm = 100.0f * fmodf(timeSec - *stepPrevTime, meanStepDuration) / meanStepDuration;
			}
		}
		else
		{
			*gaitCycleNorm = *phaseAngleNorm * 100.0f / (2.0f * (float)M_PI);
		}

		if (*phaseRadius < radiusThreshold)
		{ // Walking Stop Check
			*walkingStopTime = timeSec;
			*walkingFlag = false;
			*startMoveFlag = false;
			*istride = 0;
			*gaitCycleNorm = 0.0f;
		}
	}
	else
	{
		*gaitCycleNorm = 0.0f;
	}

	if (*walkingFlag && *istride > STEP_TRACK_NUM)
	{ // # stride: to start providing torque
		float gaitCycle = *gaitCycleNorm;

		// Torque profile based on parameters
		gaitCycle = fmodf(gaitCycle - profile->x[0], 100.0f) + profile->x[0];

		for (int i = 0; i < NUM_SEGMENTS; i++)
		{
			if (gaitCycle >= profile->x[i] && gaitCycle <= profile->x[i + 1])
			{
				*torqueDesired = evaluate_hermite(gaitCycle - profile->x[i],
												  profile->c0[i], profile->c1[i], profile->c2[i],
												  profile->c3[i]);
				break;
			}
		}
	}
	else
	{
		*torqueDesired = 0;
	}
}

void main_controller(float timeSec,
					 float imuMeanLeft, float imuMeanRight,
					 IMUData *imuDataLeft, IMUData *imuDataRight,
					 float *gaitCycleNormLeft, float *gaitCycleNormRight,
					 bool *walkingFlagLeft, bool *walkingFlagRight,
					 TorqueProfile *profile,
					 float *torqueDesiredLeft, float *torqueDesiredRight)
{

	process_leg(timeSec, imuMeanLeft, imuDataLeft, gaitCycleNormLeft,
				walkingFlagLeft, profile, torqueDesiredLeft,
				&startMoveFlagLeft, &noMoveFlagLeft,
				&angleMinFlagLeft, &angleMaxFlagLeft,
				&velocityMinFlagLeft, &velocityMaxFlagLeft,
				&walkingStopTimeLeft, &noMoveTimeLeft,
				angleRawArrayLeft, velocityRawArrayLeft,
				angleFiltArrayLeft, velocityFiltArrayLeft,
				&phaseAngleFiltLeft, &phaseAngleFiltPrevLeft,
				angleSmoothArrayLeft, velocitySmoothArrayLeft,
				&phaseAngleSmoothLeft, &phaseAngleSmoothPrevLeft,
				&angleMinLeft, &angleMaxLeft,
				&angleCenterFactorLeft, &angleSizeFactorLeft,
				&velocityMinLeft, &velocityMaxLeft,
				&velocityCenterFactorLeft, &velocitySizeFactorLeft,
				&angleNormLeft, &velocityNormLeft,
				&phaseAngleNormLeft, &phaseRadiusLeft,
				&stepPrevTimeLeft, stepDurationsLeft,
				&phaseFiltDropFlagLeft, &phaseFiltDropTimeLeft,
				&istrideLeft);

	process_leg(timeSec, imuMeanRight, imuDataRight, gaitCycleNormRight,
				walkingFlagRight, profile, torqueDesiredRight,
				&startMoveFlagRight, &noMoveFlagRight,
				&angleMinFlagRight, &angleMaxFlagRight,
				&velocityMinFlagRight, &velocityMaxFlagRight,
				&walkingStopTimeRight, &noMoveTimeRight,
				angleRawArrayRight, velocityRawArrayRight,
				angleFiltArrayRight, velocityFiltArrayRight,
				&phaseAngleFiltRight, &phaseAngleFiltPrevRight,
				angleSmoothArrayRight, velocitySmoothArrayRight,
				&phaseAngleSmoothRight, &phaseAngleSmoothPrevRight,
				&angleMinRight, &angleMaxRight,
				&angleCenterFactorRight, &angleSizeFactorRight,
				&velocityMinRight, &velocityMaxRight,
				&velocityCenterFactorRight, &velocitySizeFactorRight,
				&angleNormRight, &velocityNormRight,
				&phaseAngleNormRight, &phaseRadiusRight,
				&stepPrevTimeRight, stepDurationsRight,
				&phaseFiltDropFlagRight, &phaseFiltDropTimeRight,
				&istrideRight);
}
