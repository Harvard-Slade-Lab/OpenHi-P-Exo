/* Host test of the profile transitions in torque_profile.c.
 *
 * Runs torque_profile_update_continuous at 100 Hz, as the control loop does, and checks
 *   - PARAMS: the 5 s linear transition to the new parameters,
 *   - STOP: both peak torques ramp to zero in 0.5 s while the timing parameters stay unchanged,
 *     from a settled profile and in the middle of a PARAMS transition, and the torque is then
 *     zero at every phase,
 *   - a PARAMS after a STOP resumes assistance with the 5 s transition.
 * A STOP ends exactly at zero torque (start + 1 * (0 - start)); a PARAMS transition ends within
 * float rounding of its target.
 */
// test-sources: Core/Src/torque_profile.c
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "torque_profile.h"

static long checks, failures;
#define CHECK(cond, ...)                                                    \
	do                                                                      \
	{                                                                       \
		checks++;                                                           \
		if (!(cond))                                                        \
		{                                                                   \
			if (++failures <= 20)                                           \
			{                                                               \
				printf("FAIL line %d: ", __LINE__);                         \
				printf(__VA_ARGS__);                                        \
				printf("\n");                                               \
			}                                                               \
		}                                                                   \
	} while (0)

static TorqueProfile profile;
static int timeMsec;

static float now(void) { return timeMsec / 1000.0f; }

static void get(float p[6])
{
	p[0] = profile.extensionPeakTorque;
	p[1] = profile.flexionPeakTorque;
	p[2] = profile.extensionPeakTime;
	p[3] = profile.flexionPeakTime;
	p[4] = profile.midTime;
	p[5] = profile.extensionRiseTime;
}

// Advances to timeMsec + ms in 10 ms steps (the 100 Hz block of the control loop)
static void run(int ms)
{
	for (int end = timeMsec + ms; timeMsec < end;)
	{
		timeMsec += 10;
		torque_profile_update_continuous(&profile, now());
	}
}

static int near(const float p[6], const float q[6])
{
	for (int i = 0; i < 6; i++)
	{
		if (fabsf(p[i] - q[i]) > 1e-5f * (1.0f + fabsf(q[i])))
			return 0;
	}
	return 1;
}

static float torque_at(float phase)
{
	for (int i = 0; i < NUM_SEGMENTS; i++)
	{
		if (phase >= profile.x[i] && phase <= profile.x[i + 1])
		{
			float x = phase - profile.x[i];
			return evaluate_hermite(x, profile.c0[i], profile.c1[i], profile.c2[i], profile.c3[i]);
		}
	}
	return NAN;
}

// Checks the ramp from `from` toward `to` over `duration` [s] started at `t0` [s]
static void check_ramp(const float from[6], const float to[6], float t0, float duration, int ms)
{
	for (int step = 0; step < ms / 10; step++)
	{
		run(10);
		float alpha = (now() - t0) / duration;
		if (alpha > 1.0f)
			alpha = 1.0f;
		float p[6];
		get(p);
		for (int i = 0; i < 6; i++)
		{
			float expect = from[i] + alpha * (to[i] - from[i]);
			CHECK(fabsf(p[i] - expect) <= 1e-5f * (1.0f + fabsf(from[i]) + fabsf(to[i])),
				  "t %.3f param %d: %.6f, expected %.6f", now(), i, p[i], expect);
		}
	}
}

int main(void)
{
	const float paramsA[6] = {-0.4f, 0.2f, 22.0f, 70.0f, 45.0f, 25.0f};
	const float paramsB[6] = {-0.6f, 0.3f, 12.0f, 62.0f, 37.0f, 33.0f};
	float start[6], target[6], p[6];

	torque_profile_init(&profile);
	timeMsec = 5000;

	// PARAMS: 5 s transition
	get(start);
	torque_profile_set_target((float *)paramsA);
	torque_profile_update_trigger(&profile, now());
	check_ramp(start, paramsA, now(), 5.0f, 5000);
	get(p);
	CHECK(near(p, paramsA), "PARAMS: target not reached after 5 s");
	run(1000);

	// STOP from a settled profile: 0.5 s ramp of the peak torques only
	float t0 = now();
	get(start);
	memcpy(target, start, sizeof(target));
	target[0] = target[1] = 0.0f;
	torque_profile_stop_trigger(&profile, t0);
	check_ramp(start, target, t0, 0.5f, 500);
	get(p);
	CHECK(memcmp(p, target, sizeof(p)) == 0, "STOP: torques not zero or timing changed after 0.5 s");
	run(1000);
	get(p);
	CHECK(memcmp(p, target, sizeof(p)) == 0, "STOP: profile changed after the ramp");
	for (int k = 0; k <= 10000; k++)
	{
		float phase = profile.x[0] + (profile.x[NUM_SEGMENTS] - profile.x[0]) * k / 10000.0f;
		float tq = torque_at(phase);
		CHECK(tq == 0.0f, "STOP: torque %.6f at phase %.2f", tq, phase);
	}

	// PARAMS after STOP: assistance resumes with the 5 s transition
	t0 = now();
	get(start);
	torque_profile_set_target((float *)paramsB);
	torque_profile_update_trigger(&profile, t0);
	check_ramp(start, paramsB, t0, 5.0f, 2000);

	// STOP in the middle of that transition: ramps from where the profile is
	t0 = now();
	get(start);
	memcpy(target, start, sizeof(target));
	target[0] = target[1] = 0.0f;
	CHECK(start[0] != 0.0f && start[1] != 0.0f, "mid-transition torques should be non-zero");
	torque_profile_stop_trigger(&profile, t0);
	check_ramp(start, target, t0, 0.5f, 500);
	get(p);
	CHECK(memcmp(p, target, sizeof(p)) == 0, "STOP mid-transition: not settled after 0.5 s");
	run(5000);
	get(p);
	CHECK(memcmp(p, target, sizeof(p)) == 0, "STOP mid-transition: the PARAMS transition resumed");

	// A second STOP keeps the profile at zero torque
	torque_profile_stop_trigger(&profile, now());
	run(1000);
	get(p);
	CHECK(memcmp(p, target, sizeof(p)) == 0, "repeated STOP changed the profile");

	// PARAMS again: 5 s, not 0.5 s
	t0 = now();
	get(start);
	torque_profile_set_target((float *)paramsA);
	torque_profile_update_trigger(&profile, t0);
	check_ramp(start, paramsA, t0, 5.0f, 5000);
	get(p);
	CHECK(near(p, paramsA), "PARAMS after STOP: target not reached after 5 s");

	printf("torque_profile_test: %ld checks, %ld failures\n", checks, failures);
	return failures ? 1 : 0;
}
