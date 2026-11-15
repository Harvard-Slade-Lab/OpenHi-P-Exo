/* Host test of the phase-to-torque mapping in control_logic.c.
 *
 * The mapping block is taken from Core/Src/control_logic.c by run_tests.sh and runs here with the
 * firmware's torque_profile.c. It is compared with a double-precision periodic reference through
 * the same knots (zero end slopes; first knot x0 = extensionPeakTime - extensionRiseTime).
 * Profiles: the optimizer's parameter box (Optimizer/param_utils.py, constraints applied, 70 kg).
 * Cases: box corners, phases at the x0 boundary, random profiles and phases, a 100 Hz sweep.
 */
// test-sources: Core/Src/torque_profile.c
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "torque_profile.h"

#define BODY_MASS 70.0f
#define HELD 1000.0f // torqueDesired before the block runs: a sample left at this value fails the reference check

static const float lo[6] = {-0.6f, 0.1f, 10.0f, 60.0f, 35.0f, 15.0f};
static const float hi[6] = {-0.2f, 0.3f, 30.0f, 80.0f, 55.0f, 35.0f};

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

/* code under test: the block of control_logic.c process_leg */
static float map_firmware(float gaitCycleNormValue, TorqueProfile *profile)
{
	float torque = HELD;
	float *gaitCycleNorm = &gaitCycleNormValue;
	float *torqueDesired = &torque;
#include "control_logic_mapping.inc"
	return torque;
}

/* periodic zero-slope Hermite through the profile's knots, in double */
static double ref_torque(const TorqueProfile *pr, double g)
{
	double x0 = pr->x[0];
	double u = g - x0;
	u = u - 100.0 * floor(u / 100.0) + x0;
	for (int i = 0; i < NUM_SEGMENTS; i++)
	{
		double a = pr->x[i], b = pr->x[i + 1];
		if (u >= a && (u <= b || i == NUM_SEGMENTS - 1))
		{
			double s = (u - a) / (b - a);
			if (s > 1.0)
				s = 1.0;
			return pr->y[i] + (pr->y[i + 1] - pr->y[i]) * (3.0 * s * s - 2.0 * s * s * s);
		}
	}
	return NAN;
}

/* p: parameters in the firmware order, torques in Nm/kg; constraints of param_utils.py */
static void make_profile(TorqueProfile *pr, const float *pin)
{
	float p[6];
	memcpy(p, pin, sizeof(p));
	if (p[5] - p[2] + p[3] > 90.0f)
		p[5] = 90.0f + p[2] - p[3];
	if (p[4] - p[2] < 15.0f)
		p[4] = 15.0f + p[2];
	else if (p[3] - p[4] < 15.0f)
		p[4] = p[3] - 15.0f;
	memset(pr, 0, sizeof(*pr));
	pr->extensionPeakTorque = p[0] * BODY_MASS;
	pr->flexionPeakTorque = p[1] * BODY_MASS;
	pr->extensionPeakTime = p[2];
	pr->flexionPeakTime = p[3];
	pr->midTime = p[4];
	pr->extensionRiseTime = p[5];
	torque_profile_update(pr);
}

static uint32_t rng = 12345u;
static float urand(void) /* [0, 1) */
{
	rng ^= rng << 13;
	rng ^= rng >> 17;
	rng ^= rng << 5;
	return (float)(rng >> 8) / 16777216.0f;
}

/* one phase: the block matches the reference */
static void check_phase(TorqueProfile *pr, float g, const char *what)
{
	float t = map_firmware(g, pr);
	double ref = ref_torque(pr, g);
	double tol = 2e-5 * (fabs(pr->y[1]) + fabs(pr->y[3]));
	CHECK(fabs((double)t - ref) <= tol, "%s: x0=%.9g phase=%.9g torque=%.9g reference=%.9g",
		  what, (double)pr->x[0], (double)g, (double)t, ref);
}

static void check_boundary(TorqueProfile *pr, const char *what)
{
	float x0 = pr->x[0];
	float b[10] = {x0 - 0.01f, nextafterf(x0, -INFINITY), x0, nextafterf(x0, INFINITY), x0 + 0.01f,
				   x0 + 100.0f - 0.01f, nextafterf(x0 + 100.0f, -INFINITY), x0 + 100.0f,
				   x0 - 100.0f, nextafterf(x0 - 100.0f, INFINITY)};
	for (int k = 0; k < 10; k++)
		if (b[k] >= 0.0f && b[k] <= 100.0f)
			check_phase(pr, b[k], what);
}

static void check_sweep_100hz(TorqueProfile *pr, const char *what)
{
	const float periods[4] = {1.0f, 1.1f, 1.2f, 1.3f};
	for (int q = 0; q < 4; q++)
	{
		int n = (int)(3.0f * periods[q] * 100.0f);
		for (int k = 0; k < n; k++)
			check_phase(pr, 100.0f * fmodf((float)k * 0.01f, periods[q]) / periods[q], what);
	}
	check_phase(pr, 100.0f, what); // saturated phase (control_logic.c, stride longer than the mean)
}

static void check_grid(TorqueProfile *pr, const char *what)
{
	for (int k = 0; k <= 10000; k++)
		check_phase(pr, (float)k * 0.01f, what);
}

int main(void)
{
	TorqueProfile pr;
	float p[6];
	int nprof = 0, nx0pos = 0;

	/* 1. corners of the parameter box: phase corners, x0 boundary, 0.01 % grid, 100 Hz sweep */
	for (int m = 0; m < 64; m++)
	{
		for (int i = 0; i < 6; i++)
			p[i] = (m >> i & 1) ? hi[i] : lo[i];
		make_profile(&pr, p);
		nprof++;
		nx0pos += pr.x[0] > 0.0f;
		const float corners[6] = {0.0f, nextafterf(0.0f, 1.0f), 50.0f, 99.99f, nextafterf(100.0f, 0.0f), 100.0f};
		for (int k = 0; k < 6; k++)
			check_phase(&pr, corners[k], "corner");
		check_boundary(&pr, "corner boundary");
		check_grid(&pr, "corner grid");
		check_sweep_100hz(&pr, "corner 100 Hz");
	}

	/* 2. x0 near zero and at the box limits (extensionPeakTime 20, midTime 45, flexionPeakTime 70) */
	const float x0s[9] = {15.0f, 5.0f, 0.5f, 0.01f, 0.0f, -0.01f, -0.5f, -5.0f, -15.0f};
	for (int j = 0; j < 9; j++)
	{
		float q[6] = {-0.4f, 0.2f, 20.0f, 70.0f, 45.0f, 20.0f - x0s[j]};
		make_profile(&pr, q);
		nprof++;
		nx0pos += pr.x[0] > 0.0f;
		check_boundary(&pr, "x0 boundary");
		check_grid(&pr, "x0 grid");
		check_sweep_100hz(&pr, "x0 100 Hz");
	}

	/* 3. random profiles: random phases and the 100 Hz sweep */
	for (int n = 0; n < 2000; n++)
	{
		for (int i = 0; i < 6; i++)
			p[i] = lo[i] + (hi[i] - lo[i]) * urand();
		make_profile(&pr, p);
		nprof++;
		nx0pos += pr.x[0] > 0.0f;
		for (int k = 0; k < 200; k++)
			check_phase(&pr, 100.0f * urand(), "random");
		check_boundary(&pr, "random boundary");
		check_sweep_100hz(&pr, "random 100 Hz");
	}

	printf("profiles: %d (%d with x0 > 0)\n", nprof, nx0pos);
	printf("fmodf_test: %ld checks, %ld failures\n", checks, failures);
	return failures ? 1 : 0;
}
