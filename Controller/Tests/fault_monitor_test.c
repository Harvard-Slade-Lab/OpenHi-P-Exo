/* Host fault-injection test of fault_monitor.c.
 *
 * Runs the firmware's control_logic.c, torque_profile.c and imu_processing.c with the fault monitor
 * in a 1 kHz loop laid out like the TIM4 interrupt of main.c: CAN1 sensor frames (100 Hz, decoded by
 * process_imu_data as can_rx_callback does), CAN2 motor feedback (200 Hz), the fault check and motor
 * gating at 500 Hz from 5 s on, and main_controller at 100 Hz. The synthetic thigh angles are
 * sinusoids (25 deg amplitude, 0.9 Hz, legs half a cycle apart) and gyroX is their rate in rad/s.
 * The motors count as driven when the gating of main.c would send a nonzero torque command.
 *
 * Every scenario starts from the same 20 s of walking (fork per scenario, because control_logic.c
 * keeps its filter and gait state in static variables) and checks:
 *   - no fault and assistance on during normal walking (no false alarm),
 *   - IMU freeze (frames stop) at 20 stride phases: the monitor stops the motors FAULT_STALE_MS
 *     (+ one 2 ms check) after the last frame, and they stay off 1-15 s after the freeze and after
 *     the frames return (latched),
 *   - load-cell freeze: frames stop (stale), frames repeat one value while assisting (stuck, after
 *     FAULT_LOAD_STUCK_MS), one constant value while standing (no fault), out-of-range values,
 *   - motor error codes 1-6 on either motor, motor feedback that stops,
 *   - CAN error injection: error warning/passive is logged only, bus-off on either bus stops,
 *   - a stream that never starts, the 32-bit millisecond counter wrapping, and latched codes.
 */
// test-sources: Core/Src/fault_monitor.c Core/Src/control_logic.c Core/Src/torque_profile.c Core/Src/imu_processing.c
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include "stm32f4xx_hal.h"
#include "globals.h"
#include "control_logic.h"
#include "torque_profile.h"
#include "imu_processing.h"
#include "fault_monitor.h"

GPIO_TypeDef stubGPIOA, stubGPIOC;
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin)
{
	(void)p;
	(void)pin;
	return GPIO_PIN_SET;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState s)
{
	(void)p;
	(void)pin;
	(void)s;
}

static long checks, failures;
#define CHECK(cond, ...)                                    \
	do                                                      \
	{                                                       \
		checks++;                                           \
		if (!(cond))                                        \
		{                                                   \
			if (++failures <= 20)                           \
			{                                               \
				printf("FAIL line %d: ", __LINE__);         \
				printf(__VA_ARGS__);                        \
				printf("\n");                               \
			}                                               \
		}                                                   \
	} while (0)

enum
{
	L = FAULT_SIDE_LEFT,
	R = FAULT_SIDE_RIGHT
};

#define CONTROL_START_MS 5000 // initializationTime + calibrationTime (main.c)
#define WALK_HZ 0.9
#define AMP_DEG 25.0
#define OFFSET_DEG 5.0

/* ---- simulated device ------------------------------------------------------------------------ */
typedef struct
{
	bool imuOn[2], loadOn[2], motorOn[2]; // frames sent
	bool walking;                         // thigh motion (else standing at the offset angle)
	bool loadHold[2];                     // load frames repeat loadValue
	float loadValue[2];
	int8_t motorError[2];
} Inject;

static FaultMonitor mon;
static TorqueProfile profile;
static IMUData imu[2];
static float gaitCycle[2], torque[2];
static bool walk[2];
static uint32_t nowMs;
static Inject inj;
static bool stopped;          // monitor: zero current
static bool drivenMonitor;    // nonzero command with the monitor
static uint32_t rng = 12345;

static int noise(void) // -2..2 ADC counts
{
	rng = rng * 1103515245u + 12345u;
	return (int)((rng >> 16) % 5) - 2;
}

static void put_be(uint8_t *b, float f)
{
	uint32_t u;
	memcpy(&u, &f, 4);
	b[0] = (uint8_t)(u >> 24);
	b[1] = (uint8_t)(u >> 16);
	b[2] = (uint8_t)(u >> 8);
	b[3] = (uint8_t)u;
}

static void put_le_i32(uint8_t *b, int32_t v)
{
	uint32_t u = (uint32_t)v;
	b[0] = (uint8_t)u;
	b[1] = (uint8_t)(u >> 8);
	b[2] = (uint8_t)(u >> 16);
	b[3] = (uint8_t)(u >> 24);
}

// One CAN1 frame as can_rx_callback handles it
static void sensor_frame(int side, uint32_t low, uint8_t *d)
{
	uint32_t id = (side == L ? IMU_LEFT_ID : IMU_RIGHT_ID) + low;
	process_imu_data(&imu[side], id, d);
	fault_monitor_sensor_frame(&mon, id, nowMs);
}

static void send_sensor_frames(int side)
{
	uint8_t d[8] = {0};
	double t = nowMs / 1000.0;
	double th = 2 * M_PI * WALK_HZ * t + (side == R ? M_PI : 0.0);
	double roll = OFFSET_DEG, gyro = 0.0;
	if (inj.walking)
	{
		roll = OFFSET_DEG + AMP_DEG * sin(th);
		gyro = AMP_DEG * M_PI / 180.0 * 2 * M_PI * WALK_HZ * cos(th);
	}

	if (inj.loadOn[side])
	{
		float load = inj.loadHold[side] ? inj.loadValue[side]
										: (float)(int)lrint(2048.0 + 30.0 * torque[side] + noise());
		put_le_i32(d, (int32_t)load); // integer ADC counts
		sensor_frame(side, 0x010, d);
	}
	if (inj.imuOn[side])
	{
		put_be(&d[0], (float)roll);
		put_be(&d[4], 0.0f);
		sensor_frame(side, 0x011, d); // roll, pitch
		put_be(&d[0], 0.0f);
		sensor_frame(side, 0x012, d);
		sensor_frame(side, 0x013, d);
		sensor_frame(side, 0x014, d);
		put_be(&d[0], (float)gyro); // rate of the thigh angle, rad/s
		sensor_frame(side, 0x015, d); // gyroX, gyroY
		put_be(&d[0], 0.0f);
		sensor_frame(side, 0x016, d);
	}
}

// One 1 kHz step (TIM4) with the CAN frames received in that millisecond
static void step(void)
{
	nowMs++;
	if (nowMs % 10 == 5)
	{
		send_sensor_frames(L);
		send_sensor_frames(R);
	}
	if (nowMs % 5 == 3)
	{
		for (int side = 0; side < 2; side++)
			if (inj.motorOn[side])
				fault_monitor_motor_frame(&mon, (FaultSide)side, inj.motorError[side], nowMs);
	}
	if (nowMs <= CONTROL_START_MS)
		return;

	if (nowMs % 2 == 0)
	{ // 500 Hz: gating of main.c (controlMode = 1)
		bool assisting = walk[L] && walk[R];
		stopped = fault_monitor_check(&mon, nowMs, imu[L].load, imu[R].load, assisting);
		bool nonzero = fabsf(torque[L]) > 1e-3f || fabsf(torque[R]) > 1e-3f;
		drivenMonitor = !stopped && assisting && nonzero;
	}
	if (nowMs % 10 == 0)
	{ // 100 Hz
		float t = nowMs / 1000.0f;
		torque_profile_update_continuous(&profile, t);
		main_controller(t, OFFSET_DEG, OFFSET_DEG, &imu[L], &imu[R], &gaitCycle[L], &gaitCycle[R],
						&walk[L], &walk[R], &profile, &torque[L], &torque[R]);
	}
}

static void run_ms(uint32_t ms)
{
	for (uint32_t i = 0; i < ms; i++)
		step();
}

static void reset_device(void)
{
	memset(&inj, 0, sizeof(inj));
	for (int side = 0; side < 2; side++)
		inj.imuOn[side] = inj.loadOn[side] = inj.motorOn[side] = true;
	inj.walking = true;
	nowMs = 0;
	fault_monitor_init(&mon, nowMs);
	torque_profile_init(&profile);
	float params[6] = {-30.0f, 15.0f, 20.0f, 70.0f, 45.0f, 25.0f};
	torque_profile_set_target(params);
	torque_profile_update_trigger(&profile, 0.0f);
}

/* ---- scenarios (each runs in a child process) ------------------------------------------------- */
typedef void (*Scenario)(int arg);
static int failedScenarios, scenarios;
static long totalChecks;

// Runs fn(arg) in a child from the current state
static void run_scenario(const char *name, Scenario fn, int arg)
{
	int fd[2];
	if (pipe(fd) != 0)
	{
		perror("pipe");
		exit(2);
	}
	fflush(stdout);
	pid_t pid = fork();
	if (pid < 0)
	{
		perror("fork");
		exit(2);
	}
	if (pid == 0)
	{
		close(fd[0]);
		checks = failures = 0;
		fn(arg);
		if (failures)
			printf("  scenario %s(%d): %ld failures\n", name, arg, failures);
		fflush(stdout);
		_exit(write(fd[1], &checks, sizeof(checks)) == (ssize_t)sizeof(checks) && failures == 0 ? 0 : 1);
	}
	close(fd[1]);
	long childChecks = 0;
	if (read(fd[0], &childChecks, sizeof(childChecks)) != (ssize_t)sizeof(childChecks))
		childChecks = 0;
	close(fd[0]);
	int status = 0;
	waitpid(pid, &status, 0);
	scenarios++;
	totalChecks += childChecks;
	if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
	{
		failedScenarios++;
		printf("  scenario %s(%d) FAILED\n", name, arg);
	}
}

static void run_to(uint32_t mod10) // steps until nowMs % 10 == mod10
{
	do
		step();
	while (nowMs % 10 != mod10);
}

// Runs until the monitor stops the motors (at most limit ms); returns the time of the stop or 0
static uint32_t run_until_stop(uint32_t limit)
{
	for (uint32_t i = 0; i < limit && !stopped; i++)
		step();
	return stopped ? nowMs : 0;
}

static void expect_stop(uint32_t first, uint32_t last, uint32_t fault, const char *what)
{
	uint32_t ref = first;
	uint32_t at = run_until_stop(last + 1000 - nowMs);
	CHECK(at != 0, "%s: no stop", what);
	CHECK(at >= first && at <= last, "%s: stop at %u, expected %u..%u", what, at, ref, last);
	CHECK(mon.faults == fault, "%s: faults 0x%x, expected 0x%x", what, mon.faults, fault);
}

// Runs ms and returns how many 500 Hz ticks drove the motors
static int driven_ticks(uint32_t ms)
{
	int n = 0;
	for (uint32_t i = 0; i < ms; i++)
	{
		step();
		if (nowMs > CONTROL_START_MS && nowMs % 2 == 0 && drivenMonitor)
			n++;
	}
	return n;
}

// Freshness: a stream whose last frame came at `last` is stale at the first 500 Hz check after
// last + FAULT_STALE_MS
static void expect_stale(uint32_t last, uint32_t fault, const char *what)
{
	expect_stop(last + FAULT_STALE_MS + 1, last + FAULT_STALE_MS + 2, fault, what);
}

static void sc_imu_freeze(int phaseIdx)
{
	run_ms((uint32_t)lrint(phaseIdx / 20.0 / WALK_HZ * 1000.0));
	run_to(5); // right after a frame
	uint32_t last = nowMs;
	inj.imuOn[L] = inj.imuOn[R] = false;
	expect_stale(last, FAULT_IMU_LEFT_STALE | FAULT_IMU_RIGHT_STALE, "IMU freeze");

	// 1-15 s after the freeze: the motors stay off
	run_ms(last + 1000 - nowMs);
	int n = driven_ticks(14000);
	CHECK(n == 0, "motors driven %d ticks after the IMU freeze", n);

	inj.imuOn[L] = inj.imuOn[R] = true; // frames return: still stopped (latched)
	CHECK(driven_ticks(5000) == 0 && stopped, "stop not latched after the IMU frames return");
}

static void sc_imu_one_side(int side)
{
	run_to(5);
	uint32_t last = nowMs;
	inj.imuOn[side] = false;
	expect_stale(last, side == L ? FAULT_IMU_LEFT_STALE : FAULT_IMU_RIGHT_STALE, "IMU one side");
}

static void sc_load_stale(int side)
{
	run_to(5);
	uint32_t last = nowMs;
	inj.loadOn[side] = false;
	expect_stale(last, side == L ? FAULT_LOAD_LEFT_STALE : FAULT_LOAD_RIGHT_STALE, "load stale");
}

static void sc_load_stuck(int side)
{
	run_to(7); // the frame at ...5 has been checked at ...6
	CHECK(walk[L] && walk[R], "not assisting before the load freeze");
	// the sender repeats the last value
	inj.loadHold[side] = true;
	inj.loadValue[side] = imu[side].load;
	uint32_t change = mon.loadChangeMs[side];
	CHECK(change >= nowMs - 11, "last change %u at %u", change, nowMs);
	expect_stop(change + FAULT_LOAD_STUCK_MS, change + FAULT_LOAD_STUCK_MS,
				side == L ? FAULT_LOAD_LEFT_STUCK : FAULT_LOAD_RIGHT_STUCK, "load stuck");
}

static void sc_load_standing(int unused)
{
	(void)unused;
	inj.walking = false; // stand: frames continue, thighs still
	for (int i = 0; i < 20000 && (walk[L] || walk[R]); i++)
		step();
	CHECK(!walk[L] && !walk[R], "walking flags still set after standing for 20 s");
	inj.loadHold[L] = inj.loadHold[R] = true;
	inj.loadValue[L] = 2048.0f;
	inj.loadValue[R] = 2051.0f;
	run_ms(10000);
	CHECK(mon.faults == 0, "fault 0x%x while standing with a constant load-cell value", mon.faults);
}

static const float rangeValues[] = {0.0f, 4095.0f, -3.0f, 5000.0f, 1.0f, 4094.0f};

static void sc_load_range(int idx)
{
	int side = idx % 2;
	float v = rangeValues[idx / 2];
	bool bad = !(v >= 1.0f && v <= 4094.0f);
	run_to(7);
	uint32_t frame = nowMs + 8; // next frame
	inj.loadHold[side] = true;
	inj.loadValue[side] = v;
	if (bad)
		expect_stop(frame + 1, frame + 1, side == L ? FAULT_LOAD_LEFT_RANGE : FAULT_LOAD_RIGHT_RANGE, "range");
	else
	{
		run_ms(400); // shorter than FAULT_LOAD_STUCK_MS
		CHECK(mon.faults == 0, "load %.0f: faults 0x%x", v, mon.faults);
	}
}

static void sc_motor_error(int idx)
{
	int side = idx % 2;
	int8_t code = (int8_t)(idx / 2 + 1); // 1..6
	while (nowMs % 5 != 4)
		step();
	uint32_t frame = nowMs + 4; // next motor frame (nowMs % 5 == 3), checked at the next even ms
	inj.motorError[side] = code;
	expect_stop(frame, frame + 1, side == L ? FAULT_MOTOR_LEFT_ERROR : FAULT_MOTOR_RIGHT_ERROR, "motor error");
	CHECK(mon.motorError[side] == code && mon.motorError[1 - side] == 0, "motor error code not recorded");
	inj.motorError[side] = 0; // the error clears: still stopped
	CHECK(driven_ticks(3000) == 0, "motors driven after motor error %d", code);
}

static void sc_motor_stale(int side)
{
	while (nowMs % 5 != 3)
		step();
	uint32_t last = nowMs;
	inj.motorOn[side] = false;
	expect_stale(last, side == L ? FAULT_MOTOR_LEFT_STALE : FAULT_MOTOR_RIGHT_STALE, "motor stale");
}

static void sc_can_error(int idx)
{
	int bus = 1 + idx % 2;
	bool busOff = idx >= 2;
	run_to(1);
	fault_monitor_can_error(&mon, bus, busOff); // HAL_CAN_ErrorCallback
	uint32_t err = bus == 1 ? FAULT_CAN1_ERROR : FAULT_CAN2_ERROR;
	uint32_t off = bus == 1 ? FAULT_CAN1_BUS_OFF : FAULT_CAN2_BUS_OFF;
	if (busOff)
	{
		expect_stop(nowMs + 1, nowMs + 1, err | off, "bus-off");
		CHECK(driven_ticks(3000) == 0, "motors driven after bus-off on CAN%d", bus);
	}
	else
	{
		int n = driven_ticks(5000);
		CHECK(n > 2000, "CAN%d error warning stopped assistance (%d of 2500 ticks driven)", bus, n);
		CHECK(mon.faults == err, "CAN%d error warning: faults 0x%x", bus, mon.faults);
	}
}

static void sc_set(int idx)
{
	uint32_t code = idx == 0 ? FAULT_WATCHDOG_RESET : idx == 1 ? FAULT_LIMIT : FAULT_MOTOR_TX;
	run_to(1);
	fault_monitor_set(&mon, code);
	expect_stop(nowMs + 1, nowMs + 1, code, "fault_monitor_set");
	CHECK(driven_ticks(3000) == 0, "code 0x%x: motors driven", code);
}

// One stream never sends from power-up: fault at the first check after calibration
static void sc_never_started(int s)
{
	reset_device();
	int side = s % 2;
	uint32_t expect;
	if (s < 2)
	{
		inj.imuOn[side] = false;
		expect = side == L ? FAULT_IMU_LEFT_STALE : FAULT_IMU_RIGHT_STALE;
	}
	else if (s < 4)
	{
		inj.loadOn[side] = false; // the value stays 0: also out of range
		expect = side == L ? FAULT_LOAD_LEFT_STALE | FAULT_LOAD_LEFT_RANGE
						   : FAULT_LOAD_RIGHT_STALE | FAULT_LOAD_RIGHT_RANGE;
	}
	else
	{
		inj.motorOn[side] = false;
		expect = side == L ? FAULT_MOTOR_LEFT_STALE : FAULT_MOTOR_RIGHT_STALE;
	}
	run_ms(CONTROL_START_MS);
	CHECK(mon.faults == 0, "fault 0x%x before control starts", mon.faults);
	expect_stop(CONTROL_START_MS + 2, CONTROL_START_MS + 2, expect, "never started");
}

/* ---- monitor alone ---------------------------------------------------------------------------- */
static void test_monitor(void)
{
	// the 32-bit millisecond counter wraps while all streams are fresh
	FaultMonitor m;
	uint32_t t = 0xFFFFFF00u;
	fault_monitor_init(&m, t);
	float load = 2000.0f;
	for (int i = 0; i < 1000; i++, t += 2)
	{
		if (i % 5 == 0)
		{
			for (uint32_t base = IMU_LEFT_ID; base <= IMU_RIGHT_ID; base += IMU_RIGHT_ID - IMU_LEFT_ID)
			{
				fault_monitor_sensor_frame(&m, base + 0x010, t);
				fault_monitor_sensor_frame(&m, base + 0x011, t);
				fault_monitor_sensor_frame(&m, base + 0x015, t);
			}
			fault_monitor_motor_frame(&m, FAULT_SIDE_LEFT, 0, t);
			fault_monitor_motor_frame(&m, FAULT_SIDE_RIGHT, 0, t);
			load += 1.0f;
		}
		CHECK(!fault_monitor_check(&m, t, load, load, true), "fault 0x%x at t = 0x%08x", m.faults, t);
	}
	uint32_t last = t - 10;
	bool stop = false;
	while (!stop && t < last + 200)
	{
		stop = fault_monitor_check(&m, t, load, load + 1, false);
		t += 2;
	}
	t -= 2;
	CHECK(stop && t > last + FAULT_STALE_MS && t <= last + FAULT_STALE_MS + 2,
		  "stale after the wrap: stop %d at +%u ms", stop, t - last);

	// frames with IDs the monitor does not watch do not keep a stream fresh
	fault_monitor_init(&m, 0);
	for (uint32_t i = 0; i <= 200; i += 10)
	{
		fault_monitor_sensor_frame(&m, IMU_LEFT_ID + 0x012, i);
		fault_monitor_sensor_frame(&m, IMU_RIGHT_ID + 0x016, i);
		fault_monitor_sensor_frame(&m, 0x410, i);
		fault_monitor_sensor_frame(&m, 0x611, i);
	}
	const uint32_t sensorsStale = FAULT_IMU_LEFT_STALE | FAULT_IMU_RIGHT_STALE | FAULT_LOAD_LEFT_STALE |
								  FAULT_LOAD_RIGHT_STALE;
	CHECK(fault_monitor_check(&m, 200, 2000.0f, 2000.0f, false), "unwatched IDs kept the streams fresh");
	CHECK((m.faults & sensorsStale) == sensorsStale, "stale faults 0x%x", m.faults);

	// a non-finite load value is out of range
	fault_monitor_init(&m, 0);
	fault_monitor_check(&m, 0, NAN, 2000.0f, false);
	CHECK(m.faults == FAULT_LOAD_LEFT_RANGE, "NaN load: faults 0x%x", m.faults);

	// error warning/passive alone does not stop; every other code does
	for (int bit = 0; bit < 19; bit++)
	{
		fault_monitor_init(&m, 0);
		fault_monitor_set(&m, 1UL << bit);
		bool logOnly = (1UL << bit) == FAULT_CAN1_ERROR || (1UL << bit) == FAULT_CAN2_ERROR;
		CHECK(fault_monitor_check(&m, 0, 2000.0f, 2000.0f, false) == !logOnly, "fault bit %d", bit);
	}
}

int main(void)
{
	test_monitor();
	for (int s = 0; s < 6; s++)
		run_scenario("never_started", sc_never_started, s);

	// common start: 20 s of walking; no false alarm, assistance on in the last 10 s
	reset_device();
	run_ms(10000);
	int n = driven_ticks(10000);
	CHECK(mon.faults == 0, "fault 0x%x during normal walking", mon.faults);
	CHECK(n > 4000, "assistance on in only %d of 5000 ticks of normal walking", n);
	if (mon.faults != 0 || n <= 4000)
	{
		printf("fault_monitor_test: normal walking failed, scenarios skipped (%ld failures)\n", failures);
		return 1;
	}

	for (int p = 0; p < 20; p++)
		run_scenario("imu_freeze", sc_imu_freeze, p);

	for (int side = 0; side < 2; side++)
	{
		run_scenario("imu_one_side", sc_imu_one_side, side);
		run_scenario("load_stale", sc_load_stale, side);
		run_scenario("load_stuck", sc_load_stuck, side);
		run_scenario("motor_stale", sc_motor_stale, side);
	}
	run_scenario("load_standing", sc_load_standing, 0);
	for (int i = 0; i < 12; i++)
	{
		run_scenario("load_range", sc_load_range, i);
		run_scenario("motor_error", sc_motor_error, i);
	}
	for (int i = 0; i < 4; i++)
		run_scenario("can_error", sc_can_error, i);
	for (int i = 0; i < 3; i++)
		run_scenario("set", sc_set, i);

	checks += totalChecks;
	failures += failedScenarios;
	printf("fault_monitor_test: %d scenarios (%d failed), %ld checks, %ld failures\n", scenarios, failedScenarios,
		   checks, failures);
	return failures ? 1 : 0;
}
