#include <string.h>

#include "globals.h"
#include "fault_monitor.h"

#define SENSOR_LOAD 0x010 // low byte of the CAN1 ID (imu_processing.c)
#define SENSOR_ROLL 0x011
#define SENSOR_GYRO 0x015

static const uint32_t staleFault[2][FAULT_STREAM_COUNT] = {
    {FAULT_IMU_LEFT_STALE, FAULT_IMU_LEFT_STALE, FAULT_LOAD_LEFT_STALE, FAULT_MOTOR_LEFT_STALE},
    {FAULT_IMU_RIGHT_STALE, FAULT_IMU_RIGHT_STALE, FAULT_LOAD_RIGHT_STALE, FAULT_MOTOR_RIGHT_STALE},
};
static const uint32_t rangeFault[2] = {FAULT_LOAD_LEFT_RANGE, FAULT_LOAD_RIGHT_RANGE};
static const uint32_t stuckFault[2] = {FAULT_LOAD_LEFT_STUCK, FAULT_LOAD_RIGHT_STUCK};
static const uint32_t motorFault[2] = {FAULT_MOTOR_LEFT_ERROR, FAULT_MOTOR_RIGHT_ERROR};

void fault_monitor_init(FaultMonitor *m, uint32_t nowMs)
{
    memset(m, 0, sizeof(*m));
    for (int side = 0; side < 2; side++)
    {
        for (int s = 0; s < FAULT_STREAM_COUNT; s++)
        {
            m->lastRxMs[side][s] = nowMs;
        }
        m->loadChangeMs[side] = nowMs;
    }
}

void fault_monitor_set(FaultMonitor *m, uint32_t faults)
{
    m->faults |= faults;
}

void fault_monitor_sensor_frame(FaultMonitor *m, uint32_t stdId, uint32_t nowMs)
{
    int side;
    if (stdId >= IMU_LEFT_ID && stdId < IMU_LEFT_ID + 0x100)
        side = FAULT_SIDE_LEFT;
    else if (stdId >= IMU_RIGHT_ID && stdId < IMU_RIGHT_ID + 0x100)
        side = FAULT_SIDE_RIGHT;
    else
        return;

    switch (stdId & 0x0FF)
    {
    case SENSOR_LOAD:
        m->lastRxMs[side][FAULT_STREAM_LOAD] = nowMs;
        break;
    case SENSOR_ROLL:
        m->lastRxMs[side][FAULT_STREAM_ROLL] = nowMs;
        break;
    case SENSOR_GYRO:
        m->lastRxMs[side][FAULT_STREAM_GYRO] = nowMs;
        break;
    }
}

void fault_monitor_motor_frame(FaultMonitor *m, FaultSide side, int8_t errorCode, uint32_t nowMs)
{
    m->lastRxMs[side][FAULT_STREAM_MOTOR] = nowMs;
    m->motorError[side] = errorCode;
    if (errorCode != 0)
    {
        m->faults |= motorFault[side];
    }
}

void fault_monitor_can_error(FaultMonitor *m, int bus, bool busOff)
{
    if (bus == 1)
        m->faults |= FAULT_CAN1_ERROR | (busOff ? FAULT_CAN1_BUS_OFF : 0);
    else
        m->faults |= FAULT_CAN2_ERROR | (busOff ? FAULT_CAN2_BUS_OFF : 0);
}

bool fault_monitor_check(FaultMonitor *m, uint32_t nowMs, float loadLeft, float loadRight, bool assisting)
{
    const float load[2] = {loadLeft, loadRight};

    for (int side = 0; side < 2; side++)
    {
        for (int s = 0; s < FAULT_STREAM_COUNT; s++)
        {
            if ((int32_t)(nowMs - m->lastRxMs[side][s]) > FAULT_STALE_MS)
            {
                m->faults |= staleFault[side][s];
            }
        }

        if (!(load[side] >= FAULT_LOAD_MIN && load[side] <= FAULT_LOAD_MAX))
        {
            m->faults |= rangeFault[side];
        }

        // stuck: a frozen value would make the torque feedback a constant offset
        if (!assisting || load[side] != m->loadLast[side])
        {
            m->loadLast[side] = load[side];
            m->loadChangeMs[side] = nowMs;
        }
        else if ((int32_t)(nowMs - m->loadChangeMs[side]) >= FAULT_LOAD_STUCK_MS)
        {
            m->faults |= stuckFault[side];
        }
    }

    return (m->faults & FAULT_STOP_MASK) != 0;
}
