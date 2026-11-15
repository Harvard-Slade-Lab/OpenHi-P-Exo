#include <string.h>
#include <stdint.h>

#include "ble_protocol.h"

uint16_t ble_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++)
        {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void ble_parser_reset(BleParser *parser)
{
    parser->len = 0;
}

static void drop(BleParser *parser, uint8_t count)
{
    parser->len -= count;
    memmove(parser->buf, &parser->buf[count], parser->len);
}

void ble_parser_push(BleParser *parser, uint8_t byte)
{
    if (parser->len == BLE_MAX_FRAME)
    {
        drop(parser, 1);
    }
    parser->buf[parser->len++] = byte;
}

int ble_parser_next(BleParser *parser, BleFrame *frame)
{
    uint8_t *b = parser->buf;

    while (parser->len > 0)
    {
        if (b[0] != BLE_SOF0)
        {
            drop(parser, 1);
            continue;
        }
        if (parser->len < 2)
            return 0;
        if (b[1] != BLE_SOF1)
        {
            drop(parser, 1);
            continue;
        }
        if (parser->len < 3)
            return 0;
        if (b[2] != BLE_VERSION)
        {
            drop(parser, 1);
            continue;
        }
        if (parser->len < BLE_HEADER_BYTES)
            return 0;
        if (b[5] > BLE_MAX_PAYLOAD)
        {
            drop(parser, 1);
            continue;
        }

        uint8_t total = BLE_HEADER_BYTES + b[5] + BLE_CRC_BYTES;
        if (parser->len < total)
            return 0;

        uint16_t crc = (uint16_t)b[total - 2] | ((uint16_t)b[total - 1] << 8);
        if (ble_crc16(&b[2], total - 2 - BLE_CRC_BYTES) != crc)
        {
            drop(parser, 1);
            continue;
        }

        frame->type = b[3];
        frame->seq = b[4];
        frame->len = b[5];
        memcpy(frame->payload, &b[BLE_HEADER_BYTES], frame->len);
        drop(parser, total);
        return 1;
    }
    return 0;
}

int ble_parser_flush(BleParser *parser, BleFrame *frame)
{
    while (parser->len > 0)
    {
        if (ble_parser_next(parser, frame))
            return 1;
        if (parser->len > 0)
            drop(parser, 1); // incomplete frame whose remaining bytes never came
    }
    return 0;
}

void ble_session_reset(BleSession *session)
{
    session->lastParamsSeq = 0;
    session->lastParamsSeqValid = 0;
}

// Exponent bits all set: Inf or NaN (independent of compiler floating-point options)
static int is_finite(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7F800000u) != 0x7F800000u;
}

BleCommand ble_handle_frame(BleSession *session, const BleFrame *frame, int faulted,
                            float params[BLE_PARAM_COUNT], uint8_t *status)
{
    *status = BLE_STATUS_UNKNOWN;

    switch (frame->type)
    {
    case BLE_TYPE_PARAMS:
    {
        if (frame->len != BLE_PARAMS_LEN)
            return BLE_CMD_NONE;
        if (faulted)
        { // the motors get zero current until reset: tell the host the parameters are not applied
            *status = BLE_STATUS_FAULT;
            return BLE_CMD_NONE;
        }
        float values[BLE_PARAM_COUNT];
        memcpy(values, frame->payload, sizeof(values));
        for (int i = 0; i < BLE_PARAM_COUNT; i++)
        {
            if (!is_finite(values[i]))
            {
                *status = BLE_STATUS_REJECTED;
                return BLE_CMD_NONE;
            }
        }
        *status = BLE_STATUS_APPLIED;
        if (session->lastParamsSeqValid && session->lastParamsSeq == frame->seq)
            return BLE_CMD_NONE; // retransmission of the applied frame
        session->lastParamsSeq = frame->seq;
        session->lastParamsSeqValid = 1;
        memcpy(params, values, sizeof(values));
        return BLE_CMD_PARAMS;
    }

    case BLE_TYPE_STOP:
    case BLE_TYPE_PING:
        if (frame->len != 0)
            return BLE_CMD_NONE;
        session->lastParamsSeqValid = 0;
        if (frame->type == BLE_TYPE_STOP)
        { // applied also while faulted
            *status = BLE_STATUS_APPLIED;
            return BLE_CMD_STOP;
        }
        // PING reports a latched fault, so the host can check a condition before using it
        *status = faulted ? BLE_STATUS_FAULT : BLE_STATUS_APPLIED;
        return BLE_CMD_NONE;

    default:
        return BLE_CMD_NONE;
    }
}

void ble_encode_ack(uint8_t out[BLE_ACK_FRAME], uint8_t seq, uint8_t ackedType, uint8_t status)
{
    out[0] = BLE_SOF0;
    out[1] = BLE_SOF1;
    out[2] = BLE_VERSION;
    out[3] = BLE_TYPE_ACK;
    out[4] = seq;
    out[5] = 2;
    out[6] = ackedType;
    out[7] = status;

    uint16_t crc = ble_crc16(&out[2], BLE_ACK_FRAME - 2 - BLE_CRC_BYTES);
    out[8] = (uint8_t)(crc & 0xFF);
    out[9] = (uint8_t)(crc >> 8);
}
