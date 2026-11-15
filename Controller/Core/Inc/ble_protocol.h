#ifndef INC_BLE_PROTOCOL_H_
#define INC_BLE_PROTOCOL_H_

#include <stdint.h>

// Bluetooth protocol (host <-> controller over the HM-10 BLE-UART bridge), little-endian:
// SOF0 SOF1 VER TYPE SEQ LEN PAYLOAD[LEN] CRC_LO CRC_HI
// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over VER TYPE SEQ LEN PAYLOAD.

#define BLE_SOF0 0xA5
#define BLE_SOF1 0x5A
#define BLE_VERSION 0x02

#define BLE_HEADER_BYTES 6
#define BLE_CRC_BYTES 2
#define BLE_MAX_PAYLOAD 24
#define BLE_MAX_FRAME (BLE_HEADER_BYTES + BLE_MAX_PAYLOAD + BLE_CRC_BYTES)

// Host -> controller
#define BLE_TYPE_PARAMS 0x01 // LEN 24: six float32 profile parameters
#define BLE_TYPE_STOP 0x02   // LEN 0: ramp assistance to zero
#define BLE_TYPE_PING 0x03   // LEN 0: link check

// Controller -> host: PAYLOAD = [acked TYPE, STATUS], SEQ = SEQ of the acked frame
#define BLE_TYPE_ACK 0x80
#define BLE_ACK_FRAME (BLE_HEADER_BYTES + 2 + BLE_CRC_BYTES)

#define BLE_STATUS_APPLIED 0x00
#define BLE_STATUS_REJECTED 0x01 // non-finite parameter
#define BLE_STATUS_UNKNOWN 0x02  // unknown TYPE or wrong LEN
#define BLE_STATUS_FAULT 0x03    // a fault has latched assistance off until reset (PARAMS not applied)

#define BLE_PARAM_COUNT 6
#define BLE_PARAMS_LEN (BLE_PARAM_COUNT * 4) // float32, little-endian

typedef struct
{
    uint8_t buf[BLE_MAX_FRAME];
    uint8_t len;
} BleParser;

typedef struct
{
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[BLE_MAX_PAYLOAD];
} BleFrame;

typedef struct
{
    uint8_t lastParamsSeq;
    uint8_t lastParamsSeqValid;
} BleSession;

typedef enum
{
    BLE_CMD_NONE = 0,
    BLE_CMD_PARAMS,
    BLE_CMD_STOP
} BleCommand;

uint16_t ble_crc16(const uint8_t *data, uint16_t len);

void ble_parser_reset(BleParser *parser);
// Appends one received byte. Call ble_parser_next until it returns 0 before the next byte.
void ble_parser_push(BleParser *parser, uint8_t byte);
// Returns 1 and fills *frame when a whole frame with a valid CRC is buffered; drops one byte
// and searches again on a wrong SOF, VER, LEN > 24 or CRC.
int ble_parser_next(BleParser *parser, BleFrame *frame);
// After the byte stream has paused: treats every incomplete frame in the buffer as lost, so frames
// buffered behind it are found. Returns 1 per frame found; call until it returns 0.
int ble_parser_flush(BleParser *parser, BleFrame *frame);

void ble_session_reset(BleSession *session);
// Decides the ACK status and the command for a valid frame. BLE_CMD_PARAMS fills params[] and
// records its SEQ as applied, so the caller must apply it; a PARAMS repeating that SEQ is ACKed
// but not applied again. STOP and PING forget the SEQ. While faulted (assistance latched off),
// a PARAMS frame gets BLE_STATUS_FAULT and its SEQ is not recorded, and a PING gets
// BLE_STATUS_FAULT; STOP is still applied.
BleCommand ble_handle_frame(BleSession *session, const BleFrame *frame, int faulted,
                            float params[BLE_PARAM_COUNT], uint8_t *status);
// Writes the ACK frame (BLE_ACK_FRAME bytes) to out.
void ble_encode_ack(uint8_t out[BLE_ACK_FRAME], uint8_t seq, uint8_t ackedType, uint8_t status);

#endif /* INC_BLE_PROTOCOL_H_ */
