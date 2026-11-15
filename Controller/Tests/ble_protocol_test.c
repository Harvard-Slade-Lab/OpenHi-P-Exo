/* Host test of the Bluetooth protocol parser and frame handling in ble_protocol.c.
 *
 * The byte stream reaches the parser as the main loop feeds it from the receive ring buffer:
 * one byte at a time, taking every complete frame after each byte, and flushing incomplete frames
 * when the stream pauses. Host frames are written in BLE chunks of at most 20 bytes.
 * Cases: CRC check value, valid frames, lost chunks (first or second chunk of a PARAMS, followed by
 * a retransmission or by STOPs), CRC error, garbage before a frame, two frames in one chunk,
 * duplicate SEQ, non-finite values, PARAMS after a latched fault, STOP, PING, unknown type,
 * wrong LEN, LEN > 24, wrong VER, the ACK encoding, and a random stream test (random loss and
 * corruption of chunks).
 */
// test-sources: Core/Src/ble_protocol.c
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ble_protocol.h"

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

#define CHUNK 20

/* Host side: frame encoder (CRC computed bit by bit here, independent of ble_crc16) ----------- */

static uint16_t crc_ref(const uint8_t *data, int len)
{
	uint16_t crc = 0xFFFF;
	for (int i = 0; i < len; i++)
	{
		for (int bit = 7; bit >= 0; bit--)
		{
			int in = (data[i] >> bit) & 1;
			int top = (crc >> 15) & 1;
			crc = (uint16_t)(crc << 1);
			if (in ^ top)
				crc ^= 0x1021;
		}
	}
	return crc;
}

static int encode(uint8_t *out, uint8_t type, uint8_t seq, const void *payload, uint8_t len)
{
	out[0] = 0xA5;
	out[1] = 0x5A;
	out[2] = 0x02;
	out[3] = type;
	out[4] = seq;
	out[5] = len;
	if (len)
		memcpy(&out[6], payload, len);
	uint16_t crc = crc_ref(&out[2], 4 + len);
	out[6 + len] = crc & 0xFF;
	out[7 + len] = crc >> 8;
	return 8 + len;
}

static int encode_params(uint8_t *out, uint8_t seq, const float p[6])
{
	return encode(out, BLE_TYPE_PARAMS, seq, p, 24);
}

/* Controller side: the main loop's use of the module ------------------------------------------ */

typedef struct
{
	uint8_t type, seq, status;
	BleCommand cmd;
	float params[6];
} Event;

static BleParser parser;
static BleSession session;
static Event events[4096];
static int nEvents;
static float applied[6]; // the profile target: what the controller would apply
static int stopped;
static int faulted; // the controller's latched fault (eStop)

static void controller_reset(void)
{
	ble_parser_reset(&parser);
	ble_session_reset(&session);
	nEvents = 0;
	memset(applied, 0, sizeof(applied));
	stopped = 0;
	faulted = 0;
}

static void execute(const BleFrame *frame)
{
	Event *e = &events[nEvents++];
	memset(e, 0, sizeof(*e));
	e->type = frame->type;
	e->seq = frame->seq;
	e->cmd = ble_handle_frame(&session, frame, faulted, e->params, &e->status);
	if (e->cmd == BLE_CMD_PARAMS)
	{
		memcpy(applied, e->params, sizeof(applied));
		stopped = 0;
	}
	else if (e->cmd == BLE_CMD_STOP)
	{
		stopped = 1;
	}
}

static void receive(const uint8_t *data, int n)
{
	BleFrame frame;
	for (int i = 0; i < n; i++)
	{
		ble_parser_push(&parser, data[i]);
		CHECK(parser.len <= BLE_MAX_FRAME, "parser buffer overflow");
		while (ble_parser_next(&parser, &frame))
			execute(&frame);
	}
}

static void pause_stream(void)
{
	BleFrame frame;
	while (ble_parser_flush(&parser, &frame))
		execute(&frame);
	CHECK(parser.len == 0, "flush left %d bytes", parser.len);
}

// Sends a frame in BLE chunks; chunks whose bit is set in lostMask are lost
static void send(const uint8_t *frame, int n, unsigned lostMask)
{
	for (int c = 0; c * CHUNK < n; c++)
	{
		int len = (n - c * CHUNK < CHUNK) ? n - c * CHUNK : CHUNK;
		if (!(lostMask & (1u << c)))
			receive(&frame[c * CHUNK], len);
	}
}

static int expect_event(int i, uint8_t type, uint8_t seq, uint8_t status, BleCommand cmd)
{
	if (i >= nEvents)
	{
		CHECK(0, "event %d missing (have %d)", i, nEvents);
		return 0;
	}
	Event *e = &events[i];
	CHECK(e->type == type && e->seq == seq && e->status == status && e->cmd == cmd,
		  "event %d: type 0x%02x seq %d status %d cmd %d, expected 0x%02x %d %d %d", i, e->type,
		  e->seq, e->status, e->cmd, type, seq, status, cmd);
	return 1;
}

static const float P1[6] = {-0.4f, 0.2f, 22.0f, 70.0f, 45.0f, 25.0f};
static const float P2[6] = {-0.6f, 0.3f, 12.0f, 62.0f, 37.0f, 33.0f};

static void test_crc(void)
{
	CHECK(ble_crc16((const uint8_t *)"123456789", 9) == 0x29B1, "CRC check value");
	CHECK(crc_ref((const uint8_t *)"123456789", 9) == 0x29B1, "reference CRC check value");
	uint8_t data[64];
	for (int n = 0; n < 64; n++)
	{
		for (int i = 0; i < n; i++)
			data[i] = (uint8_t)rand();
		CHECK(ble_crc16(data, n) == crc_ref(data, n), "CRC of %d random bytes", n);
	}
}

static void test_valid_frames(void)
{
	uint8_t f[64];
	int n;
	controller_reset();

	n = encode(f, BLE_TYPE_PING, 200, NULL, 0);
	send(f, n, 0);
	expect_event(0, BLE_TYPE_PING, 200, BLE_STATUS_APPLIED, BLE_CMD_NONE);

	n = encode_params(f, 201, P1);
	CHECK(n == 32, "PARAMS frame length %d", n);
	send(f, n, 0);
	expect_event(1, BLE_TYPE_PARAMS, 201, BLE_STATUS_APPLIED, BLE_CMD_PARAMS);
	CHECK(memcmp(applied, P1, sizeof(P1)) == 0, "PARAMS values");

	n = encode(f, BLE_TYPE_STOP, 202, NULL, 0);
	send(f, n, 0);
	expect_event(2, BLE_TYPE_STOP, 202, BLE_STATUS_APPLIED, BLE_CMD_STOP);
	CHECK(stopped, "STOP not applied");

	// a later PARAMS resumes assistance; SEQ wraps 255 -> 0
	n = encode_params(f, 255, P2);
	send(f, n, 0);
	n = encode_params(f, 0, P1);
	send(f, n, 0);
	expect_event(3, BLE_TYPE_PARAMS, 255, BLE_STATUS_APPLIED, BLE_CMD_PARAMS);
	expect_event(4, BLE_TYPE_PARAMS, 0, BLE_STATUS_APPLIED, BLE_CMD_PARAMS);
	CHECK(!stopped && memcmp(applied, P1, sizeof(P1)) == 0, "PARAMS after STOP");
	CHECK(nEvents == 5 && parser.len == 0, "events %d, parser %d bytes left", nEvents, parser.len);
}

static void test_fragment_loss(void)
{
	uint8_t f[64];
	int n;

	// second chunk lost, then the retransmission (same SEQ)
	controller_reset();
	n = encode_params(f, 10, P1);
	send(f, n, 1u << 1);
	CHECK(nEvents == 0, "frame without its second chunk was accepted");
	send(f, n, 0);
	expect_event(0, BLE_TYPE_PARAMS, 10, BLE_STATUS_APPLIED, BLE_CMD_PARAMS);
	CHECK(nEvents == 1 && memcmp(applied, P1, sizeof(P1)) == 0, "retransmission after lost chunk");

	// first chunk lost, then the retransmission
	controller_reset();
	send(f, n, 1u << 0);
	CHECK(nEvents == 0, "frame without its first chunk was accepted");
	send(f, n, 0);
	expect_event(0, BLE_TYPE_PARAMS, 10, BLE_STATUS_APPLIED, BLE_CMD_PARAMS);
	CHECK(nEvents == 1, "events %d", nEvents);

	// second chunk lost, then three STOPs: every STOP is applied
	controller_reset();
	send(f, n, 1u << 1);
	for (int k = 0; k < 3; k++)
	{
		int m = encode(f + 32, BLE_TYPE_STOP, (uint8_t)(11 + k), NULL, 0);
		send(f + 32, m, 0);
	}
	pause_stream();
	CHECK(nEvents == 3, "STOPs after a lost chunk: %d events", nEvents);
	for (int k = 0; k < 3; k++)
		expect_event(k, BLE_TYPE_STOP, (uint8_t)(11 + k), BLE_STATUS_APPLIED, BLE_CMD_STOP);
	CHECK(stopped, "STOP not applied after a lost chunk");

	// second chunk lost, one STOP: held until the stream pauses, then applied
	controller_reset();
	send(f, n, 1u << 1);
	int m = encode(f + 32, BLE_TYPE_STOP, 20, NULL, 0);
	send(f + 32, m, 0);
	CHECK(nEvents == 0, "STOP inside an incomplete frame taken before the pause");
	pause_stream();
	expect_event(0, BLE_TYPE_STOP, 20, BLE_STATUS_APPLIED, BLE_CMD_STOP);
	CHECK(nEvents == 1 && stopped, "STOP after the pause");

	// after the loss, every later frame still parses
	controller_reset();
	send(f, n, 1u << 0);
	for (int k = 0; k < 20; k++)
	{
		n = encode_params(f, (uint8_t)(30 + k), (k & 1) ? P1 : P2);
		send(f, n, 0);
	}
	CHECK(nEvents == 20, "frames after a loss: %d of 20", nEvents);
}

static void test_crc_error(void)
{
	uint8_t f[64];
	controller_reset();
	int n = encode_params(f, 40, P1);
	for (int i = 2; i < n; i++)
	{
		uint8_t g[64];
		memcpy(g, f, n);
		g[i] ^= 0x10;
		send(g, n, 0);
		pause_stream();
	}
	CHECK(nEvents == 0, "frame with a corrupted byte accepted (%d)", nEvents);
	send(f, n, 0);
	expect_event(0, BLE_TYPE_PARAMS, 40, BLE_STATUS_APPLIED, BLE_CMD_PARAMS);

	// corrupted frame immediately followed by a good one (no pause)
	controller_reset();
	uint8_t g[64];
	memcpy(g, f, n);
	g[10] ^= 0xFF;
	send(g, n, 0);
	n = encode(f, BLE_TYPE_STOP, 41, NULL, 0);
	send(f, n, 0);
	pause_stream();
	CHECK(nEvents == 1, "events %d", nEvents);
	expect_event(0, BLE_TYPE_STOP, 41, BLE_STATUS_APPLIED, BLE_CMD_STOP);
}

static void test_garbage_and_two_frames(void)
{
	uint8_t s[128];
	int n = 0;
	controller_reset();

	// garbage including partial SOF patterns and a fake header claiming LEN 24
	const uint8_t junk[] = {0x00, 0xA5, 0xA5, 0x5A, 0x01, 0xA5, 0x5A, 0x02, 0x01, 0x07, 0x18, 0xBB};
	memcpy(s, junk, sizeof(junk));
	n = sizeof(junk);
	n += encode_params(s + n, 50, P2);
	send(s, n, 0);
	pause_stream();
	CHECK(nEvents == 1, "garbage before a frame: %d events", nEvents);
	expect_event(0, BLE_TYPE_PARAMS, 50, BLE_STATUS_APPLIED, BLE_CMD_PARAMS);

	// two frames in one chunk
	controller_reset();
	n = encode(s, BLE_TYPE_PING, 60, NULL, 0);
	n += encode(s + n, BLE_TYPE_STOP, 61, NULL, 0);
	CHECK(n <= CHUNK, "chunk length %d", n);
	receive(s, n);
	CHECK(nEvents == 2, "two frames in one chunk: %d events", nEvents);
	expect_event(0, BLE_TYPE_PING, 60, BLE_STATUS_APPLIED, BLE_CMD_NONE);
	expect_event(1, BLE_TYPE_STOP, 61, BLE_STATUS_APPLIED, BLE_CMD_STOP);

	// a STOP and a PARAMS in one write split across chunk boundaries
	controller_reset();
	n = encode(s, BLE_TYPE_STOP, 62, NULL, 0);
	n += encode_params(s + n, 63, P1);
	send(s, n, 0);
	CHECK(nEvents == 2, "STOP + PARAMS: %d events", nEvents);
	expect_event(0, BLE_TYPE_STOP, 62, BLE_STATUS_APPLIED, BLE_CMD_STOP);
	expect_event(1, BLE_TYPE_PARAMS, 63, BLE_STATUS_APPLIED, BLE_CMD_PARAMS);
}

static void test_duplicate_seq(void)
{
	uint8_t f[64];
	int n;
	controller_reset();

	n = encode_params(f, 70, P1);
	send(f, n, 0);
	send(f, n, 0); // retransmission: ACKed, not applied again
	expect_event(0, BLE_TYPE_PARAMS, 70, BLE_STATUS_APPLIED, BLE_CMD_PARAMS);
	expect_event(1, BLE_TYPE_PARAMS, 70, BLE_STATUS_APPLIED, BLE_CMD_NONE);

	// same SEQ with other values (e.g. a restarted host) is still treated as the same frame
	n = encode_params(f, 70, P2);
	send(f, n, 0);
	expect_event(2, BLE_TYPE_PARAMS, 70, BLE_STATUS_APPLIED, BLE_CMD_NONE);
	CHECK(memcmp(applied, P1, sizeof(P1)) == 0, "duplicate SEQ re-applied");

	// a PING makes the controller forget the SEQ
	n = encode(f, BLE_TYPE_PING, 71, NULL, 0);
	send(f, n, 0);
	n = encode_params(f, 70, P2);
	send(f, n, 0);
	expect_event(4, BLE_TYPE_PARAMS, 70, BLE_STATUS_APPLIED, BLE_CMD_PARAMS);
	CHECK(memcmp(applied, P2, sizeof(P2)) == 0, "SEQ not forgotten after PING");

	// so does a STOP; STOP itself is always applied, even when repeated
	n = encode(f, BLE_TYPE_STOP, 72, NULL, 0);
	send(f, n, 0);
	send(f, n, 0);
	expect_event(5, BLE_TYPE_STOP, 72, BLE_STATUS_APPLIED, BLE_CMD_STOP);
	expect_event(6, BLE_TYPE_STOP, 72, BLE_STATUS_APPLIED, BLE_CMD_STOP);
	n = encode_params(f, 70, P1);
	send(f, n, 0);
	expect_event(7, BLE_TYPE_PARAMS, 70, BLE_STATUS_APPLIED, BLE_CMD_PARAMS);

	// a new SEQ is applied
	n = encode_params(f, 73, P2);
	send(f, n, 0);
	expect_event(8, BLE_TYPE_PARAMS, 73, BLE_STATUS_APPLIED, BLE_CMD_PARAMS);
	CHECK(nEvents == 9, "events %d", nEvents);
}

static void test_non_finite(void)
{
	uint8_t f[64];
	const float bad[] = {NAN, -NAN, INFINITY, -INFINITY};
	controller_reset();

	int n = encode_params(f, 80, P1);
	send(f, n, 0);
	int e = 1;
	for (int i = 0; i < 6; i++)
	{
		for (int b = 0; b < 4; b++)
		{
			float p[6];
			memcpy(p, P2, sizeof(p));
			p[i] = bad[b];
			n = encode_params(f, (uint8_t)(81 + e), p);
			send(f, n, 0);
			expect_event(e, BLE_TYPE_PARAMS, (uint8_t)(81 + e), BLE_STATUS_REJECTED, BLE_CMD_NONE);
			e++;
		}
	}
	// a signalling NaN pattern and the largest finite values
	float p[6];
	uint32_t snan = 0x7F800001u;
	memcpy(p, P2, sizeof(p));
	memcpy(&p[3], &snan, 4);
	n = encode_params(f, 120, p);
	send(f, n, 0);
	expect_event(e++, BLE_TYPE_PARAMS, 120, BLE_STATUS_REJECTED, BLE_CMD_NONE);
	CHECK(memcmp(applied, P1, sizeof(P1)) == 0, "rejected values were applied");

	memcpy(p, P2, sizeof(p));
	p[0] = -3.4028235e38f;
	p[1] = 1e-45f;
	n = encode_params(f, 121, p);
	send(f, n, 0);
	expect_event(e++, BLE_TYPE_PARAMS, 121, BLE_STATUS_APPLIED, BLE_CMD_PARAMS);

	// a rejected frame does not become the remembered SEQ
	n = encode_params(f, 122, P1);
	send(f, n, 0);
	p[0] = NAN;
	n = encode_params(f, 123, p);
	send(f, n, 0);
	n = encode_params(f, 123, P2);
	send(f, n, 0);
	expect_event(e++, BLE_TYPE_PARAMS, 122, BLE_STATUS_APPLIED, BLE_CMD_PARAMS);
	expect_event(e++, BLE_TYPE_PARAMS, 123, BLE_STATUS_REJECTED, BLE_CMD_NONE);
	expect_event(e++, BLE_TYPE_PARAMS, 123, BLE_STATUS_APPLIED, BLE_CMD_PARAMS);
	CHECK(memcmp(applied, P2, sizeof(P2)) == 0, "valid retransmission after a rejected frame");
}

// After a fault has latched assistance off: PARAMS gets STATUS 0x03 and is not applied, and its
// SEQ is not remembered; STOP and PING are still applied.
static void test_faulted(void)
{
	uint8_t f[64];
	int n;
	controller_reset();

	n = encode_params(f, 130, P1);
	send(f, n, 0);
	faulted = 1;
	send(f, n, 0); // retransmission of the applied frame: assistance is off now, so not applied
	n = encode_params(f, 131, P2);
	send(f, n, 0);
	send(f, n, 0);
	n = encode(f, BLE_TYPE_STOP, 132, NULL, 0);
	send(f, n, 0);
	n = encode(f, BLE_TYPE_PING, 133, NULL, 0);
	send(f, n, 0);
	float p[6];
	memcpy(p, P2, sizeof(p));
	p[2] = NAN;
	n = encode_params(f, 134, p);
	send(f, n, 0);
	expect_event(0, BLE_TYPE_PARAMS, 130, BLE_STATUS_APPLIED, BLE_CMD_PARAMS);
	expect_event(1, BLE_TYPE_PARAMS, 130, BLE_STATUS_FAULT, BLE_CMD_NONE);
	expect_event(2, BLE_TYPE_PARAMS, 131, BLE_STATUS_FAULT, BLE_CMD_NONE);
	expect_event(3, BLE_TYPE_PARAMS, 131, BLE_STATUS_FAULT, BLE_CMD_NONE);
	expect_event(4, BLE_TYPE_STOP, 132, BLE_STATUS_APPLIED, BLE_CMD_STOP);
	expect_event(5, BLE_TYPE_PING, 133, BLE_STATUS_FAULT, BLE_CMD_NONE);
	expect_event(6, BLE_TYPE_PARAMS, 134, BLE_STATUS_FAULT, BLE_CMD_NONE);
	CHECK(memcmp(applied, P1, sizeof(P1)) == 0, "PARAMS applied while faulted");
	CHECK(nEvents == 7, "events %d", nEvents);

	// SEQ 131 was not recorded while faulted: after a reset of the fault it is applied
	faulted = 0;
	n = encode_params(f, 131, P2);
	send(f, n, 0);
	expect_event(7, BLE_TYPE_PARAMS, 131, BLE_STATUS_APPLIED, BLE_CMD_PARAMS);
	CHECK(memcmp(applied, P2, sizeof(P2)) == 0, "SEQ recorded while faulted");
	n = encode(f, BLE_TYPE_PING, 135, NULL, 0);
	send(f, n, 0);
	expect_event(8, BLE_TYPE_PING, 135, BLE_STATUS_APPLIED, BLE_CMD_NONE);
}

static void test_type_and_len(void)
{
	uint8_t f[64], payload[32] = {0};
	int n;
	controller_reset();

	// unknown types (valid CRC): STATUS 0x02, nothing applied
	const uint8_t types[] = {0x00, 0x04, 0x7F, 0x80, 0x81, 0xFF};
	for (unsigned i = 0; i < sizeof(types); i++)
	{
		n = encode(f, types[i], (uint8_t)i, payload, 0);
		send(f, n, 0);
		expect_event(i, types[i], (uint8_t)i, BLE_STATUS_UNKNOWN, BLE_CMD_NONE);
	}
	int e = sizeof(types);

	// wrong LEN for known types
	const struct
	{
		uint8_t type, len;
	} wrong[] = {{BLE_TYPE_PARAMS, 0}, {BLE_TYPE_PARAMS, 23}, {BLE_TYPE_PARAMS, 20}, {BLE_TYPE_STOP, 1}, {BLE_TYPE_STOP, 24}, {BLE_TYPE_PING, 4}};
	for (unsigned i = 0; i < sizeof(wrong) / sizeof(wrong[0]); i++)
	{
		n = encode(f, wrong[i].type, (uint8_t)(100 + i), payload, wrong[i].len);
		send(f, n, 0);
		expect_event(e++, wrong[i].type, (uint8_t)(100 + i), BLE_STATUS_UNKNOWN, BLE_CMD_NONE);
	}
	CHECK(!stopped && applied[0] == 0.0f, "a frame with a wrong LEN was applied");

	// LEN > 24 and a wrong VER: not a frame (no ACK), and the next frame is still found
	controller_reset();
	uint8_t g[80];
	g[0] = 0xA5, g[1] = 0x5A, g[2] = 0x02, g[3] = BLE_TYPE_PARAMS, g[4] = 1, g[5] = 25;
	memset(&g[6], 0, 25);
	uint16_t crc = crc_ref(&g[2], 4 + 25);
	g[31] = crc & 0xFF, g[32] = crc >> 8;
	n = 33;
	n += encode(g + n, BLE_TYPE_STOP, 2, NULL, 0);
	send(g, n, 0);
	pause_stream();
	CHECK(nEvents == 1, "LEN > 24: %d events", nEvents);
	expect_event(0, BLE_TYPE_STOP, 2, BLE_STATUS_APPLIED, BLE_CMD_STOP);

	controller_reset();
	n = encode(g, BLE_TYPE_STOP, 3, NULL, 0);
	g[2] = 0x01; // wrong version byte
	crc = crc_ref(&g[2], 4);
	g[6] = crc & 0xFF, g[7] = crc >> 8;
	n += encode(g + n, BLE_TYPE_PING, 4, NULL, 0);
	send(g, n, 0);
	pause_stream();
	CHECK(nEvents == 1, "wrong VER: %d events", nEvents);
	expect_event(0, BLE_TYPE_PING, 4, BLE_STATUS_APPLIED, BLE_CMD_NONE);

	// a stream without the SOF bytes (0xAA 0xAA + 24 bytes + 0xBB 0xBB) is ignored
	controller_reset();
	g[0] = g[1] = 0xAA;
	memcpy(&g[2], P1, 24);
	g[26] = g[27] = 0xBB;
	send(g, 28, 0);
	pause_stream();
	CHECK(nEvents == 0, "frame without SOF accepted");
}

static void test_ack(void)
{
	uint8_t a[BLE_ACK_FRAME], ref[16];
	for (int seq = 0; seq < 256; seq++)
	{
		for (int st = 0; st < 3; st++)
		{
			uint8_t payload[2] = {(uint8_t)(seq % 3 + 1), (uint8_t)st};
			ble_encode_ack(a, (uint8_t)seq, payload[0], (uint8_t)st);
			int n = encode(ref, BLE_TYPE_ACK, (uint8_t)seq, payload, 2);
			CHECK(n == BLE_ACK_FRAME && memcmp(a, ref, n) == 0, "ACK seq %d status %d", seq, st);
		}
	}
	// an ACK is itself a valid frame for the parser (the host uses the same format)
	controller_reset();
	ble_encode_ack(a, 9, BLE_TYPE_STOP, BLE_STATUS_APPLIED);
	receive(a, BLE_ACK_FRAME);
	expect_event(0, BLE_TYPE_ACK, 9, BLE_STATUS_UNKNOWN, BLE_CMD_NONE);
}

// Random stream: frames with random loss and corruption of chunks and random garbage between
// writes. Every frame delivered intact must be found; nothing else may be accepted.
static void test_random_stream(void)
{
	int intact = 0, found = 0, wrongAccepted = 0;

	for (int round = 0; round < 20000; round++)
	{
		controller_reset();
		uint8_t f[64];
		uint8_t seq = (uint8_t)round;
		int kind = rand() % 3;
		int n = (kind == 0) ? encode_params(f, seq, P1)
							: encode(f, kind == 1 ? BLE_TYPE_STOP : BLE_TYPE_PING, seq, NULL, 0);

		// garbage before
		uint8_t junk[40];
		int nj = rand() % 40;
		for (int i = 0; i < nj; i++)
			junk[i] = (rand() % 4 == 0) ? 0xA5 : (uint8_t)rand();
		receive(junk, nj);
		if (rand() % 2)
			pause_stream();

		// the frame, sometimes with a lost or corrupted chunk
		int fate = rand() % 4; // 0, 1: intact; 2: chunk lost; 3: byte corrupted
		uint8_t g[64];
		memcpy(g, f, n);
		unsigned lost = 0;
		if (fate == 2)
			lost = 1u << (rand() % ((n + CHUNK - 1) / CHUNK));
		if (fate == 3)
			g[rand() % n] ^= (uint8_t)(1 + rand() % 255);
		int before = nEvents;
		send(g, n, lost);
		pause_stream();
		int ok = (fate < 2);
		if (ok)
			intact++;
		for (int i = before; i < nEvents; i++)
		{
			if (ok && events[i].seq == seq && events[i].type == f[3])
				found++;
			else
				wrongAccepted++;
		}
	}
	CHECK(found == intact, "random stream: %d of %d intact frames found", found, intact);
	// random garbage can form a valid frame only by matching SOF, VER, LEN and CRC
	CHECK(wrongAccepted == 0, "random stream: %d frames accepted that were not sent intact", wrongAccepted);
	printf("random stream: %d intact frames, all found; 0 other frames accepted\n", intact);
}

int main(void)
{
	srand(1);
	test_crc();
	test_valid_frames();
	test_fragment_loss();
	test_crc_error();
	test_garbage_and_two_frames();
	test_duplicate_seq();
	test_non_finite();
	test_faulted();
	test_type_and_len();
	test_ack();
	test_random_stream();

	printf("ble_protocol_test: %ld checks, %ld failures\n", checks, failures);
	return failures ? 1 : 0;
}
