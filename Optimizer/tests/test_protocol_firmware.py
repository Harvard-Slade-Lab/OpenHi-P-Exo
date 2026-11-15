"""The Bluetooth protocol end to end: the optimizer's frames against the controller's own C code.

Controller/Core/Src/ble_protocol.c (frame parser, frame handling and ACK encoder of the
firmware) is compiled with the host C compiler and driven as the controller's main loop drives
it (process_ble_data in Controller/Core/Src/main.c): one byte at a time, every complete frame
handled after each byte, and incomplete frames flushed when no byte arrived for 200 ms. The
optimizer's encoder and writer task send to it, and its ACKs go back to the optimizer's ACK
parser. Skipped when no working C compiler is found (set CC to choose one).

Run from the repository root:
    python -m unittest discover -s Optimizer/tests -t Optimizer
"""
import math
import os
import random
import shutil
import struct
import subprocess
import tempfile
import time
import unittest

from tests import _stubs  # noqa: F401  (before the optimizer imports)
import bleFunctions as ble
from tests._ble_mock import LinkThread, MockClient

CONTROLLER = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'Controller'))


def working_cc():
    """The C compiler to use, or None when none is found or it cannot build a trivial
    program (e.g. the macOS cc stub without the command line tools)."""
    cc = os.environ.get('CC') or shutil.which('cc') or shutil.which('gcc') or shutil.which('clang')
    if cc is None:
        return None
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, 'probe.c')
        with open(src, 'w') as f:
            f.write('#include <stdio.h>\nint main(void) { return 0; }\n')
        try:
            subprocess.run([cc, '-o', os.path.join(tmp, 'probe'), src], check=True,
                           capture_output=True, timeout=120)
        except (OSError, subprocess.SubprocessError):
            return None
    return cc


CC = working_cc()
PARTIAL_TIMEOUT = 0.2  # [s] BLE_PARTIAL_TIMEOUT_MS in Controller/Core/Src/main.c
P1 = [-10.0, 6.0, 25.0, 70.0, 40.0, 20.0]

# One command per input line, one answer line per command:
#   D <hex>   bytes received: push them one at a time and handle every complete frame
#   F         the byte stream paused: flush incomplete frames
#   Z <0|1>   latched fault off / on
# For each handled frame the answer has "A<ACK frame hex>", then "P<payload hex>" when the
# parameters are applied or "S" for a STOP.
DRIVER = r'''
#include <stdio.h>
#include "ble_protocol.h"

static BleParser parser;
static BleSession session;
static int faulted;

static void handle(const BleFrame *frame)
{
    float params[BLE_PARAM_COUNT];
    uint8_t status, ack[BLE_ACK_FRAME];
    BleCommand cmd = ble_handle_frame(&session, frame, faulted, params, &status);
    ble_encode_ack(ack, frame->seq, frame->type, status);
    printf(" A");
    for (int i = 0; i < BLE_ACK_FRAME; i++)
        printf("%02x", ack[i]);
    if (cmd == BLE_CMD_PARAMS)
    {
        const unsigned char *bytes = (const unsigned char *)params;
        printf(" P");
        for (int i = 0; i < BLE_PARAMS_LEN; i++)
            printf("%02x", bytes[i]);
    }
    else if (cmd == BLE_CMD_STOP)
        printf(" S");
}

int main(void)
{
    static char line[8192];
    BleFrame frame;
    ble_parser_reset(&parser);
    ble_session_reset(&session);
    while (fgets(line, sizeof(line), stdin))
    {
        if (line[0] == 'D')
        {
            unsigned byte;
            for (const char *s = line + 2; sscanf(s, "%2x", &byte) == 1; s += 2)
            {
                ble_parser_push(&parser, (uint8_t)byte);
                while (ble_parser_next(&parser, &frame))
                    handle(&frame);
            }
        }
        else if (line[0] == 'F')
        {
            while (ble_parser_flush(&parser, &frame))
                handle(&frame);
        }
        else if (line[0] == 'Z')
            faulted = line[2] == '1';
        printf("\n");
        fflush(stdout);
    }
    return 0;
}
'''


class FirmwareController:
    """The firmware's protocol code in a child process, with the interface of the mock
    controller in tests/_ble_mock.py (receive() returns the ACK bytes; applied)."""
    def __init__(self, exe):
        self.proc = subprocess.Popen([exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     universal_newlines=True)
        self.applied = []  # ('PARAMS', params) and ('STOP',) in the order applied
        self.last_chunk = None

    def command(self, line):
        self.proc.stdin.write(line + '\n')
        self.proc.stdin.flush()
        acks = b''
        for token in self.proc.stdout.readline().split():
            if token[0] == 'A':
                acks += bytes.fromhex(token[1:])
            elif token[0] == 'P':
                self.applied.append(('PARAMS', list(struct.unpack('<6f', bytes.fromhex(token[1:])))))
            elif token == 'S':
                self.applied.append(('STOP',))
        return acks

    def receive(self, data):
        """Bytes from the HM-10; returns the ACK bytes the controller sends back. A pause of
        PARTIAL_TIMEOUT before the bytes flushes incomplete frames first, as the main loop does."""
        now = time.monotonic()
        acks = b''
        if self.last_chunk is not None and now - self.last_chunk >= PARTIAL_TIMEOUT:
            acks += self.command('F')
        self.last_chunk = now
        return acks + self.command('D ' + bytes(data).hex())

    def set_fault(self, on):
        self.command(f'Z {int(on)}')

    def close(self):
        self.proc.stdin.close()
        self.proc.wait(timeout=5)
        self.proc.stdout.close()


@unittest.skipIf(CC is None, "no working C compiler (set CC)")
class FirmwareCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        driver = os.path.join(cls._tmp.name, 'driver.c')
        with open(driver, 'w') as f:
            f.write(DRIVER)
        cls.exe = os.path.join(cls._tmp.name, 'ble_protocol_driver')
        try:
            subprocess.run([CC, '-std=gnu11', '-O2', '-Wall', '-I', os.path.join(CONTROLLER, 'Core', 'Inc'),
                            '-o', cls.exe, driver, os.path.join(CONTROLLER, 'Core', 'Src', 'ble_protocol.c')],
                           check=True, capture_output=True)
        except subprocess.CalledProcessError as e:
            cls._tmp.cleanup()
            raise AssertionError(e.stderr.decode(errors='replace')) from e

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def setUp(self):
        self.controller = FirmwareController(self.exe)

    def tearDown(self):
        self.controller.close()

    def send(self, frame):
        """Write a frame in BLE chunks as the optimizer does; returns the ACKs as (TYPE, SEQ,
        PAYLOAD), read with the optimizer's parser."""
        acks = b''
        for i in range(0, len(frame), ble.CHUNK_SIZE):
            acks += self.controller.receive(frame[i:i + ble.CHUNK_SIZE])
        return ble.FrameParser().feed(acks)


class TestFrames(FirmwareCase):
    def test_parameters_arrive_bit_exact(self):
        rng = random.Random(7)
        sent = []
        for seq in range(256):
            params = [rng.uniform(-60.0, 60.0) for _ in range(6)]
            if seq % 16 == 0:
                params = [0.0, -0.0, 1e-38, -3.4e38, 3.4e38, 100.0]
            acks = self.send(ble.encode_frame(ble.TYPE_PARAMS, seq, ble.params_payload(params)))
            self.assertEqual(acks, [(ble.TYPE_ACK, seq, bytes([ble.TYPE_PARAMS, ble.ACK_APPLIED]))])
            sent.append(list(struct.unpack('<6f', ble.params_payload(params))))
        received = [p for kind, p in self.controller.applied]
        self.assertEqual(len(received), 256)
        for got, want in zip(received, sent):
            self.assertEqual(struct.pack('<6f', *got), struct.pack('<6f', *want))

    def test_ack_bytes_match_the_optimizer_encoder(self):
        for frame_type, seq, payload, status in [
                (ble.TYPE_PING, 0, b'', ble.ACK_APPLIED),
                (ble.TYPE_STOP, 255, b'', ble.ACK_APPLIED),
                (ble.TYPE_PARAMS, 17, ble.params_payload(P1), ble.ACK_APPLIED),
                (ble.TYPE_PARAMS, 18, ble.params_payload([math.nan] + P1[1:]), ble.ACK_REJECTED),
                (0x07, 19, b'', ble.ACK_UNKNOWN)]:
            frame = ble.encode_frame(frame_type, seq, payload)
            acks = b''.join(self.controller.receive(frame[i:i + ble.CHUNK_SIZE])
                            for i in range(0, len(frame), ble.CHUNK_SIZE))
            self.assertEqual(acks, ble.encode_frame(ble.TYPE_ACK, seq, bytes([frame_type, status])))

    def test_statuses(self):
        ack = lambda frame_type, seq, status: [(ble.TYPE_ACK, seq, bytes([frame_type, status]))]
        nan_params = ble.params_payload(P1[:2] + [math.nan] + P1[3:])
        inf_params = ble.params_payload(P1[:5] + [math.inf])
        self.assertEqual(self.send(ble.encode_frame(ble.TYPE_PARAMS, 1, nan_params)),
                         ack(ble.TYPE_PARAMS, 1, ble.ACK_REJECTED))
        self.assertEqual(self.send(ble.encode_frame(ble.TYPE_PARAMS, 2, inf_params)),
                         ack(ble.TYPE_PARAMS, 2, ble.ACK_REJECTED))
        self.assertEqual(self.send(ble.encode_frame(ble.TYPE_PARAMS, 3, b'')),
                         ack(ble.TYPE_PARAMS, 3, ble.ACK_UNKNOWN))
        self.assertEqual(self.send(ble.encode_frame(0x07, 4)), ack(0x07, 4, ble.ACK_UNKNOWN))
        # Parameters without the frame header and CRC are not accepted
        self.assertEqual(self.send(b'\xaa\xaa' + ble.params_payload(P1) + b'\xbb\xbb'), [])
        # A retransmission of applied parameters is acknowledged but not applied again
        frame = ble.encode_frame(ble.TYPE_PARAMS, 5, ble.params_payload(P1))
        self.assertEqual(self.send(frame), ack(ble.TYPE_PARAMS, 5, ble.ACK_APPLIED))
        self.assertEqual(self.send(frame), ack(ble.TYPE_PARAMS, 5, ble.ACK_APPLIED))
        # A latched fault: parameters are not applied, PING reports it, STOP is applied
        self.controller.set_fault(True)
        self.assertEqual(self.send(ble.encode_frame(ble.TYPE_PARAMS, 6, ble.params_payload(P1))),
                         ack(ble.TYPE_PARAMS, 6, ble.ACK_FAULT))
        self.assertEqual(self.send(ble.encode_frame(ble.TYPE_PING, 7)), ack(ble.TYPE_PING, 7, ble.ACK_FAULT))
        self.assertEqual(self.send(ble.encode_frame(ble.TYPE_STOP, 8)), ack(ble.TYPE_STOP, 8, ble.ACK_APPLIED))
        self.controller.set_fault(False)
        self.assertEqual(self.send(ble.encode_frame(ble.TYPE_PING, 9)), ack(ble.TYPE_PING, 9, ble.ACK_APPLIED))
        self.assertEqual(self.controller.applied, [('PARAMS', P1), ('STOP',)])


class TestWriter(FirmwareCase):
    """The optimizer's writer task (queue, chunks, ACK wait, retransmission) against the
    firmware's protocol code."""
    def link(self, **client):
        sender = ble.DataSender()
        sender.ack_timeout = 0.3  # above PARTIAL_TIMEOUT, as 0.5 s is above 200 ms on the device
        self.faults = []
        sender.on_link_fault = lambda kind, message: self.faults.append((kind, message))
        return LinkThread(MockClient(self.controller, **client), sender)

    def test_commands_are_applied(self):
        with self.link() as link:
            self.assertTrue(link.sender.check_link())
            params = link.sender.send_params(P1)
            self.assertTrue(params.done.wait(3.0))
            stop = link.sender.send_stop()
            self.assertTrue(stop.done.wait(3.0))
        self.assertEqual((params.status, stop.status), ('applied', 'applied'))
        self.assertEqual(link.sender.current_params, P1)
        self.assertEqual(self.controller.applied, [('PARAMS', P1), ('STOP',)])
        self.assertEqual(self.faults, [])

    def test_lost_chunks(self):
        # Chunks 0-1: PARAMS; chunk 1 is lost, so the first chunk waits in the parser until
        # the pause flushes it and the retransmission (chunks 2-3) is applied. Chunk 4: the
        # first PARAMS chunk of the next frame is lost; its second chunk is dropped by resync
        with self.link(lose={1, 4}) as link:
            first = link.sender.send_params(P1)
            self.assertTrue(first.done.wait(5.0))
            second_params = [v + 1.0 for v in P1]
            second = link.sender.send_params(second_params)
            self.assertTrue(second.done.wait(5.0))
            stop = link.sender.send_stop()
            self.assertTrue(stop.done.wait(3.0))
        self.assertEqual([first.status, second.status, stop.status], ['applied'] * 3)
        self.assertEqual(self.controller.applied, [('PARAMS', P1), ('PARAMS', second_params), ('STOP',)])
        self.assertEqual(self.faults, [])

    def test_controller_fault(self):
        with self.link() as link:
            self.assertTrue(link.sender.check_link())
            self.controller.set_fault(True)
            params = link.sender.send_params(P1)
            self.assertTrue(params.done.wait(3.0))
            self.assertFalse(link.sender.check_link())
            self.assertTrue(link.sender.controller_faulted)
            self.controller.set_fault(False)  # the controller was reset
            self.assertTrue(link.sender.check_link())
            self.assertFalse(link.sender.controller_faulted)
        self.assertEqual(params.status, 'rejected')
        self.assertEqual(self.controller.applied, [])
        self.assertEqual(self.faults, [('PARAMS', ble.CONTROLLER_FAULT), ('PING', ble.CONTROLLER_FAULT)])


if __name__ == '__main__':
    unittest.main()
