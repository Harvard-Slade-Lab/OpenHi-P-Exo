"""The Bluetooth protocol on the host: frames, ACK parser, retransmission and link faults.

The mock controller in tests/_ble_mock.py implements the controller side as specified
(resync, CRC, ACK, SEQ dedupe), with chunk loss, delayed and missing ACKs.

Run from the repository root:
    python -m unittest discover -s Optimizer/tests -t Optimizer
"""
import asyncio
import contextlib
import io
import itertools
import math
import struct
import tempfile
import time
import unittest
from unittest import mock

from tests import _stubs  # noqa: F401  (before the optimizer imports)
import bleFunctions as ble
from bleFunctions import DataSender, FrameParser
from tests._ble_mock import LinkThread, MockClient, MockController, crc16, frame
from tests._harness import make_controller, measure_valid, run_loop

P1 = [-10.0, 6.0, 25.0, 70.0, 40.0, 20.0]
STOP_ALERT = "Stop not acknowledged: use the control-mode switch or the emergency stop"


def ack(seq, acked_type, status=0):
    return frame(0x80, seq, bytes([acked_type, status]))


class TestFrames(unittest.TestCase):
    def test_crc_check_value(self):
        self.assertEqual(ble.crc16_ccitt(b'123456789'), 0x29B1)
        self.assertEqual(crc16(b'123456789'), 0x29B1)

    def test_stop_and_ping_frames(self):
        crc = ble.crc16_ccitt(bytes([0x02, 0x02, 0x05, 0x00]))
        self.assertEqual(ble.encode_frame(ble.TYPE_STOP, 5),
                         bytes([0xA5, 0x5A, 0x02, 0x02, 0x05, 0x00, crc & 0xFF, crc >> 8]))
        self.assertEqual(ble.encode_frame(ble.TYPE_PING, 255), frame(0x03, 255))

    def test_params_frame(self):
        payload = ble.params_payload(P1)
        encoded = ble.encode_frame(ble.TYPE_PARAMS, 7, payload)
        self.assertEqual(len(encoded), 32)
        self.assertEqual(encoded, frame(0x01, 7, payload))
        self.assertEqual(list(struct.unpack('<6f', encoded[6:30])), P1)
        with self.assertRaises(ValueError):
            ble.params_payload(P1[:5])


class TestFrameParser(unittest.TestCase):
    def test_resync_after_garbage_and_bad_frames(self):
        good1, good2 = ack(1, 0x01), ack(2, 0x02)
        bad_crc = bytearray(ack(3, 0x01))
        bad_crc[-1] ^= 0xFF
        bad_ver = bytearray(ack(4, 0x01))
        bad_ver[2] = 0x01
        long_len = bytes([0xA5, 0x5A, 0x02, 0x80, 0x05, 25])
        stream = b'\x00\xa5\x13' + good1 + bytes(bad_crc) + bytes(bad_ver) + long_len + good2
        parser = FrameParser()
        frames = parser.feed(stream)
        self.assertEqual(frames, [(0x80, 1, bytes([0x01, 0])), (0x80, 2, bytes([0x02, 0]))])

    def test_byte_by_byte_and_two_frames_in_one_chunk(self):
        parser = FrameParser()
        frames = []
        for b in ack(9, 0x03):
            frames += parser.feed(bytes([b]))
        frames += parser.feed(ack(10, 0x01) + ack(11, 0x02))
        self.assertEqual([f[1] for f in frames], [9, 10, 11])

    def test_partial_frame_waits(self):
        parser = FrameParser()
        whole = ack(12, 0x01)
        self.assertEqual(parser.feed(whole[:5]), [])
        self.assertEqual(parser.feed(whole[5:]), [(0x80, 12, bytes([0x01, 0]))])


class LinkCase(unittest.TestCase):
    def link(self, ack_timeout=0.1, retransmissions=3, **client):
        sender = DataSender()
        sender.ack_timeout = ack_timeout
        sender.retransmissions = retransmissions
        self.faults = []
        sender.on_link_fault = lambda kind, message: self.faults.append((kind, message))
        return LinkThread(MockClient(**client), sender)


class TestLink(LinkCase):
    def test_lost_chunk_is_retransmitted(self):
        with self.link(lose={0}) as link:
            command = link.sender.send_params(P1)
            self.assertTrue(command.done.wait(3.0))
        self.assertEqual(command.status, 'applied')
        self.assertEqual(link.received(), [('PARAMS', P1)])
        self.assertEqual(len(link.client.chunks), 4)
        self.assertTrue(all(len(c) <= 20 for c in link.client.chunks))
        self.assertEqual(link.sender.current_params, P1)
        self.assertEqual(self.faults, [])

    def test_lost_chunk_then_repeated_stops(self):
        # Every tail chunk of the PARAMS frame is lost, so the controller holds partial
        # frames when the Stops arrive: it resynchronizes and applies every Stop.
        with self.link(lose={1, 3, 5, 7}) as link:
            params = link.sender.send_params(P1)
            self.assertTrue(params.done.wait(3.0))
            stops = []
            for _ in range(3):
                stops.append(link.sender.send_stop())
                self.assertTrue(stops[-1].done.wait(3.0))
        self.assertEqual(params.status, 'no_ack')
        self.assertEqual(self.faults[0][0], 'PARAMS')
        self.assertEqual([s.status for s in stops], ['applied'] * 3)
        received = link.received()
        self.assertNotIn('PARAMS', [c[0] for c in received])
        self.assertGreaterEqual(len(received), 3)
        self.assertEqual(received[-1], ('STOP',))

    def test_lost_chunk_then_retransmission_and_stops(self):
        with self.link(lose={1}) as link:
            params = link.sender.send_params(P1)
            self.assertTrue(params.done.wait(3.0))
            stops = [link.sender.send_stop() for _ in range(1)]
            self.assertTrue(stops[0].done.wait(3.0))
            stops.append(link.sender.send_stop())
            self.assertTrue(stops[1].done.wait(3.0))
        self.assertEqual(params.status, 'applied')
        self.assertEqual(link.received(), [('PARAMS', P1), ('STOP',), ('STOP',)])

    def test_delayed_ack_within_timeout(self):
        with self.link(ack_timeout=0.3, ack_delay=0.05) as link:
            command = link.sender.send_params(P1)
            self.assertTrue(command.done.wait(3.0))
        self.assertEqual(command.status, 'applied')
        self.assertEqual(len(link.client.chunks), 2)

    def test_late_ack_retransmits_without_reapplying(self):
        with self.link(ack_timeout=0.1, ack_delay=0.15) as link:
            command = link.sender.send_params(P1)
            self.assertTrue(command.done.wait(3.0))
        self.assertEqual(command.status, 'applied')
        self.assertGreaterEqual(len(link.client.chunks), 4)  # sent again with the same SEQ
        self.assertEqual(link.received(), [('PARAMS', P1)])  # applied once
        self.assertEqual(self.faults, [])

    def test_late_ack_of_an_earlier_stop_does_not_confirm_a_later_one(self):
        # The controller became unreachable after the first Stop: late copies of its ACK
        # must not confirm the second Stop
        sender = DataSender()
        sender.bleConnected = True
        first = sender.send_stop()
        self.assertIs(sender.next_command(), first)
        sender.complete(first, 'applied')
        second = sender.send_stop()
        self.assertIs(sender.next_command(), second)
        sender.expect_ack(second)
        sender.on_notify(None, ack(first.seq, ble.TYPE_STOP))
        self.assertIsNone(sender.ack_status())
        sender.on_notify(None, ack(second.seq, ble.TYPE_STOP))
        self.assertEqual(sender.ack_status(), ble.ACK_APPLIED)

    def test_unacknowledged_stop_raises_link_fault(self):
        with self.link(controller=MockController(mute={0x02})) as link:
            command = link.sender.send_stop()
            self.assertTrue(command.done.wait(3.0))
        self.assertEqual(command.status, 'no_ack')
        self.assertEqual(len(self.faults), 1)
        kind, message = self.faults[0]
        self.assertEqual(kind, 'STOP')
        self.assertTrue(message.startswith(STOP_ALERT))
        self.assertEqual(link.sender.link_fault, message)
        # 1 + 3 retransmissions, all with the same SEQ
        self.assertEqual(link.client.chunks, [ble.encode_frame(ble.TYPE_STOP, command.seq)] * 4)
        self.assertEqual(link.received(), [('STOP',)] * 4)  # applied each time, as specified

    def test_rejected_parameters(self):
        with self.link() as link:
            command = link.sender.send_params([math.nan] + P1[1:])
            self.assertTrue(command.done.wait(3.0))
        self.assertEqual(command.status, 'rejected')
        self.assertEqual(len(link.client.chunks), 2)  # not sent again
        self.assertEqual(link.received(), [])
        self.assertEqual(self.faults[0][0], 'PARAMS')
        self.assertIn('rejected by the controller', self.faults[0][1])

    def test_stop_ends_retransmission_of_parameters(self):
        with self.link(controller=MockController(mute={0x01}), ack_timeout=1.0) as link:
            params = link.sender.send_params(P1)
            self.assertTrue(link.client.writing.wait(2.0))
            stop = link.sender.send_stop()
            # The Stop does not wait for the ACK timeout of the parameters
            self.assertTrue(stop.done.wait(0.5))
            self.assertTrue(params.done.wait(3.0))
        self.assertEqual((params.status, stop.status), ('superseded', 'applied'))
        stop_frame = ble.encode_frame(ble.TYPE_STOP, stop.seq)
        self.assertEqual(link.client.chunks[-1], stop_frame)
        self.assertEqual(link.client.chunks.count(stop_frame), 1)
        self.assertEqual(self.faults, [])

    def test_stop_ends_parameters_whose_write_failed(self):
        # The PARAMS write times out while a Stop waits: the PARAMS is not written again
        class StalledParams(MockClient):
            params_writes = 0

            async def write_gatt_char(self, uuid, data, response=True):
                if bytes(data[:4]) == bytes([0xA5, 0x5A, 0x02, 0x01]):
                    self.params_writes += 1
                    self.writing.set()
                    await asyncio.sleep(10.0)  # never completes
                return await super().write_gatt_char(uuid, data, response)

        sender = DataSender()
        sender.write_timeout = 0.3
        self.faults = []
        sender.on_link_fault = lambda kind, message: self.faults.append((kind, message))
        with LinkThread(StalledParams(), sender) as link:
            params = sender.send_params(P1)
            self.assertTrue(link.client.writing.wait(2.0))
            stop = sender.send_stop()
            self.assertTrue(stop.done.wait(3.0))
            self.assertTrue(params.done.wait(3.0))
        self.assertEqual((params.status, stop.status), ('superseded', 'applied'))
        self.assertEqual(link.client.params_writes, 1)
        self.assertEqual(link.received(), [('STOP',)])
        self.assertEqual(self.faults, [])

    def test_ping(self):
        with self.link() as link:
            self.assertTrue(link.sender.check_link())
        self.assertEqual(link.client.controller.applied, [('PING',)])
        with self.link(controller=MockController(mute={0x03})) as link:
            self.assertFalse(link.sender.check_link())
        self.assertIn('PB6', link.sender.link_fault)

    def test_controller_fault(self):
        controller = MockController()
        controller.faulted = True  # a fault has latched the assistance off
        with self.link(controller=controller) as link:
            params = link.sender.send_params(P1)
            self.assertTrue(params.done.wait(3.0))
            self.assertFalse(link.sender.check_link())
            self.assertTrue(link.sender.controller_faulted)
            stop = link.sender.send_stop()
            self.assertTrue(stop.done.wait(3.0))
            # A PING without ACK is a link fault, not a controller fault
            controller.mute.add(0x03)
            self.assertFalse(link.sender.check_link())
            self.assertFalse(link.sender.controller_faulted)
            # The controller was reset
            controller.mute.clear()
            controller.faulted = False
            self.assertTrue(link.sender.check_link())
        self.assertEqual((params.status, stop.status), ('rejected', 'applied'))
        self.assertEqual(link.received(), [('STOP',)])  # no parameters applied while faulted
        self.assertEqual([kind for kind, _ in self.faults], ['PARAMS', 'PING', 'PING'])
        self.assertEqual(self.faults[0][1], ble.CONTROLLER_FAULT)
        self.assertEqual(self.faults[1][1], ble.CONTROLLER_FAULT)
        self.assertIn('No reply from the controller', self.faults[2][1])
        self.assertIsNone(link.sender.link_fault)

    def test_stop_lost_with_the_link_is_sent_after_reconnecting(self):
        # The link drops during the Stop's writes (chunks 0-1 are the PARAMS frame), so
        # the Stop ends without ACK while BLE still counts as connected
        controller = MockController()
        with self.link(controller=controller, drop_at=2) as link:
            params = link.sender.send_params(P1)
            self.assertTrue(params.done.wait(3.0))
            stop = link.sender.send_stop()
            self.assertTrue(stop.done.wait(3.0))
            link.thread.join(2.0)  # the writer returns when the connection is lost
        self.assertEqual(stop.status, 'no_ack')
        self.assertEqual(self.faults[0][0], 'STOP')
        self.assertTrue(self.faults[0][1].startswith(STOP_ALERT))
        self.assertEqual(controller.applied, [('PARAMS', P1)])
        # Reconnected: the Stop is sent first and applied
        with LinkThread(MockClient(controller), link.sender) as relink:
            end = time.monotonic() + 3.0
            while ('STOP',) not in controller.applied and time.monotonic() < end:
                time.sleep(0.005)
            relink.drain()
        self.assertEqual(controller.applied, [('PARAMS', P1), ('STOP',)])
        self.assertFalse(link.sender.stop_unconfirmed)
        self.assertEqual(len(self.faults), 1)

    def test_parameters_applied_after_host_restart(self):
        # The controller stays powered while the optimizer is restarted. Both sessions
        # start at the same SEQ, so the first PARAMS of session 2 repeats the SEQ of the
        # last PARAMS applied in session 1; the start-up PING clears it.
        controller = MockController()
        sent = []
        for first in (10.0, 20.0):
            sender = DataSender()
            sender._seq = itertools.count(0)
            with LinkThread(MockClient(controller), sender) as link:
                self.assertTrue(link.sender.check_link())  # start-up PING
                command = link.sender.send_params([first] + P1[1:])
                self.assertTrue(command.done.wait(3.0))
            self.assertEqual(command.status, 'applied')
            sent.append(command)
        self.assertEqual(sent[0].seq, sent[1].seq)
        self.assertEqual([c for c in controller.applied if c[0] == 'PARAMS'],
                         [('PARAMS', [10.0] + P1[1:]), ('PARAMS', [20.0] + P1[1:])])

    def test_seq_starts_at_a_random_value(self):
        with mock.patch('bleFunctions.random.randrange', return_value=200) as randrange:
            sender = DataSender()
        randrange.assert_called_once_with(256)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual([sender.send_stop().seq, sender.send_ping().seq], [200, 201])

    def test_stop_without_connection(self):
        sender = DataSender()
        faults = []
        sender.on_link_fault = lambda kind, message: faults.append((kind, message))
        with contextlib.redirect_stdout(io.StringIO()):
            command = sender.send_stop()
        self.assertEqual(faults[0][0], 'STOP')
        self.assertTrue(faults[0][1].startswith(STOP_ALERT))
        self.assertFalse(command.done.is_set())  # still queued, sent after reconnecting
        self.assertIs(sender.next_command(), command)

    def test_stop_before_a_queued_ping(self):
        sender = DataSender()
        sender.bleConnected = True
        ping = sender.send_ping()
        stop = sender.send_stop()
        self.assertIs(sender.next_command(), stop)
        self.assertIs(sender.next_command(), ping)


class TestControllerLinkFault(LinkCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = self._tmp.name

    def tearDown(self):
        self._tmp.cleanup()

    def controller(self, link):
        hilo = make_controller(self.tmp)
        hilo.client = link.sender
        link.sender.on_link_fault = hilo.on_link_fault
        return hilo

    def test_unacknowledged_stop_alerts_the_operator(self):
        with self.link(controller=MockController(mute={0x02}), ack_timeout=0.05) as link:
            hilo = self.controller(link)

            def measure(h):
                link.drain()
                h.stop_optimization()
            run_loop(hilo, measure=measure)
            link.drain()
        self.assertTrue(hilo.operator_message.startswith(STOP_ALERT))
        self.assertTrue(hilo.link_fault.startswith(STOP_ALERT))
        m = hilo.generation_record[0]['conditions'][0]['measurements'][0]
        self.assertEqual((m['status'], m['reason']), ('STOPPED', 'stopped by the operator'))

    def test_invalid_condition_waits_for_the_stop_ack(self):
        # "Assistance set to zero" only once the controller acknowledged the Stop
        with self.link(controller=MockController(mute={0x02}), ack_timeout=0.05) as link:
            hilo = self.controller(link)

            def measure(h):
                link.drain()  # the parameters were applied
                h.end_condition('INVALID', 'no valid breath for 30 s')
            run_loop(hilo, measure=measure)
            message = hilo.operator_message
            link.drain()
        self.assertTrue(message.startswith(STOP_ALERT))
        self.assertIn('Condition #1 INVALID: no valid breath for 30 s', message)
        self.assertNotIn('Assistance set to zero', message)

    def test_unacknowledged_parameters_stop_the_conditions(self):
        with self.link(controller=MockController(mute={0x01}), ack_timeout=0.05) as link:
            hilo = self.controller(link)
            run_loop(hilo, measure=lambda h: time.sleep(0.002))
            link.drain()
        m = hilo.generation_record[0]['conditions'][0]['measurements'][0]
        self.assertEqual(m['status'], 'STOPPED')
        self.assertTrue(m['reason'].startswith('link fault: Parameters not applied'))
        self.assertTrue(hilo.operator_message.startswith('Parameters not applied'))
        # The controller applied the parameters but its ACKs were lost; then assistance to zero
        self.assertEqual([c[0] for c in link.received()], ['PARAMS', 'STOP'])
        params_frames = [c for c in link.client.chunks if c[:4] == bytes([0xA5, 0x5A, 0x02, 0x01])]
        self.assertEqual(len(params_frames), 4)  # one condition's frame, 1 + 3 retransmissions
        self.assertEqual(hilo.condCnt, 0)

    def test_fault_during_a_condition_invalidates_it(self):
        controller = MockController()
        with self.link(controller=controller, ack_timeout=0.05) as link:
            hilo = self.controller(link)

            def fault_then_valid(h):
                link.drain()  # the parameters were applied
                controller.faulted = True  # e.g. stale IMU data latched the assistance off
                measure_valid(h)
            run_loop(hilo, measure=fault_then_valid)
            link.drain()
            first = hilo.generation_record[0]['conditions'][0]['measurements']
            self.assertEqual([(m['status'], m['reason'], m['cost']) for m in first],
                             [('INVALID', 'controller fault during the condition', None)])
            self.assertTrue(hilo.operator_message.startswith(
                'Condition #1 INVALID: controller fault during the condition'))
            self.assertIn('Reset the controller; press Start to measure it again', hilo.operator_message)
            self.assertEqual(hilo.met_estimate, [])

            # Start is refused until the controller is reset
            run_loop(hilo)
            link.drain()
            self.assertEqual(hilo.operator_message, ble.CONTROLLER_FAULT)
            self.assertEqual(len(hilo.generation_record[0]['conditions'][0]['measurements']), 1)

            # After the reset, the condition is measured again with the same parameters
            controller.faulted = False

            def measure(h):
                link.drain()
                measure_valid(h)
            run_loop(hilo, measure=measure, crash_after=1)
            link.drain()
        measurements = hilo.generation_record[0]['conditions'][0]['measurements']
        self.assertEqual([m['status'] for m in measurements], ['INVALID', 'VALID'])
        self.assertIsNone(hilo.link_fault)
        # Zero assistance from the fault until the reset, then the same parameters again and
        # the next condition
        kinds = [c[0] for c in link.received()]
        self.assertEqual(kinds[0], 'PARAMS')
        self.assertEqual(set(kinds[1:-2]), {'STOP'})
        self.assertEqual(kinds[-2:], ['PARAMS', 'PARAMS'])
        sent = [c[1] for c in link.received() if c[0] == 'PARAMS']
        self.assertEqual(sent[0], sent[1])

    def test_start_needs_an_acknowledged_ping(self):
        controller = MockController(mute={0x03})
        with self.link(controller=controller, ack_timeout=0.05) as link:
            hilo = self.controller(link)
            run_loop(hilo)
            link.drain()
            self.assertIn('No reply from the controller', hilo.operator_message)
            self.assertNotIn('PARAMS', [c[0] for c in link.received()])
            self.assertEqual(hilo.generation_record, [])
            # The link works again: the next Start runs conditions
            controller.mute.clear()

            def measure(h):
                link.drain()  # a condition lasts minutes: its parameters were applied
                measure_valid(h)
            run_loop(hilo, measure=measure, crash_after=1)
            link.drain()
        self.assertIsNone(hilo.link_fault)
        self.assertEqual([c[0] for c in link.received()][:2], ['STOP', 'PARAMS'])
        self.assertEqual(hilo.generation_record[0]['conditions'][0]['measurements'][0]['status'], 'VALID')


if __name__ == '__main__':
    unittest.main()
