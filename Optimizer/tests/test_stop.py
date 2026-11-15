"""Stop in every phase, through the queued single writer.

Most tests press Stop in one phase and check, with a mock BLE client and a mock controller,
that the controller applies the Stop and no parameters after it until the next Start;
received() gives the commands the controller applied. The others check the writer queue,
that the collection flags are released when the loop fails, and that Start is refused while
the loop ends.

Run from the repository root:
    python -m unittest discover -s Optimizer/tests -t Optimizer
"""
import contextlib
import io
import tempfile
import unittest
from unittest import mock

import numpy as np

from tests import _stubs  # noqa: F401  (before the optimizer imports)
from bleFunctions import DataSender
from tests._ble_mock import LinkThread, MockClient
from tests._harness import make_controller, measure_valid, run_loop

P1 = [-10.0, 6.0, 25.0, 70.0, 40.0, 20.0]
P2 = [-12.0, 7.0, 22.0, 71.0, 41.0, 21.0]


def kinds(commands):
    return [c[0] for c in commands]


class TestWriterQueue(unittest.TestCase):
    def test_stop_during_a_write_is_sent(self):
        # A Stop queued while the parameters are being written is sent after them.
        with LinkThread(MockClient(write_delay=0.05)) as link:
            s = link.sender
            s.send_params(P1)
            self.assertTrue(link.client.writing.wait(2.0))
            self.assertEqual(s.in_flight.kind, 'PARAMS')
            s.send_stop()
            link.drain()
            received = link.received()
        self.assertEqual(kinds(received), ['PARAMS', 'STOP'])
        np.testing.assert_allclose(received[0][1], P1, rtol=1e-6)

    def test_stop_drops_queued_parameters(self):
        with LinkThread(MockClient(write_delay=0.05)) as link:
            s = link.sender
            s.send_params(P1)
            self.assertTrue(link.client.writing.wait(2.0))
            queued = s.send_params(P2)
            s.send_stop()
            link.drain()
            received = link.received()
        self.assertEqual(kinds(received), ['PARAMS', 'STOP'])
        self.assertEqual(queued.status, 'superseded')

    def test_stop_before_parameters_and_repeated_stops(self):
        s = DataSender()
        s.bleConnected = True
        s.send_params(P1)
        stop = s.send_stop()
        self.assertIs(s.send_stop(), stop)  # one Stop waiting at a time
        self.assertIs(s.next_command(), stop)
        self.assertIsNone(s.next_command())


class StopCase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = self._tmp.name

    def tearDown(self):
        self._tmp.cleanup()

    def controller(self, link):
        hilo = make_controller(self.tmp)
        hilo.client = link.sender
        return hilo

    def presented(self, hilo, k):
        return hilo.param_utils.weight_params(
            hilo.param_utils.recover_params(hilo.candidate_params_list[0][k]))


class TestStopPhases(StopCase):
    def test_stop_during_collection_while_writing(self):
        with LinkThread(MockClient(write_delay=0.05)) as link:
            hilo = self.controller(link)
            in_flight = []

            def measure(h):
                if not in_flight:
                    link.client.writing.wait(2.0)
                    in_flight.append(link.sender.in_flight and link.sender.in_flight.kind)
                    h.stop_optimization()
            run_loop(hilo, measure=measure)
            link.drain()
            received = link.received()
        self.assertEqual(in_flight, ['PARAMS'])
        self.assertEqual(kinds(received), ['PARAMS', 'STOP'])
        self.assertEqual(hilo.generation_record[0]['conditions'][0]['measurements'][0]['status'], 'STOPPED')

    def test_stop_between_conditions(self):
        # Stop while the first condition is being saved: the next condition's parameters
        # are not sent and the breath collection flags stay cleared.
        with LinkThread(MockClient()) as link:
            hilo = self.controller(link)
            stops = []

            def on_save(h):
                link.drain()  # a condition lasts minutes: its parameters were written
                if not stops:
                    stops.append(True)
                    h.stop_optimization()
            run_loop(hilo, on_save=on_save)
            link.drain()
            received = link.received()
        self.assertEqual(kinds(received), ['PARAMS', 'STOP'])
        np.testing.assert_allclose(received[0][1], self.presented(hilo, 0), rtol=1e-6)
        self.assertEqual(hilo.condCnt, 1)
        self.assertEqual(hilo.generation_record[0]['costs'][1], None)
        self.assertFalse(hilo.dataCollectFlag)
        self.assertFalse(hilo.collecting)

    def test_stop_before_the_first_condition(self):
        # Stop before the first condition: the controller gets the Stop and no parameters,
        # and no condition runs.
        with LinkThread(MockClient()) as link:
            hilo = self.controller(link)
            initialize = hilo.initialize_optimization

            def initialize_then_stop():
                ok = initialize()
                with contextlib.redirect_stdout(io.StringIO()):
                    hilo.stop_optimization()
                return ok
            hilo.initialize_optimization = initialize_then_stop
            run_loop(hilo, measure=measure_valid)
            link.drain()
            received = link.received()
        self.assertEqual(kinds(received), ['STOP'])
        self.assertEqual(hilo.condCnt, 0)
        self.assertFalse(hilo.dataCollectFlag)

    def test_pending_stop_then_start(self):
        # A Stop pressed while nothing runs does not end the next run: after Start the
        # conditions run until the next Stop, which is sent.
        with LinkThread(MockClient()) as link:
            hilo = self.controller(link)
            with contextlib.redirect_stdout(io.StringIO()):
                hilo.stop_optimization()
            self.assertTrue(hilo.stop_requested)

            def measure(h):
                link.drain()  # a condition lasts minutes: its parameters were written
                if h.condCnt < 2:
                    measure_valid(h)
                else:
                    h.stop_optimization()
            run_loop(hilo, measure=measure, start=True)
            link.drain()
            received = link.received()
        self.assertEqual(kinds(received), ['STOP', 'PARAMS', 'PARAMS', 'PARAMS', 'STOP'])
        for k in range(3):
            np.testing.assert_allclose(received[1 + k][1], self.presented(hilo, k), rtol=1e-6)
        statuses = [c['measurements'][0]['status'] for c in hilo.generation_record[0]['conditions'][:3]]
        self.assertEqual(statuses, ['VALID', 'VALID', 'STOPPED'])
        self.assertFalse(hilo.dataCollectFlag)

    def test_collection_released_when_the_loop_fails(self):
        hilo = make_controller(self.tmp)
        run_loop(hilo, crash_after=0)  # an error while the first condition is collected
        self.assertFalse(hilo.dataCollectFlag)
        self.assertFalse(hilo.collecting)

    def test_start_refused_while_the_loop_ends(self):
        hilo = make_controller(self.tmp)
        hilo.cmaThread = mock.Mock(**{'is_alive.return_value': True})
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(hilo.start_optimization())
        self.assertIn('still ending', hilo.operator_message)


if __name__ == '__main__':
    unittest.main()
