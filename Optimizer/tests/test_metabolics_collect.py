"""The metabolics thread: plausibility checks and energy equation on scripted Omnia readings.

Run from the repository root:
    python -m unittest discover -s Optimizer/tests -t Optimizer
"""
import contextlib
import io
import types
import unittest
from unittest import mock

from tests import _stubs  # noqa: F401  (before the optimizer imports)
import metabolics_collect


class TestMetabolicsThread(unittest.TestCase):
    def test_checks_and_energy_equation(self):
        readings = [(300, 1500, 1300),  # first breath
                    (303, 50, 1300),    # VO2 below 100 mL/min: discarded
                    (306, 1600, 1350),
                    (306, 1600, 1350),  # same time: not a new breath
                    (345, 1500, 1300),  # more than 30 s after the last breath: discarded
                    (309, 1700, 1400)]
        pending = list(readings)
        breaths = []
        controller = types.SimpleNamespace(isRun=True, dataCollectFlag=False, hwnd=1,
                                           process_breath=lambda *b: breaths.append(b))
        collector = metabolics_collect.MetabolicsCollector(controller, boundaries=None)

        def sleep(_):
            controller.dataCollectFlag = True  # collecting from the second pass on
            collector.is_running = bool(pending)

        out = io.StringIO()
        with mock.patch.object(metabolics_collect, 'windowGrab', lambda hwnd: None), \
                mock.patch.object(metabolics_collect, 'extract_data', lambda image, b: pending.pop(0)), \
                mock.patch.object(metabolics_collect, 'time', mock.Mock(sleep=sleep)), \
                contextlib.redirect_stdout(out):
            collector.stream_metabolics()
        self.assertEqual([b[:3] for b in breaths], [(300, 1500, 1300), (306, 1600, 1350), (309, 1700, 1400)])
        for t, vo2, vco2, met in breaths:
            self.assertAlmostEqual(met, (16.48 * vo2 + 4.48 * vco2) / 60.0, places=9)
        self.assertEqual(out.getvalue().count('Data excluded'), 2)


if __name__ == '__main__':
    unittest.main()
