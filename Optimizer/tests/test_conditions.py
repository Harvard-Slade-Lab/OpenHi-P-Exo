"""Measured data only and condition states.

Run from the repository root:
    python -m unittest discover -s Optimizer/tests -t Optimizer
"""
import contextlib
import io
import pickle
import tempfile
import unittest
import warnings
from unittest import mock

import cma
import numpy as np
import pandas as pd

from tests import _stubs  # noqa: F401  (before the optimizer imports)
import metabolics_collect
from metabolics_extract import metabolic_rate_estimation
from tests._ble_mock import FakeLink
from tests._harness import (MEAN_ATOL, cost, drop_folded_samples, make_controller, measure_valid, run_loop,
                            run_until_told)

MET = 450.0  # metabolic cost of every fed breath [W]


def feed_breaths(times):
    """A measure() for run_loop that feeds one breath per wait, at the given Omnia times."""
    pending = list(times)

    def measure(hilo):
        if pending:
            hilo.process_breath(pending.pop(0), 1000.0, 800.0, MET)
    return measure


def no_breath(hilo):
    pass


# 5 breaths skipped, then breaths recorded at 0, 30, ..., 120 and 125 s: an estimate at 125 s.
VALID_TIMES = [0, 2, 4, 6, 8] + [10 + 30 * k for k in range(5)] + [135]


def with_client(hilo):
    hilo.client = FakeLink()
    return hilo


class TestOmniaMissing(unittest.TestCase):
    def test_exits_when_window_not_found(self):
        controller = mock.Mock(hwnd=None, metConnected=False)
        with mock.patch.object(metabolics_collect, 'find_window_containing_title', return_value=None), \
                mock.patch.object(metabolics_collect, 'windowGrab') as grab, \
                contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(SystemExit) as cm:
                metabolics_collect.init_met_task(controller, None, 'Omnia')
        self.assertIn("'Omnia' not found", str(cm.exception.code))
        grab.assert_not_called()
        self.assertFalse(controller.metConnected)



class ConditionCase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = self._tmp.name

    def tearDown(self):
        self._tmp.cleanup()

    def record(self, hilo):
        return hilo.generation_record


class TestInvalid(ConditionCase):
    def check_invalid(self, hilo, reason):
        self.assertEqual(hilo.condition_result['status'], 'INVALID')
        self.assertEqual(hilo.condition_result['reason'], reason)
        # Assistance to zero: a Stop is the last command
        self.assertEqual(hilo.client.commands[-1], ('STOP',))
        self.assertIn('INVALID', hilo.operator_message)
        self.assertFalse(hilo.dataCollectFlag)
        self.assertFalse(hilo.collecting)
        row = hilo.read_log().iloc[0]
        self.assertEqual(row['status'], 'INVALID')
        self.assertEqual(row['reason'], f'1 INVALID: {reason}')

    def test_no_breath_for_30_s(self):
        hilo = with_client(make_controller(self.tmp))
        clock = []
        def measure(h):
            clock.append(None)
        run_loop(hilo, measure=measure)
        self.check_invalid(hilo, 'no valid breath for 30 s')
        self.assertAlmostEqual(len(clock) * 0.1, 30.0, delta=0.15)  # ended after 30 s

    def test_breaths_stop_after_recording_started(self):
        hilo = with_client(make_controller(self.tmp))
        run_loop(hilo, measure=feed_breaths([0, 2, 4, 6, 8, 10, 40]))
        self.check_invalid(hilo, 'no valid breath for 30 s')

    def test_fewer_than_5_breaths_in_window(self):
        hilo = with_client(make_controller(self.tmp))
        run_loop(hilo, measure=feed_breaths([0, 2, 4, 6, 8, 10, 40, 70, 175]))
        self.check_invalid(hilo, 'fewer than 5 breaths in the estimation window')

    def test_ocr_thread_stops(self):
        hilo = with_client(make_controller(self.tmp))
        def measure(h):
            h.metabolics_collector.is_alive.return_value = False
        run_loop(hilo, measure=measure)
        self.check_invalid(hilo, 'respirometry OCR stopped')

    def test_stop_sent_before_saving(self):
        # The Stop is sent before the measurement is recorded
        hilo = with_client(make_controller(self.tmp))
        at_save = []
        run_loop(hilo, measure=no_breath, on_save=lambda h: at_save.append(list(h.client.commands)))
        self.assertEqual(len(at_save), 1)
        self.assertEqual(at_save[0][-1], ('STOP',))
        self.check_invalid(hilo, 'no valid breath for 30 s')
        self.assertEqual(hilo.client.commands.count(('STOP',)), 1)

    def test_start_refused_without_respirometry(self):
        hilo = make_controller(self.tmp)
        hilo.metabolics_collector.is_alive.return_value = False
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(hilo.start_optimization())
        self.assertIsNone(hilo.cmaThread)


class TestValid(ConditionCase):
    def test_estimate_from_breaths(self):
        hilo = make_controller(self.tmp)
        run_loop(hilo, measure=feed_breaths(VALID_TIMES), crash_after=1)
        t = np.array(VALID_TIMES[5:]) - VALID_TIMES[5]
        expected, _, mse = metabolic_rate_estimation(t, np.full(len(t), MET), tau=42)
        self.assertEqual(hilo.met_estimate, [expected])
        record = self.record(hilo)
        self.assertEqual(record[0]['costs'][0], expected)
        m = record[0]['conditions'][0]['measurements']
        self.assertEqual([(x['status'], x['breaths']) for x in m], [('VALID', len(t))])
        self.assertEqual(m[0]['fit_mse'], mse)  # the fit residual is kept
        row = hilo.read_log().iloc[0]
        self.assertEqual(row['fit_mse'], round(mse, 2))
        self.assertEqual((row['status'], row['attempts']), ('VALID', 1))
        self.assertTrue(pd.isna(row['reason']))


class TestSkip(ConditionCase):
    def test_skip_outside_collection_is_ignored(self):
        hilo = make_controller(self.tmp)
        with contextlib.redirect_stdout(io.StringIO()):
            hilo.skipCondition()  # before Start
        self.assertIsNone(hilo.condition_result)
        # Skip pressed while each finished condition is being saved is ignored: all stay VALID
        run_loop(hilo, crash_after=10, on_save=lambda h: h.skipCondition())
        record = self.record(hilo)
        gen1 = record[0]
        self.assertEqual(len(record), 2)  # generation 1 told, generation 2 asked
        self.assertEqual(gen1['costs'], [cost(x) for x in gen1['presented']])
        self.assertEqual(hilo.met_estimate[:9], gen1['costs'])
        self.assertEqual([c['measurements'][0]['status'] for c in gen1['conditions']], ['VALID'] * 9)
        df = hilo.read_log()
        self.assertTrue((df['status'].iloc[:10] == 'VALID').all())
        np.testing.assert_allclose(df['cost'].iloc[:9], np.round(gen1['costs'], 2))

    def test_skip_during_collection(self):
        hilo = make_controller(self.tmp)
        def measure(h):
            if h.condCnt == 0 and not h.generation_record[0]['conditions'][0]['measurements']:
                h.skipCondition()
            else:
                measure_valid(h)
        run_loop(hilo, measure=measure, crash_after=2)
        m = self.record(hilo)[0]['conditions'][0]['measurements']
        self.assertEqual(m[0]['status'], 'SKIPPED')
        self.assertEqual(hilo.read_log().iloc[0]['reason'], '1 SKIPPED: skipped by the operator')


class TestStop(ConditionCase):
    def test_stop_during_collection_then_start(self):
        hilo = with_client(make_controller(self.tmp))
        with contextlib.redirect_stdout(io.StringIO()):
            run_loop(hilo, measure=lambda h: h.stop_optimization())
        self.assertTrue(hilo.stop_requested)  # until the next Start
        self.assertEqual(hilo.client.commands[-1], ('STOP',))
        self.assertEqual(hilo.condCnt, 0)
        row = hilo.read_log().iloc[0]
        self.assertEqual((row['status'], row['reason']), ('STOPPED', '1 STOPPED: stopped by the operator'))
        self.assertTrue(np.isnan(row['cost']))
        # Start again: the same condition is measured
        run_loop(hilo, crash_after=1)
        gen1 = self.record(hilo)[0]
        self.assertEqual([m['status'] for m in gen1['conditions'][0]['measurements']], ['STOPPED', 'VALID'])
        self.assertEqual(gen1['costs'][0], cost(gen1['presented'][0]))
        self.assertEqual(hilo.read_log().iloc[0]['attempts'], 2)


class TestResumeBetweenTellAndAsk(ConditionCase):
    def test_resume_after_tell_before_ask(self):
        with tempfile.TemporaryDirectory() as ref_dir:
            ref = make_controller(ref_dir, max_generations=2)
            run_until_told(ref, 1)
        first = make_controller(self.tmp, max_generations=2)
        run_until_told(first, 1)  # told, generation 2 not asked
        self.assertEqual(first.optimizer.es.countiter, 1)
        self.assertEqual(first.read_log()['gen'].max(), 1)
        # Continued from the condition log: generation 1 is told again, then generation 2 asked
        resumed = make_controller(self.tmp, max_generations=2)
        run_until_told(resumed, 1)
        self.assertEqual(resumed.optimizer.es.countiter, 1)
        np.testing.assert_allclose(resumed.met_estimate, ref.met_estimate, atol=0.005)  # 0.01 W in the log
        self.assertEqual(resumed.optimizer.es.sigma, ref.optimizer.es.sigma)
        np.testing.assert_allclose(resumed.optimizer.es.mean, ref.optimizer.es.mean, atol=MEAN_ATOL)


def skip_where(pred):
    """A measure() that presses Skip when pred(generation, ask_idx) is true, else ends VALID."""
    def measure(hilo):
        gen = hilo.generation_record[hilo.genCnt - 1]
        pos = hilo.condCnt - (hilo.genCnt - 1) * hilo.lambdaPopSize
        if pred(hilo.genCnt, gen['ask_order'][pos]):
            hilo.skipCondition()
        else:
            measure_valid(hilo)
    return measure


class TestRemeasureAndExclusion(ConditionCase):
    """An INVALID or SKIPPED condition is measured once more, then excluded."""

    def spy_tells(self, hilo):
        opt, tells = hilo.optimizer, []
        tell = opt.tell_results

        def spy(solutions, costs, keep_sigma=False):
            before = {'state': opt.es.pickle_dumps(), 'sigma': opt.es.sigma,
                      's': getattr(opt.es.adapt_sigma, 's', None)}
            out = tell(solutions, costs, keep_sigma=keep_sigma)
            tells.append(dict(before, solutions=[np.array(x) for x in solutions], costs=list(costs),
                              keep_sigma=keep_sigma, sigma_after=opt.es.sigma, mean=opt.es.mean.copy(),
                              C=np.array(opt.es.sm.C), s_after=getattr(opt.es.adapt_sigma, 's', None)))
            return out
        opt.tell_results = spy
        return tells

    def reference_tell(self, t):
        """The same tell made directly on a copy of the pycma state from before it."""
        es = pickle.loads(t['state'])
        drop_folded_samples(es, t['solutions'])
        with contextlib.redirect_stdout(io.StringIO()), warnings.catch_warnings():
            warnings.simplefilter('ignore')
            es.tell(t['solutions'], np.array(t['costs']))
        return es

    def test_invalid_then_valid_on_remeasure(self):
        hilo = with_client(make_controller(self.tmp))
        run_loop(hilo, measure=no_breath)
        self.assertIn('press Start to measure it again', hilo.operator_message)
        self.assertEqual(hilo.condCnt, 0)
        self.assertEqual(hilo.met_estimate, [])
        first_sent = np.array(hilo.client.params_sent()[0])
        sent = []
        def measure(h):
            sent.append(np.array(h.client.commands[-1][1]))
            measure_valid(h)
        run_loop(hilo, measure=measure, crash_after=1)  # Start: the same parameters again
        np.testing.assert_array_equal(sent[0], hilo.param_utils.weight_params(
            hilo.param_utils.recover_params(hilo.candidate_params_list[0][0])))
        np.testing.assert_array_equal(sent[0], first_sent)
        gen1 = self.record(hilo)[0]
        self.assertEqual([m['status'] for m in gen1['conditions'][0]['measurements']], ['INVALID', 'VALID'])
        self.assertEqual(gen1['costs'][0], cost(gen1['presented'][0]))
        row = hilo.read_log().iloc[0]
        self.assertEqual((row['status'], row['attempts']), ('VALID', 2))
        self.assertEqual(row['reason'], '1 INVALID: no valid breath for 30 s')

    def test_invalid_twice_excluded(self):
        hilo = with_client(make_controller(self.tmp))
        run_loop(hilo, measure=no_breath)
        run_loop(hilo, measure=no_breath)
        self.assertIn('excluded from the update', hilo.operator_message)
        self.assertEqual(hilo.condCnt, 1)
        self.assertTrue(np.isnan(hilo.met_estimate[0]))
        cond = self.record(hilo)[0]['conditions'][0]
        self.assertTrue(cond['excluded'])
        self.assertEqual(hilo.read_log().iloc[0]['reason'], '1 INVALID: no valid breath for 30 s; '
                         '2 INVALID: no valid breath for 30 s; excluded from the update')
        # Resume restores the excluded condition
        resumed = make_controller(self.tmp)
        with contextlib.redirect_stdout(io.StringIO()):
            resumed.initialize_optimization()
        self.assertEqual(resumed.condCnt, 1)
        self.assertTrue(np.isnan(resumed.met_estimate[0]))

    def test_non_pair_member_excluded(self):
        hilo = make_controller(self.tmp, max_generations=2)
        tells = self.spy_tells(hilo)
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter('always')
            run_loop(hilo, measure=skip_where(lambda g, k: g == 2 and k == 5))
        gen2 = self.record(hilo)[1]
        pos = gen2['ask_order'].index(5)
        self.assertTrue(gen2['conditions'][pos]['excluded'])
        self.assertEqual([m['status'] for m in gen2['conditions'][pos]['measurements']], ['SKIPPED', 'SKIPPED'])
        self.assertIsNone(gen2['costs'][pos])
        t = tells[1]
        self.assertFalse(t['keep_sigma'])
        self.assertEqual(len(t['solutions']), 8)
        expected = [np.array(x) for k, x in enumerate(gen2['told']) if k != 5]
        for xt, xe in zip(t['solutions'], expected):
            np.testing.assert_array_equal(xt, xe)
        # Same as pycma told the 8 solutions directly; TPA reads the pair at indices 0 and 1
        ref = self.reference_tell(t)
        self.assertEqual(t['sigma_after'], ref.sigma)
        np.testing.assert_array_equal(t['mean'], ref.mean)
        self.assertNotEqual(t['sigma_after'], t['sigma'])
        self.assertEqual([str(w.message) for w in caught if 'TPA' in str(w.message)], [])

    def test_pair_member_excluded_keeps_sigma(self):
        hilo = make_controller(self.tmp, max_generations=2)
        tells = self.spy_tells(hilo)
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter('always')
            run_loop(hilo, measure=skip_where(lambda g, k: g == 2 and k == 0))
        t = tells[1]
        self.assertTrue(t['keep_sigma'])
        self.assertEqual(len(t['solutions']), 8)
        # Step size and TPA state unchanged; mean and covariance updated as pycma does
        self.assertEqual(t['sigma_after'], t['sigma'])
        self.assertEqual(t['s_after'], t['s'])
        ref = self.reference_tell(t)
        np.testing.assert_array_equal(t['mean'], ref.mean)
        np.testing.assert_array_equal(t['C'], np.array(ref.sm.C))
        self.assertNotEqual(ref.sigma, t['sigma'])  # pycma alone would adapt from a wrong pair
        # The override is removed again: TPA is plain again
        tpa = hilo.optimizer.es.adapt_sigma
        self.assertNotIn('update', vars(tpa))
        self.assertNotIn('check_consistency', vars(tpa))
        self.assertIs(type(tpa), cma.sigma_adaptation.CMAAdaptSigmaTPA)
        self.assertEqual([str(w.message) for w in caught if 'TPA' in str(w.message)], [])
        # The next generation still has a mirrored pair and is told normally
        self.assertEqual(len(tells), 2)
        self.assertTrue(self.record(hilo)[2]['proposal'])

    def test_too_few_conditions_for_an_update(self):
        hilo = make_controller(self.tmp)
        tells = self.spy_tells(hilo)
        run_loop(hilo, measure=skip_where(lambda g, k: k >= 2))  # 7 of 9 excluded
        self.assertEqual(tells, [])
        self.assertIn('needs at least 4', hilo.operator_message)
        self.assertEqual(hilo.optimizer.es.countiter, 0)


if __name__ == '__main__':
    unittest.main()
