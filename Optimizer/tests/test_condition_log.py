"""Condition log and resume checks."""
import os
import tempfile
import unittest

import numpy as np
import pandas as pd

from tests import _stubs  # noqa: F401  (before the optimizer imports)
from tests._harness import MEAN_ATOL, make_controller, run_loop, run_until_told


class TestLogDtypes(unittest.TestCase):
    def test_text_written_into_empty_text_column(self):
        with tempfile.TemporaryDirectory() as tmp:
            hilo = make_controller(tmp)
            pd.DataFrame([{h: (1 if h == 'gen' else None) for h in hilo.headers}]).to_csv(
                hilo.csvFileName, index=False)
            df = hilo.read_log()
            df.at[0, 'reason'] = '1 INVALID: no valid breath for 30 s'
            df.to_csv(hilo.csvFileName, index=False)
            self.assertEqual(hilo.read_log().at[0, 'reason'], '1 INVALID: no valid breath for 30 s')


class TestProposal(unittest.TestCase):
    def test_proposal_rows_and_personalized_params(self):
        with tempfile.TemporaryDirectory() as tmp:
            hilo = make_controller(tmp, max_generations=2)
            self.assertIsNone(hilo.personalized_params())
            run_loop(hilo)

            df = hilo.read_log()
            self.assertEqual(len(df), 27)
            evaluated, proposal = df[df['gen'] <= 2], df[df['gen'] == 3]
            self.assertTrue((evaluated['status'] == 'VALID').all())
            self.assertTrue((proposal['status'] == 'proposal').all())
            self.assertTrue(proposal['cost'].isna().all())
            self.assertEqual(len(proposal), 9)

            record = hilo.generation_record
            self.assertEqual([g['proposal'] for g in record], [False, False, True])
            expected = np.mean([hilo.param_utils.recover_params(x) for x in record[2]['presented']], axis=0)
            np.testing.assert_array_equal(hilo.personalized_params(), expected)
            self.assertEqual(record[2]['personalized_params'], expected.tolist())
            pars = proposal[[f'par{i}' for i in range(6)]].to_numpy().mean(axis=0)
            np.testing.assert_allclose(pars, expected, atol=1e-4)


class TestResume(unittest.TestCase):
    def test_continue_from_the_condition_log(self):
        with tempfile.TemporaryDirectory() as ref_dir, tempfile.TemporaryDirectory() as tmp:
            ref = make_controller(ref_dir, max_generations=2)
            run_loop(ref)

            first = make_controller(tmp, max_generations=2)
            self.assertEqual(run_loop(first, crash_after=13), 13)  # stops in generation 2
            logged = first.read_log()
            resumed = make_controller(tmp, max_generations=2)
            run_loop(resumed)
            log = resumed.read_log()

            # The logged rows are kept; the conditions left in generation 2 were presented
            # with their logged parameters
            pars = [f'par{i}' for i in range(6)]
            pd.testing.assert_frame_equal(log.loc[:17, pars + ['gen', 'sigma', 'ask_idx']],
                                          logged.loc[:17, pars + ['gen', 'sigma', 'ask_idx']])
            presented = [resumed.param_utils.recover_params(x) for x in resumed.candidate_params_list[0][13:18]]
            np.testing.assert_allclose(presented, logged.loc[13:17, pars].to_numpy(dtype=float), atol=1e-9)
            # The step size follows from the order of the costs: as without the interruption
            self.assertEqual(log['sigma'].tolist(), ref.read_log()['sigma'].tolist())
            self.assertEqual(resumed.optimizer.es.sigma, ref.optimizer.es.sigma)
            # The mean follows from the logged (rounded) parameters: close to it
            np.testing.assert_allclose(resumed.optimizer.es.mean, ref.optimizer.es.mean, atol=MEAN_ATOL)
            # Costs: logged to 0.01 W before the interruption; after it, the conditions were
            # presented with the logged (four-decimal) parameters
            np.testing.assert_allclose(resumed.met_estimate[:13], ref.met_estimate[:13], atol=0.005)
            np.testing.assert_allclose(resumed.met_estimate[13:], ref.met_estimate[13:], rtol=1e-3)
            self.assertEqual(len(log), 27)
            self.assertTrue((log['gen'] == 3).sum() == 9 and (log.loc[log['gen'] == 3, 'status'] == 'proposal').all())

    def test_continued_state_at_several_points(self):
        # Interrupted at the start or in the middle of generation 3 or 5: once the continued
        # run has told that generation, its step size and TPA state are the uninterrupted
        # run's, and its mean and covariance differ only through the logged decimals. The runs
        # include conditions the timing constraint changed and samples pycma folded into the
        # bounds.
        for seed in (12345, 5):
            for gen in (3, 5):
                for pos in (0, 4):
                    with self.subTest(seed=seed, gen=gen, pos=pos), tempfile.TemporaryDirectory() as ref_dir, \
                            tempfile.TemporaryDirectory() as tmp:
                        ref = make_controller(ref_dir, cma_seed=seed)
                        run_until_told(ref, gen)
                        run_loop(make_controller(tmp, cma_seed=seed), crash_after=9 * (gen - 1) + pos)
                        resumed = make_controller(tmp, cma_seed=seed)
                        run_until_told(resumed, gen)
                        a, b = resumed.optimizer.es, ref.optimizer.es
                        self.assertEqual(a.countiter, gen)
                        self.assertEqual(a.sigma, b.sigma)
                        self.assertEqual(a.adapt_sigma.s, b.adapt_sigma.s)
                        np.testing.assert_allclose(a.mean, b.mean, atol=MEAN_ATOL)
                        np.testing.assert_allclose(np.array(a.sm.C), np.array(b.sm.C), atol=0.01)

    def test_measurements_from_the_log(self):
        from hiloMetCmaController import HILOMetCmaController
        m = HILOMetCmaController.measurements_from_log(
            3.0, '1 INVALID: no valid breath for 30 s; 2 STOPPED: stopped by the operator')
        self.assertEqual([(x['attempt'], x['status'], x['reason']) for x in m],
                         [(1, 'INVALID', 'no valid breath for 30 s'), (2, 'STOPPED', 'stopped by the operator'),
                          (3, 'VALID', None)])
        m = HILOMetCmaController.measurements_from_log(
            2.0, '1 SKIPPED: skipped by the operator; 2 INVALID: metabolic estimate is not finite; '
                 'excluded from the update')
        self.assertEqual([x['status'] for x in m], ['SKIPPED', 'INVALID'])
        self.assertEqual(m[1]['reason'], 'metabolic estimate is not finite')
        self.assertEqual(HILOMetCmaController.measurements_from_log(float('nan'), ''), [])


if __name__ == '__main__':
    unittest.main()
