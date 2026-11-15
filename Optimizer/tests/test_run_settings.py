"""Run configuration of the study: eight generations of nine conditions, CMA-ES settings, seed."""
import contextlib
import io
import tempfile
import unittest
from unittest import mock

import cma
import numpy as np

from tests import _stubs  # noqa: F401  (before the optimizer imports)
import main
from tests._harness import make_controller, run_loop


class _Stop(Exception):
    pass


def initialized(hilo):
    with contextlib.redirect_stdout(io.StringIO()):
        hilo.optimizer.initialize_cma_es(hilo.params_init, hilo.scaling_range)
    return hilo.optimizer.es


class TestConfiguration(unittest.TestCase):
    def test_main_runs_eight_generations(self):
        seen = {}

        def controller(**kwargs):
            seen.update(kwargs)
            raise _Stop  # before main() connects to the exoskeleton

        answers = iter(['0', '70'])  # subject number, body weight
        with mock.patch.object(main, 'HILOMetCmaController', controller), \
                mock.patch('builtins.input', lambda prompt='': next(answers)), \
                self.assertRaises(_Stop):
            main.main()
        self.assertEqual(seen['max_generations'], 8)
        self.assertNotIn('cma_seed', seen)  # pycma's time-based seed

    def test_controller_default_generations(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(make_controller(tmp).maxNumbGen, 8)

    def test_cma_es_settings(self):
        with tempfile.TemporaryDirectory() as tmp:
            hilo = make_controller(tmp)
            es = initialized(hilo)
            pu = hilo.param_utils
        self.assertEqual((es.sigma0, es.popsize), (1.5, 9))
        self.assertIsInstance(es.adapt_sigma, cma.sigma_adaptation.CMAAdaptSigmaTPA)
        self.assertEqual(np.asarray(es.opts['bounds'], dtype=float).ravel().tolist(), [0.0, 10.0])
        self.assertEqual(list(pu.min_params), [-0.6, 0.1, 10.0, 60.0, 35.0, 15.0])
        self.assertEqual(list(pu.max_params), [-0.2, 0.3, 30.0, 80.0, 55.0, 35.0])
        np.testing.assert_allclose(pu.recover_params(es.x0), [-0.4, 0.2, 20.0, 70.0, 45.0, 25.0])


class TestSeed(unittest.TestCase):
    def test_given_seed_used_by_pycma(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(initialized(make_controller(tmp, cma_seed=777)).opts['seed'], 777)

    def test_time_based_seed_without_one(self):
        with tempfile.TemporaryDirectory() as tmp:
            seed = initialized(make_controller(tmp, cma_seed=None)).opts['seed']
        self.assertGreater(seed, 0)
        self.assertEqual(seed, int(seed))

    def test_continued_run_gets_a_new_seed(self):
        # A run continued from the condition log samples with a new time-based seed; the
        # step size still follows the log.
        with tempfile.TemporaryDirectory() as ref_dir, tempfile.TemporaryDirectory() as tmp:
            ref = make_controller(ref_dir, cma_seed=777, max_generations=2)
            run_loop(ref)
            run_loop(make_controller(tmp, cma_seed=777, max_generations=2), crash_after=4)
            resumed = make_controller(tmp, cma_seed=None, max_generations=2)
            run_loop(resumed)
            self.assertNotEqual(resumed.optimizer.es.opts['seed'], 777)
            # generations 1 and 2; later candidates are sampled with the new seed
            self.assertEqual(resumed.read_log()['sigma'][:18].tolist(), ref.read_log()['sigma'][:18].tolist())


if __name__ == '__main__':
    unittest.main()
