"""CMA-ES flow checks against pycma.

Run from the repository root:
    python -m unittest discover -s Optimizer/tests -t Optimizer
"""
import contextlib
import io
import os
import subprocess
import sys
import tempfile
import types
import unittest
import warnings

import cma
import numpy as np

from tests._harness import cost, make_controller, run_loop
import cma_optimizer

OPTIMIZER_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestWarningsVisible(unittest.TestCase):
    def test_import_does_not_silence_pycma_warnings(self):
        # A clean interpreter, so the filters seen are the ones the module installs.
        code = (
            "import warnings\n"
            "import cma_optimizer\n"
            "from cma.evolution_strategy import InjectionWarning\n"
            "with warnings.catch_warnings(record=True) as w:\n"
            "    warnings.warn('orphanated injected solution', InjectionWarning)\n"
            "    warnings.warn('TPA: apparent inconsistency with mirrored samples, where ...')\n"
            "print(len(w))\n"
        )
        env = {k: v for k, v in os.environ.items() if k != 'PYTHONWARNINGS'}
        out = subprocess.run([sys.executable, '-c', code], cwd=OPTIMIZER_DIR, env=env,
                             capture_output=True, text=True, timeout=60)
        self.assertEqual(out.returncode, 0, out.stderr)
        self.assertEqual(out.stdout.strip().splitlines()[-1], '2')


def make_optimizer(tmp):
    """A CMAOptimizer that writes its files under tmp."""
    import cma_optimizer
    from param_utils import ParameterUtils

    class _Controller:
        genCnt = 1
        param_utils = ParameterUtils(body_weight=70.0)

    opt = cma_optimizer.CMAOptimizer(_Controller(), '0', 8, seed=12345)
    return opt


class TestStepSizeAdaptation(unittest.TestCase):
    def test_tpa(self):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            opt = make_optimizer(tmp)
            opt.initialize_cma_es([-0.4, 0.2, 20.0, 70.0, 45.0, 25.0], [0, 10])
        self.assertIsInstance(opt.es.adapt_sigma, cma.sigma_adaptation.CMAAdaptSigmaTPA)


CMA_DIR = os.path.dirname(cma.__file__)


class TestTpaFlow(unittest.TestCase):
    """The controller's ask/present/tell flow against a direct pycma run."""

    GENERATIONS = 8

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        hilo = make_controller(cls.tmp.name, max_generations=cls.GENERATIONS)
        opt = hilo.optimizer
        cls.asks, cls.tells, cls.sigmas = [], [], []

        ask, tell = opt.ask_new_parameters, opt.tell_results

        def ask_spy(reinject=None):
            X = ask(reinject)
            geno = [opt.es.sent_solutions[x]['geno'] for x in X[:2]]
            cls.asks.append({'X': [x.copy() for x in X], 'reinject': reinject, 'geno01': geno,
                             'mean': opt.es.mean.copy(),
                             'mean_old': getattr(opt.es, 'mean_old', opt.es.mean).copy()})
            return X

        def tell_spy(solutions, costs, **kwargs):
            cls.tells.append(([np.array(x) for x in solutions], list(costs)))
            out = tell(solutions, costs, **kwargs)
            cls.sigmas.append(opt.es.sigma)
            return out

        opt.ask_new_parameters, opt.tell_results = ask_spy, tell_spy
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter('always')
            run_loop(hilo)
        cls.cma_warnings = [str(w.message) for w in caught
                            if w.filename.startswith(CMA_DIR) or 'TPA' in str(w.message)]
        cls.hilo = hilo

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def presented_cost(self, x):
        """Cost of the condition presented for the asked solution x."""
        pu = self.hilo.param_utils
        return cost(pu.scale_params(pu.constrain_params(pu.recover_params(x))))

    def told(self, x):
        """The solution told for the asked solution x: x itself, or the parameters as
        presented when the timing constraint changed them."""
        pu = self.hilo.param_utils
        p = pu.recover_params(x)
        c = pu.constrain_params(np.array(p, dtype=float))
        return x if np.allclose(c, p, rtol=0, atol=1e-9) else pu.scale_params(c)

    def test_runs_all_generations(self):
        self.assertEqual(len(self.tells), self.GENERATIONS)
        self.assertEqual(len(self.asks), self.GENERATIONS + 1)  # + the proposal generation

    def test_mirrored_pair_preserved(self):
        for g, a in enumerate(self.asks[1:], start=2):
            d0, d1 = a['geno01'][0] - a['mean'], a['geno01'][1] - a['mean']
            shift = a['mean'] - a['mean_old']
            np.testing.assert_allclose(d0, -d1, rtol=0, atol=1e-12, err_msg=f'gen {g}')
            cos = d0 @ shift / np.linalg.norm(d0) / np.linalg.norm(shift)
            self.assertAlmostEqual(cos, 1.0, places=9, msg=f'gen {g}')

    def test_previous_best_reinjected(self):
        for g, a in enumerate(self.asks, start=1):
            # pycma rebuilds an injected solution as mean + sigma * y: equal up to rounding
            idx = [k for k, x in enumerate(a['X'])
                   if np.allclose(x, a['reinject'], rtol=0, atol=1e-12)]
            self.assertEqual(idx, [0] if g == 1 else [2], f'gen {g}')
        for g in range(1, self.GENERATIONS):
            X, f = self.tells[g - 1]
            np.testing.assert_array_equal(self.asks[g]['reinject'], X[int(np.argmin(f))])

    def test_tell_order_is_ask_order(self):
        changed = 0
        for g, (X_told, f_told) in enumerate(self.tells, start=1):
            X_asked = self.asks[g - 1]['X']
            self.assertEqual(len(X_told), len(X_asked))
            for k, (xt, xa) in enumerate(zip(X_told, X_asked)):
                np.testing.assert_array_equal(xt, self.told(xa), err_msg=f'gen {g}, ask index {k}')
                self.assertEqual(f_told[k], self.presented_cost(xa), f'gen {g}, ask index {k}')
                changed += self.told(xa) is not xa
        self.assertGreater(changed, 0)  # the run includes conditions the constraint changed

    def test_presentation_is_torque_sorted(self):
        presented = []
        for gen in self.hilo.generation_record:
            p = [self.hilo.param_utils.recover_params(x) for x in gen['presented']]
            keys = [-q[0] + q[1] for q in p]
            self.assertEqual(keys, sorted(keys))
            presented.append(gen['ask_order'])
        self.assertTrue(any(order != sorted(order) for order in presented))

    def test_sigma_equals_direct_pycma_run(self):
        pu = self.hilo.param_utils
        with contextlib.redirect_stdout(io.StringIO()):
            es = cma.CMAEvolutionStrategy(
                pu.scale_params(self.hilo.params_init), 1.5,
                {'bounds': [0, 10], 'AdaptSigma': cma.sigma_adaptation.CMAAdaptSigmaTPA,
                 'CMA_active_injected': 1, 'seed': 12345})
        best = pu.scale_params(self.hilo.params_init)
        for g in range(1, self.GENERATIONS + 1):
            es.inject([es.boundary_handler.inverse(np.array(best))], force=True)
            X = es.ask()
            for xr, xc in zip(X, self.asks[g - 1]['X']):
                np.testing.assert_array_equal(xr, xc, err_msg=f'gen {g}')
            f = [self.presented_cost(x) for x in X]
            told = [self.told(x) for x in X]
            cma_optimizer.CMAOptimizer.tell_results(types.SimpleNamespace(es=es), told, f)
            self.assertEqual(es.sigma, self.sigmas[g - 1], f'gen {g}')
            best = told[int(np.argmin(f))]
        np.testing.assert_array_equal(es.mean, self.hilo.optimizer.es.mean)

    def test_no_pycma_warnings_at_default_verbose(self):
        # pycma drops its warnings only below verbose -2; the run uses the defaults.
        self.assertGreaterEqual(cma.utilities.utils.global_verbosity, -2)
        self.assertGreaterEqual(self.hilo.optimizer.es.opts['verbose'], -2)
        self.assertEqual(self.cma_warnings, [])


if __name__ == '__main__':
    unittest.main()
