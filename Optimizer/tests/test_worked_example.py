"""The worked example (tests/worked_example.py) for 2 generations, and its output files.

Run from the repository root:
    python -m unittest discover -s Optimizer/tests -t Optimizer
"""
import contextlib
import io
import os
import struct
import tempfile
import unittest
from unittest import mock

import cma
import numpy as np

from tests import _stubs  # noqa: F401  (before the optimizer imports)
from hiloMetCmaController import HILOMetCmaController
from metabolics_extract import metabolic_rate_estimation
from tests import worked_example as we

GENERATIONS = 2
SEED = 3
POP = 9  # population size for six parameters
N = GENERATIONS * POP  # conditions evaluated


class TestWorkedExample(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.out = os.path.join(cls._tmp.name, 'out')
        progress = io.StringIO()
        # The recorded breaths of each measurement, as the optimizer holds them when it records it
        cls.breaths = []
        record = HILOMetCmaController.record_condition

        def recording(hilo, generation, pos, result):
            with hilo.data_lock:
                cls.breaths.append(np.array(hilo.met_list, dtype=float).T)  # time, VO2, VCO2, W
            return record(hilo, generation, pos, result)

        with mock.patch.object(HILOMetCmaController, 'record_condition', recording):
            cls.sim = we.run(cls.out, generations=GENERATIONS, seed=SEED, progress=progress, timeout=300.0)
        cls.progress = progress.getvalue().splitlines()
        cls.hilo = cls.sim.hilo
        cls.log = cls.hilo.read_log()
        cls.evaluated = cls.log[cls.log['gen'] <= GENERATIONS]
        cls.record = cls.hilo.generation_record
        cls.costs = [c for g in cls.record[:GENERATIONS] for c in g['costs']]

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def test_output_files(self):
        self.assertTrue(self.sim.completed)
        self.assertEqual(sorted(os.listdir(self.out)), ['optimizer_console.log', 'subject_0_cmaLogFile.csv'])

    def test_every_condition_valid(self):
        self.assertEqual(len(self.evaluated), N)
        self.assertTrue((self.evaluated['status'] == 'VALID').all())
        self.assertTrue((self.evaluated['attempts'] == 1).all())
        self.assertTrue(self.evaluated['reason'].isna().all())
        self.assertEqual(list(self.evaluated['cond'].astype(int)), list(range(1, N + 1)))
        for g in self.record[:GENERATIONS]:
            for c in g['conditions']:
                self.assertEqual([m['status'] for m in c['measurements']], ['VALID'])
                self.assertNotIn('excluded', c)
        self.assertEqual(len(self.progress), N)
        self.assertTrue(all(': VALID, estimate' in line for line in self.progress))
        # Every simulated breath passed the plausibility checks of the metabolics thread
        with open(self.sim.log_path) as f:
            self.assertNotIn('Data excluded', f.read())

    def test_costs_recorded(self):
        self.assertTrue(all(c is not None for c in self.costs))
        self.assertEqual(self.hilo.met_estimate, self.costs)
        np.testing.assert_allclose(self.evaluated['cost'], np.round(self.costs, 2))
        self.assertEqual(len(self.breaths), N)
        k = 0
        for g in self.record[:GENERATIONS]:
            for c, cost in zip(g['conditions'], g['costs']):
                k += 1
                m = c['measurements'][0]
                self.assertEqual(m['cost'], cost)
                self.assertEqual(self.evaluated['fit_mse'].iloc[k - 1], round(m['fit_mse'], 2))
                breaths = self.breaths[k - 1]
                self.assertEqual(len(breaths), m['breaths'])
                # Energy equation: metabolic rate [W] from VO2 and VCO2 [mL/min]
                np.testing.assert_allclose(breaths[:, 3], (16.48 * breaths[:, 1] + 4.48 * breaths[:, 2]) / 60.0,
                                           rtol=1e-12)
                self.assertEqual(breaths[0, 0], 0.0)
                self.assertTrue(120.0 < breaths[-1, 0] < 150.0)
                # The cost is the first-order fit of the recorded breaths
                est, _, mse = metabolic_rate_estimation(breaths[:, 0], breaths[:, 3], tau=42)
                self.assertAlmostEqual(est, cost, places=6)
                self.assertAlmostEqual(mse, m['fit_mse'], places=4)

    def test_parameters_sent_over_ble(self):
        kinds = [command[0] for _, command in self.sim.applied]
        # PING at start-up and at Start, then for each condition its PARAMS and the PING at its end
        self.assertEqual(kinds, ['PING', 'PING'] + ['PARAMS', 'PING'] * N)
        received = [command[1] for _, command in self.sim.applied if command[0] == 'PARAMS']
        presented = [x for g in self.record[:GENERATIONS] for x in g['presented']]
        pu = self.hilo.param_utils
        for got, x in zip(received, presented):
            sent = pu.weight_params(pu.recover_params(x))
            self.assertEqual(struct.pack('<6f', *got), struct.pack('<6f', *sent))

    def test_estimates_follow_the_landscape(self):
        received = [command[1] for _, command in self.sim.applied if command[0] == 'PARAMS']
        for p, cost in zip(received, self.costs):
            q = np.array(p, dtype=float)
            q[:2] /= we.BODY_MASS
            self.assertLess(abs(cost - we.landscape(q)), 50.0)

    def test_cma_es_settings(self):
        es = self.hilo.optimizer.es
        self.assertEqual((es.opts['seed'], es.popsize, es.sigma0), (SEED, POP, 1.5))
        self.assertIsInstance(es.adapt_sigma, cma.sigma_adaptation.CMAAdaptSigmaTPA)
        np.testing.assert_allclose(self.hilo.param_utils.recover_params(es.x0), self.hilo.params_init)

    def test_generation_record(self):
        self.assertEqual([g['gen'] for g in self.record], [1, 2, 3])
        self.assertEqual([g['proposal'] for g in self.record], [False, False, True])
        self.assertEqual(self.hilo.optimizer.es.countiter, GENERATIONS)  # told once per generation
        for g in self.record:
            self.assertEqual(sorted(g['ask_order']), list(range(POP)))
            self.assertEqual((len(g['asked']), len(g['presented'])), (POP, POP))
        proposal = self.record[-1]
        self.assertEqual(proposal['costs'], [None] * POP)
        pu = self.hilo.param_utils
        expected = np.mean([pu.recover_params(x) for x in proposal['presented']], axis=0)
        np.testing.assert_allclose(proposal['personalized_params'], expected)
        np.testing.assert_allclose(self.hilo.personalized_params(), expected)
        # The condition log: the parameters of every row, and the proposal rows
        presented = [pu.recover_params(x) for g in self.record for x in g['presented']]
        np.testing.assert_allclose(self.log[[f'par{i}' for i in range(6)]].to_numpy(), presented, atol=1e-4)
        rows = self.log[self.log['gen'] == GENERATIONS + 1]
        self.assertEqual(len(rows), POP)
        self.assertTrue((rows['status'] == 'proposal').all())
        self.assertTrue(rows['cost'].isna().all())

    def test_summary(self):
        text = we.summary(self.sim)
        self.assertIn(f'Personalized parameters (mean of the {POP} proposed sets)', text)
        for name in we.PARAMETERS:
            self.assertIn(name, text)
        self.assertIn(f'Output files in {self.out}', text)

    def test_command_line(self):
        with tempfile.TemporaryDirectory() as tmp:
            args = we.parse_args(['--out', os.path.join(tmp, 'new')])
            self.assertEqual((args.generations, args.seed), (8, 1))
            for argv in (['--out', tmp, '--generations', '0'], ['--out', tmp, '--seed', '0'],
                         ['--out', self.out],  # self.out: not empty
                         ['--out', '']):
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    we.parse_args(argv)
            a_file = os.path.join(tmp, 'file')
            open(a_file, 'w').close()
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                we.parse_args(['--out', os.path.join(a_file, 'new')])  # below a file


if __name__ == '__main__':
    unittest.main()
