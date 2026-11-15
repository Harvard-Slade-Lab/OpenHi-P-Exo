"""Worked example: the optimizer end to end on simulated inputs.

Run from the Optimizer folder; no Omnia, Tesseract, BLE device or display is needed:

    python -m tests.worked_example --out <new folder> [--generations 8] [--seed 1]

What runs as in a session started with main.py:
  - HILOMetCmaController: Start (start_optimization) and the optimization loop, with the
    condition states, the CMA-ES ask and tell and the condition log;
  - the BLE writer (bleFunctions.DataSender, BleFunctions.send_signal): it sends the
    PARAMS and PING frames to the mock controller of tests/_ble_mock.py, which
    decodes, applies and acknowledges them;
  - the metabolics thread (metabolics_collect.init_met_task, MetabolicsCollector): it reads
    the Omnia display every 0.5 s, applies the plausibility checks and the energy equation,
    and passes each new breath to process_breath.
What is simulated:
  - the Omnia window: the screen capture and the OCR return the time (whole seconds), VO2
    and VCO2 (whole mL/min) of the last breath of a simulated participant;
  - the participant (70 kg), whose metabolic rate depends on the parameters the mock
    controller applied (below);
  - time: every time.sleep of the optimization loop and of the metabolics thread runs on a
    simulated clock (SimClock in tests/_harness.py), so 8 generations of 2-min conditions
    (about 3 h of walking) take seconds. The frames reach the mock controller without
    simulated delay.

Simulated participant
  Steady-state metabolic rate for the applied parameters p (torques divided by the body
  mass, Nm/kg; times in % gait cycle):
      J(p) = 560 W + sum_i WEIGHTS[i] * ((p[i] - OPTIMUM[i]) / r[i])**2
  with r[i] the width of the bounds of parameter i (param_utils.py). J is 611 W at the
  initial parameters (the middle of the bounds) and 560 W at OPTIMUM. Without assistance
  (before the first parameters, after a STOP) it is 650 W. The metabolic rate follows a
  change of J with a first-order response (time constant 42 s). A breath every 2 to 3 s,
  with VO2 = 60 * rate / (16.48 + 4.48 * 0.85) mL/min (the energy equation of
  metabolics_collect.py for a respiratory exchange ratio of 0.85) times (1 + 0.08 n1) and
  VCO2 = VO2 * (0.85 + 0.02 n2), n1 and n2 independent standard normal draws for each breath.

Output in --out: the files a real run writes to outcmaes/subject_<N>/ (here subject 0)
and optimizer_console.log, the console output of the optimizer.
"""
import argparse
import contextlib
import math
import os
import sys
import types
from unittest import mock

import numpy as np

from tests import _stubs  # noqa: F401  (before the optimizer imports)
import metabolics_collect
from bleFunctions import DataSender
from param_utils import ParameterUtils
from tests._ble_mock import LinkThread, MockClient, MockController
from tests._harness import SimClock, make_controller, run_loop

BODY_MASS = 70.0  # [kg]
PARAMETERS = ['extension peak torque (Nm/kg)', 'flexion peak torque (Nm/kg)',
              'extension peak time (% gait)', 'flexion peak time (% gait)',
              'mid time (% gait)', 'extension rise time (% gait)']
J_OPTIMUM = 560.0  # [W]
OPTIMUM = np.array([-0.52, 0.26, 16.0, 70.0, 42.0, 27.0])
WEIGHTS = np.array([300.0, 200.0, 100.0, 60.0, 60.0, 60.0])  # [W]
J_UNASSISTED = 650.0  # [W]
TAU = 42.0  # [s]
RER = 0.85
RER_SD = 0.02
NOISE = 0.08  # SD of VO2 from breath to breath, relative
BREATH_INTERVAL = (2.0, 3.0)  # [s]
OMNIA_START = 300.0  # [s] Omnia time when the optimizer starts
OCR_REGIONS = [((20, 40), (140, 70)), ((20, 90), (140, 120)), ((20, 140), (140, 170))]  # Time, VO2, VCO2

_bounds = ParameterUtils()
RANGES = np.array(_bounds.max_params) - np.array(_bounds.min_params)

OUTPUT_FILES = [
    ('subject_0_cmaLogFile.csv', 'condition log, one row per condition'),
    ('optimizer_console.log', 'console output of the optimizer'),
]


def landscape(p):
    """Steady-state metabolic rate [W] of the participant for parameters p (Nm/kg, % gait)."""
    return J_OPTIMUM + float(np.sum(WEIGHTS * ((np.asarray(p, dtype=float) - OPTIMUM) / RANGES) ** 2))


class Participant:
    """Metabolic rate of the simulated participant: a first-order response (TAU) to the
    steady state of the assistance the mock controller applies."""
    def __init__(self):
        self.t = 0.0
        self.level = J_UNASSISTED  # metabolic rate at time t
        self.target = J_UNASSISTED  # steady state the rate moves to after time t
        self.latest = J_UNASSISTED  # steady state of the last applied assistance
        self.changes = []  # (time, steady state) of applied assistance after time t

    def assist(self, t, params):
        """The controller applied params (as sent: torques in Nm) at time t; None: STOP."""
        if params is None:
            self.latest = J_UNASSISTED
        else:
            p = np.array(params, dtype=float)
            p[:2] /= BODY_MASS
            self.latest = landscape(p)
        self.changes.append((t, self.latest))

    def rate(self, t):
        """Metabolic rate [W] at time t; t does not decrease from call to call."""
        while self.changes and self.changes[0][0] <= t:
            t_change, target = self.changes.pop(0)
            self._evolve(t_change)
            self.target = target
        self._evolve(t)
        return self.level

    def _evolve(self, t):
        self.level = self.target + (self.level - self.target) * math.exp(-(t - self.t) / TAU)
        self.t = t


class Respirometer:
    """The participant's breaths as the Omnia window shows them: Omnia time (whole seconds),
    VO2 and VCO2 (whole mL/min) of the last breath."""
    def __init__(self, participant, rng):
        self.participant = participant
        self.rng = rng
        self.next_breath = 0.0
        self.shown = None
        self.breaths = 0

    def display(self, now):
        while self.next_breath <= now:
            t = self.next_breath
            vo2 = 60.0 * self.participant.rate(t) / (16.48 + 4.48 * RER) * (1.0 + NOISE * self.rng.standard_normal())
            vco2 = vo2 * (RER + RER_SD * self.rng.standard_normal())
            self.shown = (int(OMNIA_START + t), int(round(vo2)), int(round(vco2)))
            self.breaths += 1
            self.next_breath = t + self.rng.uniform(*BREATH_INTERVAL)
        return self.shown


def end_threads(hilo, clock):
    """Stop the metabolics thread and the optimization loop, as closing the GUI does."""
    hilo.isRun = False
    if hilo.metabolicsThread is not None:
        hilo.metabolicsThread.stop()
    clock.release()
    hilo.close()


def run(out_dir, generations=8, seed=1, progress=None, timeout=None):
    """Run the optimization with the simulated participant, writing to out_dir (new or
    empty). progress: stream for one line per condition. timeout: [s] for the optimization
    loop (None: no limit). Returns the run's objects (types.SimpleNamespace)."""
    if os.path.isdir(out_dir) and os.listdir(out_dir):
        raise ValueError(f"{out_dir} is not empty: give a new or empty folder")
    os.makedirs(out_dir, exist_ok=True)
    participant = Participant()
    respirometer = Respirometer(participant, np.random.default_rng(seed))
    controller = MockController()
    hilo = make_controller(out_dir, max_generations=generations, cma_seed=seed, body_weight=BODY_MASS)
    sender = DataSender()
    link = LinkThread(MockClient(controller), sender, quiet=False)  # the writer's lines go to the log
    applied = []  # (simulated time, command) in the order the mock controller applied them

    def deliver():
        """Before the simulated time moves on: the writer sends what is queued, and the
        participant gets the assistance the controller applied."""
        if not sender.idle():
            link.drain()
        for command in controller.applied[len(applied):]:
            applied.append((clock.now, command))
            if command[0] == 'PARAMS':
                participant.assist(clock.now, command[1])
            elif command[0] == 'STOP':
                participant.assist(clock.now, None)

    clock = SimClock(before_advance=deliver)

    def report(h):
        """One line per finished condition (called when the optimizer records it)."""
        result = h.condition_result
        line = f"Condition {h.condCnt + 1:3d}, generation {h.genCnt}: {result['status']}"
        if result['status'] == 'VALID':
            line += (f", estimate {result['cost']:5.1f} W (landscape {participant.latest:5.1f} W), "
                     f"{len(h.met_list[0])} breaths")
        else:
            line += f" ({result['reason']})"
        if progress is not None:
            print(line, file=progress, flush=True)

    log_path = os.path.join(out_dir, 'optimizer_console.log')
    with open(log_path, 'w', encoding='utf-8') as log, contextlib.redirect_stdout(log), \
            contextlib.ExitStack() as stack:
        # The Omnia window: screen capture and OCR replaced by the simulated respirometer
        stack.enter_context(mock.patch.object(metabolics_collect, 'find_window_containing_title', lambda title: 1))
        stack.enter_context(mock.patch.object(metabolics_collect, 'windowGrab', lambda hwnd: None))
        stack.enter_context(mock.patch.object(metabolics_collect, 'extract_data',
                                              lambda image, regions: respirometer.display(clock.now)))
        stack.enter_context(mock.patch.object(metabolics_collect, 'time', mock.Mock(sleep=clock.follower_sleep)))
        stack.enter_context(link)
        stack.callback(end_threads, hilo, clock)
        # As main.py: the controller acknowledges a PING, then the metabolics thread starts
        if not sender.check_link():
            raise RuntimeError(f"the mock controller did not acknowledge a PING: {sender.link_fault}")
        hilo.client = sender
        sender.on_link_fault = hilo.on_link_fault
        boundaries = metabolics_collect.init_met_task(hilo, OCR_REGIONS)
        hilo.metabolics_collector = None  # replaces the stand-in of make_controller
        hilo.start_Metabolics_collector(boundaries)
        clock.follow(hilo.metabolicsThread)
        # Start pressed once; breaths reach process_breath through the metabolics thread
        run_loop(hilo, measure=lambda h: None, on_save=report, start=True, clock=clock,
                 output=log, timeout=timeout)
        deliver()  # the commands the mock controller applied after the last time step
    return types.SimpleNamespace(
        out_dir=out_dir, hilo=hilo, controller=controller, applied=applied, participant=participant,
        respirometer=respirometer, clock=clock, log_path=log_path,
        completed=hilo.personalized_params() is not None)


def summary(sim):
    """The personalized parameters, compared with the initial ones and the optimum of the
    landscape, and the output files."""
    personalized = sim.hilo.personalized_params()
    if personalized is None:
        return (f"The optimization did not finish: see {sim.log_path} and "
                f"{os.path.join(sim.out_dir, 'subject_0_cmaLogFile.csv')}.")
    initial = np.array(sim.hilo.params_init)
    proposals = len(sim.hilo.generation_record[-1]['presented'])
    lines = [f"Personalized parameters (mean of the {proposals} proposed sets):",
             f"  {'':31s} {'initial':>8s} {'personalized':>13s} {'optimum':>8s}"]
    for name, a, b, c in zip(PARAMETERS, initial, personalized, OPTIMUM):
        lines.append(f"  {name:31s} {a:8.3f} {b:13.3f} {c:8.3f}")
    lines += [f"Steady-state metabolic rate of the simulated participant: {landscape(initial):.1f} W "
              f"with the initial parameters, {landscape(personalized):.1f} W with the personalized "
              f"parameters, {J_OPTIMUM:.1f} W at the optimum.",
              "",
              f"Output files in {sim.out_dir}:"]
    lines += [f"  {name:37s} {what}" for name, what in OUTPUT_FILES]
    return '\n'.join(lines)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        prog='python -m tests.worked_example',
        description="Run the optimizer end to end on a simulated participant and a mock controller.")
    parser.add_argument('--out', required=True, help="new or empty folder for the output files")
    parser.add_argument('--generations', type=int, default=8,
                        help="number of generations evaluated (default: 8, as in the study)")
    parser.add_argument('--seed', type=int, default=1,
                        help="seed of CMA-ES and of the simulated breaths, 1 to 2**32 - 1 (default: 1)")
    args = parser.parse_args(argv)
    if args.generations < 1:
        parser.error("--generations must be at least 1")
    if not 0 < args.seed < 2**32:  # pycma takes 0 to mean a time-based seed
        parser.error(f"--seed must be an integer from 1 to 2**32 - 1, got {args.seed}")
    if not args.out:
        parser.error("--out must name a folder")
    if os.path.exists(args.out) and not (os.path.isdir(args.out) and not os.listdir(args.out)):
        parser.error(f"--out {args.out} exists and is not an empty folder")
    parent = os.path.dirname(os.path.abspath(args.out))
    while not os.path.exists(parent) and os.path.dirname(parent) != parent:  # stop at the root
        parent = os.path.dirname(parent)
    if not os.path.isdir(parent):
        parser.error(f"--out {args.out}: {parent} does not exist or is not a folder")
    return args


def main(argv=None):
    args = parse_args(argv)
    out_dir = os.path.abspath(args.out)
    print(f"Worked example: {args.generations} generations, seed {args.seed}, simulated participant, "
          f"mock controller")
    sim = run(out_dir, args.generations, args.seed, progress=sys.stdout)
    print()
    print(summary(sim))
    return 0 if sim.completed else 1


if __name__ == '__main__':
    sys.exit(main())
