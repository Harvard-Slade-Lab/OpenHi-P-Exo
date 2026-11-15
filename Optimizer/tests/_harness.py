"""Runs the controller's optimization loop without hardware, for the tests."""
import contextlib
import io
import os
import threading
from unittest import mock

import numpy as np

from tests import _stubs  # noqa: F401  (before the optimizer imports)
from hiloMetCmaController import HILOMetCmaController

# Optimum of the test cost on the [0, 10] scale, near the bounds to exercise the bound transform.
OPTIMUM = np.array([9.8, 9.9, 3.0, 5.0, 0.02, 7.0])


def cost(scaled_params):
    return 300.0 + 20.0 * float(np.sum((np.asarray(scaled_params) - OPTIMUM) ** 2))


class Crash(Exception):
    pass


class SimClock:
    """Simulated time for the optimizer's time module (run_loop).

    advance(dt) moves the time forward; run_loop calls it for every time.sleep of the loop,
    after before_advance() (if given). A second thread, the follower, can sleep on the same
    clock with follower_sleep(): advance() then returns only when the follower has run
    everything that was due up to the new time, so the two threads take turns and a run
    does not depend on thread scheduling. release() makes the follower's sleeps return at
    once, so that it can end.
    """
    EPS = 1e-9  # [s] tolerance of the wake-up time for the summed time steps

    def __init__(self, before_advance=None):
        self.now = 0.0
        self.before_advance = before_advance
        self._cv = threading.Condition()
        self._follower = None
        self._wake = None  # wake-up time of the sleeping follower; None while it runs
        self._released = False

    def monotonic(self):
        return self.now

    def advance(self, dt):
        if self.before_advance is not None:
            self.before_advance()
        with self._cv:
            self.now += dt
            self._cv.notify_all()
            while self._follower_due():
                self._cv.wait(0.05)

    def follow(self, thread):
        """From now on advance() waits for thread, which sleeps with follower_sleep()."""
        with self._cv:
            self._follower = thread

    def follower_sleep(self, dt):
        with self._cv:
            self._wake = self.now + dt
            self._cv.notify_all()
            while not self._released and self.now < self._wake - self.EPS:
                self._cv.wait()
            self._wake = None

    def release(self):
        with self._cv:
            self._released = True
            self._cv.notify_all()

    def _follower_due(self):
        f = self._follower
        return (f is not None and not self._released and f.is_alive()
                and (self._wake is None or self.now >= self._wake - self.EPS))


def make_controller(tmp, **kwargs):
    """A controller whose output files are under tmp."""
    kwargs.setdefault('body_weight', 70.0)
    kwargs.setdefault('cma_seed', 12345)
    hilo = HILOMetCmaController(subject_number='0', **kwargs)
    with contextlib.redirect_stdout(io.StringIO()):
        hilo.start_CMA_thread()
    hilo.file_path = tmp
    hilo.csvFileName = os.path.join(tmp, 'subject_0_cmaLogFile.csv')
    hilo.metabolics_collector = mock.Mock(**{'is_alive.return_value': True})  # respirometry running
    return hilo


def measure_valid(hilo):
    """Ends the condition being collected as VALID with cost() of its presented parameters."""
    hilo.end_condition('VALID', cost=cost(hilo.candidate_params_list[0][hilo.condCnt]), fit_mse=0.0)


def run_loop(hilo, crash_after=None, measure=measure_valid, on_save=None, start=False,
             clock=None, output=None, timeout=60.0):
    """Run cma_optimization_loop on a simulated clock. While a condition is collected,
    each wait calls measure(hilo); measure_valid ends it with cost().

    crash_after: raise Crash when the condition after this many measurements is due.
    on_save: called when a measurement that ended (not STOPPED) is about to be recorded.
    start: run the loop through start_optimization (the Start button) in its thread.
    clock: the SimClock to run on (default: a new one).
    output: stream for the optimizer's console output (default: discarded).
    timeout: [s] wait for the loop started with start=True (None: no limit).
    Returns the number of measurements that measure() ended.
    """
    measured = [0]
    if clock is None:
        clock = SimClock()

    def fake_sleep(dt):
        clock.advance(dt)
        if hilo.collecting and hilo.condition_result is None:
            if crash_after is not None and measured[0] >= crash_after:
                raise Crash()
            measure(hilo)
            if hilo.condition_result is not None:
                measured[0] += 1

    record = type(hilo).record_condition

    def record_condition(generation, pos, result):
        if on_save is not None and result['status'] != 'STOPPED':
            on_save(hilo)
        return record(hilo, generation, pos, result)

    fake_time = mock.Mock(sleep=fake_sleep, monotonic=clock.monotonic)
    with mock.patch('hiloMetCmaController.time', fake_time), \
            mock.patch.object(hilo, 'record_condition', record_condition), \
            contextlib.redirect_stdout(output if output is not None else io.StringIO()):
        if start:
            if hilo.start_optimization():
                hilo.cmaThread.join(timeout=timeout)
        else:
            hilo.stop_requested = False  # as start_optimization does
            try:
                hilo.cma_optimization_loop()
            except Crash:
                pass
    return measured[0]


# A run continued from the condition log tells CMA-ES the logged parameters (four decimals);
# its step size is the same as without the interruption and its mean differs by less than
# this on the [0, 10] scale of CMA-ES.
MEAN_ATOL = 0.02


def run_until_told(hilo, generations, crash_after=None):
    """run_loop until CMA-ES has been told the given number of generations; the next ask()
    raises Crash, so that a continued run can be compared where it has caught up."""
    ask = hilo.optimizer.ask_new_parameters

    def ask_or_crash(reinject=None):
        if hilo.optimizer.es is not None and hilo.optimizer.es.countiter >= generations:
            raise Crash()
        return ask(reinject)
    hilo.optimizer.ask_new_parameters = ask_or_crash
    return run_loop(hilo, crash_after=crash_after)


def drop_folded_samples(es, solutions):
    """For a reference tell made directly on pycma: forget pycma's stored sample of each
    solution it folded into the bounds, so that it updates from the solution as presented,
    as the optimizer's tell_results does."""
    for x in solutions:
        stored = es.sent_solutions.get(x)
        if stored is not None and np.max(np.abs(
                stored['geno'] - es.boundary_handler.inverse(np.array(x, dtype=float)))) > 1e-9:
            es.sent_solutions.pop(x)
