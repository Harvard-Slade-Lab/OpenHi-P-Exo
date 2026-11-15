import os
import re
import numpy as np
import time
from threading import Lock, Thread
import pandas as pd

from cma_optimizer import CMAOptimizer
from metabolics_collect import MetabolicsCollector
from metabolics_extract import metabolic_rate_estimation
from param_utils import ParameterUtils

# Reason of an INVALID condition during which a controller fault latched the assistance off
CONTROLLER_FAULT_REASON = 'controller fault during the condition'

class HILOMetCmaController:
    """
    Handles the CMA-based HILO logic for Metabolics.
    This is the central controller that coordinates all functionality.
    """

    def __init__(self, subject_number, max_generations=8, body_weight = 84, cma_seed = None):
        self.client = None
        self.bleConnected = False

        self.subject_number = str(subject_number)

        self.metConnected = False
        self.hwnd = None
        self.final_boundaries = []
        self.dataCollectFlag = False

        self.isRun = True
        # Set by Stop; no parameters are sent while it is set. Reset at Start.
        self.stop_requested = False
        # Operator message of a frame the controller never acknowledged; no conditions run
        # while it is set. Cleared at Start when the controller acknowledges a PING.
        self.link_fault = None

        self.maxNumbGen = max_generations
        self.cma_seed = cma_seed
        self.number_params = 6
        self.lambdaPopSize = 4 + int(np.floor(3 * np.log(self.number_params)))
        self.totalConditions = self.maxNumbGen * self.lambdaPopSize

        # Initial parameters
        self.genCnt = 1
        self.condCnt = 0
        self.breathCnt = 0

        # Control parameters
        self.timeToEstimate = 120 # [s]        
        self.breathsToSkip = 5 # [counts]
        self.minBreaths = 5 # [counts] recorded breaths needed for an estimate
        self.breathTimeout = 30 # [s] without a valid breath during collection: INVALID
        # An INVALID or SKIPPED condition is measured once more with the same parameters;
        # after maxFailures such measurements it is excluded from the CMA-ES update.
        self.maxFailures = 2

        # The condition being collected ends with a state: VALID, INVALID, STOPPED or
        # SKIPPED (condition_result). Skip is accepted only while collecting is True.
        self.collecting = False
        self.condition_result = None
        self.last_breath_clock = None
        self.operator_message = ''

        self.param_utils = ParameterUtils(body_weight=body_weight)
        self.params_init = ((np.array(self.param_utils.min_params) + np.array(self.param_utils.max_params)) / 2.0).tolist()
        self.scaling_range = self.param_utils.scaling_range.copy()

        # Data for plotting:
        self.candidate_params_list = [[]]
        # Record of every generation while the optimizer runs (a run continues from the
        # condition log, see continue_from_log).
        self.generation_record = []
        self.current_params = []
        self.current_6_params = []
        self.met_list = [[] for _ in range(4)]  # [time, VO2, VCO2, MetW]
        self.met_estimate = []
        self.torque_dots = None
        self.gaitCycle = np.linspace(0, 100, 1000)
        self.torqueDesired = np.zeros_like(self.gaitCycle)

        # Where to save the output files
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.file_path = os.path.join(current_dir, 'outcmaes', f'subject_{self.subject_number}')

        self.csvFileName = os.path.join(self.file_path, f'subject_{self.subject_number}_cmaLogFile.csv')
        
        # ask_idx: index of the condition in pycma's ask() of its generation.
        # status: state of the last measurement of the condition (VALID, INVALID, STOPPED,
        # SKIPPED), or 'proposal' for the generation asked after the last update.
        # reason: the measurements that did not end VALID, with their reasons.
        # attempts: number of measurements of the condition.
        # fit_mse: mean squared residual of the metabolic fit [W^2] of the VALID measurement.
        self.headers = ['gen', 'cond', 'cost', 'sigma'] + [f'par{i}' for i in range(6)] + \
            ['ask_idx', 'status', 'reason', 'attempts', 'fit_mse']
        # Text columns are read as text: an empty column is otherwise read as float and
        # recent pandas refuses to write text into it.
        self.text_columns = ['status', 'reason']

        # Thread references
        self.cmaThread = None
        self.metabolicsThread = None
        self.optimizer = None
        self.metabolics_collector = None

        # For concurrency
        self.data_lock = Lock()
        # Held while checking stop_requested and queueing parameters, and while Stop sets it
        self.command_lock = Lock()

    def start_CMA_thread(self):
        """Initialize the CMA optimization components but don't start yet"""
        if self.cmaThread is None:
            # Only initialize the CMAOptimizer
            self.optimizer = CMAOptimizer(
                controller=self,
                subject_number=str(self.subject_number),
                max_generations=self.maxNumbGen,
                seed=self.cma_seed
            )
            print("\nCMA optimizer initialized. Ready to start.")

    def start_optimization(self):
        """Start the optimization process (called by button click). Returns False when the
        respirometry source is not running."""
        print("[HILOMetCmaController] Starting optimization process...")
        if self.cmaThread is not None and self.cmaThread.is_alive():
            self.operator_message = "The previous run is still ending: press Start again."
            print(f"[HILOMetCmaController] {self.operator_message}")
            return False
        if not self.respirometry_running():
            self.operator_message = "Respirometry (Omnia OCR) is not running: restart the optimizer."
            print(f"[HILOMetCmaController] {self.operator_message}")
            return False
        self.operator_message = ''
        self.isRun = True
        # A Stop pressed while nothing ran must not end this run
        self.stop_requested = False

        # Create and start the optimization thread
        optimization_thread = Thread(
            target=self.cma_optimization_loop,
            daemon=True
        )
        self.cmaThread = optimization_thread
        optimization_thread.start()
        print("CMA optimization thread started.")
        return True

    def respirometry_running(self):
        """True while the thread that reads the breaths from the Omnia window runs."""
        return self.metabolics_collector is not None and self.metabolics_collector.is_alive()

    def stop_optimization(self): 
        print("[HILOMetCmaController] Stopping optimization of current condition...")

        # Set the assistance to zero (the BLE connection remains) and tell the loop to stop.
        # Works in every phase: during a condition, between conditions and when not running.
        with self.command_lock:
            self.stop_requested = True
            self.send_stop()
        # The condition being collected, if any, ends as STOPPED
        self.end_condition('STOPPED', 'stopped by the operator')

        print("[HILOMetCmaController] Current condition aborted. BLE & metabolics preserved. Boundary preserved.")

    def send_stop(self):
        """Set the assistance to zero. The Stop is queued ahead of any parameters.
        Returns the queued Stop (a bleFunctions.Command), or None without a BLE client."""
        if self.client is not None:
            return self.client.send_stop()
        return None

    def on_link_fault(self, kind, message):
        """Called by the BLE link when a frame was never acknowledged: alert the operator
        and stop running conditions."""
        self.link_fault = message
        self.operator_message = message
        self.end_condition('STOPPED', f'link fault: {message}')
        if kind != 'STOP':
            self.send_stop()

    def start_Metabolics_collector(self, boundaries):
        """Start the metabolics collector thread"""
        if self.metabolicsThread is None:
            self.metabolics_collector = MetabolicsCollector(
                controller=self,
                boundaries=boundaries
            )
            self.metabolicsThread = self.metabolics_collector
            self.metabolics_collector.start()
            print("MetabolicsCollector thread started.")

    def end_condition(self, status, reason='', cost=None, fit_mse=None):
        """End the condition being collected with a state. Only the first call for a
        condition takes effect; returns False when no condition is being collected."""
        with self.data_lock:
            if not self.collecting or self.condition_result is not None:
                return False
            self.condition_result = {'status': status, 'reason': reason, 'cost': cost, 'fit_mse': fit_mse}
            self.collecting = False
        self.dataCollectFlag = False
        return True

    def skipCondition(self):
        """Skip the condition being collected (Skip button). Ignored at any other time, e.g.
        while a finished condition is being saved."""
        if self.end_condition('SKIPPED', 'skipped by the operator'):
            print(f"[HILOController] Skipped condition #{self.condCnt + 1}")
        else:
            print("[HILOController] Skip ignored: no condition is being collected.")

    def process_breath(self, t, vo2, vco2, met_cost):
        """
        Process metabolic data for each breath.
        
        Args:
            t (float): Timestamp of the breath
            vo2 (float): VO2 value
            vco2 (float): VCO2 value
            met_cost (float): Calculated metabolic cost
        """
        if not self.collecting:
            return

        if self.breathCnt < self.breathsToSkip:
            # Skip breath phase
            if self.breathCnt == 0:
                self.t_first = t
                print(f"########## Skipping Breath ##########")
            
            self.breathCnt += 1
            self.last_breath_clock = time.monotonic()
            elapsed_time_skip = t - self.t_first
            print(f"Skipping Breath #{self.breathCnt}: Time={elapsed_time_skip:.2f}, VO2={vo2:.2f}, VCO2={vco2:.2f}, Met={met_cost:.2f}")
            
        else:
            # Data collection phase
            if self.breathCnt == self.breathsToSkip:
                # First breath after skipping - reset time reference for data collection
                self.t_first = t
                print(f"########## Recording Breath ##########")
            
            # Calculate elapsed time since data collection started
            elapsed_time = t - self.t_first
            
            # Check if elapsed time is within valid range
            if 0 <= elapsed_time < self.timeToEstimate + 30:
                self.last_breath_clock = time.monotonic()
                with self.data_lock:
                    # Store breath data
                    self.met_list[0].append(elapsed_time)
                    self.met_list[1].append(vo2)
                    self.met_list[2].append(vco2)
                    self.met_list[3].append(met_cost)
                
                print(f"Recording Breath #{self.breathCnt + 1 - self.breathsToSkip}: Time={elapsed_time:.2f}, VO2={vo2:.2f}, VCO2={vco2:.2f}, Met={met_cost:.2f}")
                
                # Check if we have enough data to estimate metabolic rate
                if elapsed_time > self.timeToEstimate and len(self.met_list[0]) >= self.minBreaths:
                    with self.data_lock:
                        t_rec, met_rec = np.array(self.met_list[0]), np.array(self.met_list[3])
                    met_est, _, fit_mse = metabolic_rate_estimation(t_rec, met_rec, tau=42)
                    if np.isfinite(met_est):
                        print(f"Estimated Metabolic Cost: {met_est:.2f} W (fit MSE {fit_mse:.2f} W^2)")
                        self.end_condition('VALID', cost=float(met_est), fit_mse=float(fit_mse))
                    else:
                        self.end_condition('INVALID', 'metabolic estimate is not finite')
                    
                # Update breath counter and previous time
                self.breathCnt += 1
                
            else:
                print(f"Elapsed time ({elapsed_time:.2f}) is out of valid range [0, {self.timeToEstimate + 30}]")
                if elapsed_time >= self.timeToEstimate + 30:
                    # The estimation window ended without an estimate
                    if len(self.met_list[0]) < self.minBreaths:
                        reason = f'fewer than {self.minBreaths} breaths in the estimation window'
                    else:
                        reason = f'no breath between {self.timeToEstimate} and {self.timeToEstimate + 30} s'
                    self.end_condition('INVALID', reason)
                    
    
    def read_log(self):
        """Read the condition log CSV with fixed dtypes for the text columns."""
        return pd.read_csv(self.csvFileName, dtype={c: object for c in self.text_columns})

    def initialize_optimization(self):
        """Start the optimization, or continue it: at Start after a Stop or an INVALID
        condition from where it is, and when the optimizer starts again from the condition
        log."""
        if self.generation_record and self.optimizer.es is not None:
            print(f"\nContinuing from condition #{self.condCnt + 1} in generation #{self.genCnt}")
            return True
        if os.path.exists(self.csvFileName) and not self.read_log()['gen'].dropna().empty:
            print(f"\nFound previous data on subject #{self.subject_number}")
            return self.continue_from_log(self.read_log())
        else:
            print(f"\nNo previous data on subject #{self.subject_number}")
            os.makedirs(self.file_path, exist_ok=True)
            os.chmod(self.file_path, 0o777)

            df = pd.DataFrame(columns=self.headers)
            df.to_csv(self.csvFileName, index=False)
            print(f"\nStart optimization from condition #{self.condCnt + 1} (Gen #{self.genCnt})")
            # Initialize the CMA-ES
            if not self.optimizer.initialize_cma_es(self.params_init, self.scaling_range):
                return False
            # Generate initial candidate parameters
            self.update_params()
            
        return True
    
    def continue_from_log(self, df):
        """Continue an optimization from its condition log, as in the study. A new CMA-ES is
        asked each logged generation again and told its logged costs with the logged
        parameters (as presented, as the optimizer tells them), in ask order; the last
        logged generation is asked only, and its remaining conditions are presented as
        logged. The step size, mean and covariance thus follow the log, up to the four
        decimals of the logged parameters."""
        if not self.optimizer.initialize_cma_es(self.params_init, self.scaling_range):
            return False
        pars = [f'par{i}' for i in range(6)]
        record, logged_sigma = [], []
        for gen, rows in df.groupby('gen', sort=True):
            presented = [self.param_utils.scale_params(row[pars].to_numpy(dtype=float))
                         for _, row in rows.iterrows()]
            ask_order = [int(k) for k in rows['ask_idx']]
            told = [None] * len(ask_order)
            for pos, k in enumerate(ask_order):
                told[k] = [float(v) for v in presented[pos]]
            conditions = []
            for _, row in rows.iterrows():
                reason = row['reason'] if isinstance(row['reason'], str) else ''
                condition = {'measurements': self.measurements_from_log(row['attempts'], reason)}
                if reason.endswith('excluded from the update'):
                    condition['excluded'] = True
                conditions.append(condition)
            record.append({
                'gen': int(gen),
                'proposal': bool((rows['status'] == 'proposal').all()),
                'told': told,
                'ask_order': ask_order,
                'presented': [[float(v) for v in x] for x in presented],
                'costs': [None if pd.isna(c) else float(c) for c in rows['cost']],
                'conditions': conditions,
            })
            logged_sigma.append(float(rows['sigma'].iloc[0]))

        for k, generation in enumerate(record):
            # Re-measured solution as in update_params: the initial parameters, then the best
            # condition of the previous generation
            if k == 0:
                reinject = self.param_utils.scale_params(self.params_init)
            else:
                prev = record[k - 1]
                measured = [pos for pos, c in enumerate(prev['costs']) if c is not None]
                reinject = None
                if measured:
                    best_pos = min(measured, key=lambda pos: prev['costs'][pos])
                    reinject = np.array(prev['told'][prev['ask_order'][best_pos]])
            self.optimizer.ask_new_parameters(reinject)
            if abs(self.optimizer.get_sigma() - logged_sigma[k]) > 0.005 + 1e-9:
                print(f"[HILOMetCmaController] Generation #{generation['gen']}: step size "
                      f"{self.optimizer.get_sigma():.4f}, logged {logged_sigma[k]:.2f}")
            if k < len(record) - 1:
                solutions, f_values, keep_sigma = self.in_ask_order(generation)
                self.optimizer.tell_results(solutions, f_values, keep_sigma=keep_sigma)

        with self.data_lock:
            self.generation_record = record
            # One entry per finished condition: its cost, or NaN when excluded
            self.met_estimate = [c if c is not None else float('nan')
                                 for g in record for c, cond in zip(g['costs'], g['conditions'])
                                 if c is not None or cond.get('excluded')]
            self.candidate_params_list = [[np.array(x) for g in record for x in g['presented']]]
            self.condCnt = len(self.met_estimate)
            self.genCnt = record[-1]['gen']
        print(f"Continuing from condition #{self.condCnt + 1} in generation #{self.genCnt}")
        return True

    @staticmethod
    def measurements_from_log(attempts, reason):
        """The measurements of a condition from its log row: the reason column lists those
        that did not end VALID ('<attempt> <state>: <reason>'); the others ended VALID."""
        listed = {int(m.group(1)): (m.group(2), m.group(3)) for m in re.finditer(
            r'(\d+) (INVALID|SKIPPED|STOPPED): (.*?)'
            r'(?=; \d+ (?:INVALID|SKIPPED|STOPPED): |; excluded from the update$|$)', reason)}
        count = 0 if pd.isna(attempts) else int(attempts)
        return [{'attempt': k, 'status': listed.get(k, ('VALID', None))[0],
                 'reason': listed.get(k, ('VALID', None))[1]} for k in range(1, count + 1)]

    def update_params(self):
        """Generate new parameters for the next generation"""
        # Re-measure the initial parameters (generation 1) or the best condition of the
        # previous generation, as presented. It is injected into pycma, whose ask() returns it
        # after the TPA mirrored pair (ask indices 0 and 1, from generation 2 on).
        reinject = None
        if self.genCnt == 1:
            reinject = self.param_utils.scale_params(self.params_init)
        else:
            prev = self.generation_record[self.genCnt - 2]
            measured = [pos for pos, c in enumerate(prev['costs']) if c is not None]
            if measured:
                best_pos = min(measured, key=lambda pos: prev['costs'][pos])
                reinject = np.array(prev['told'][prev['ask_order'][best_pos]])

                best_params = self.param_utils.recover_params(reinject)
                formatted = [f"{p:.2f}" for p in best_params]
                print(f"\nAdded best params to next generation: {formatted}")

        # Get raw candidate parameters from CMA-ES (ask order)
        candidate_params_list = self.optimizer.ask_new_parameters(reinject)

        if candidate_params_list is None:
            print("Failed to get new parameters from optimizer.")
            return False

        recovered_params_list = [
            self.param_utils.recover_params(candidate_params)
            for candidate_params in candidate_params_list
        ]

        # Present the conditions sorted by torque. ask_order[k] is the ask index of the k-th
        # presented condition; the costs go back to pycma in ask order, with the parameters
        # as presented (after the timing constraint): a condition the constraint did not
        # change is told as ask() returned it, one it changed is told as changed.
        ask_order = sorted(range(len(recovered_params_list)),
                           key=lambda k: -recovered_params_list[k][0] + recovered_params_list[k][1])

        constrained_params_list = [
            self.param_utils.constrain_params(np.array(recovered_params_list[k], dtype=float))
            for k in ask_order
        ]
        scaled_params_list = [
            self.param_utils.scale_params(constrained_params)
            for constrained_params in constrained_params_list
        ]
        told = list(candidate_params_list)
        for pos, k in enumerate(ask_order):
            if np.any(np.abs(constrained_params_list[pos] - recovered_params_list[k]) > 1e-9):
                told[k] = scaled_params_list[pos]
    
        if len(self.candidate_params_list) != 0:
            self.candidate_params_list[0].extend(scaled_params_list)
        else:
            self.candidate_params_list.append(scaled_params_list)

        # The generation asked after the last update is not evaluated: it is the proposal
        # from which the personalized parameters are computed.
        proposal = self.genCnt > self.maxNumbGen or bool(self.optimizer.is_optimization_complete())

        self.generation_record.append({
            'gen': self.genCnt,
            'proposal': proposal,
            'asked': [[float(v) for v in x] for x in candidate_params_list],
            'told': [[float(v) for v in x] for x in told],
            'ask_order': ask_order,
            'presented': [[float(v) for v in x] for x in scaled_params_list],
            'costs': [None] * len(scaled_params_list),
            'conditions': [{'measurements': []} for _ in scaled_params_list],
        })

        # Save new candidate parameters to CSV
        if os.path.exists(self.csvFileName):
            df = self.read_log()
        else:
            df = pd.DataFrame(columns=self.headers)

        new_rows = []
        for params, ask_idx in zip(constrained_params_list, ask_order):
            new_row = {
                'gen': self.genCnt,
                'cond': None,
                'cost': None,
                'sigma': round(self.optimizer.get_sigma(), 2) if self.optimizer.get_sigma() is not None else None,
            }
            for i in range(6):
                param_name = f'par{i}'
                new_row[param_name] = round(params[i], 4)
            new_row['ask_idx'] = ask_idx
            new_row['status'] = 'proposal' if proposal else None
            new_rows.append(new_row)

        new_df = pd.DataFrame(new_rows)
        df = pd.concat([df, new_df], ignore_index=True)
        df.to_csv(self.csvFileName, index=False)
        print("\nSaved data to CSV file")
        
        return True
    
    def personalized_params(self):
        """Mean of the parameter sets of the proposal generation (torque in Nm/kg, times in
        % gait), or None while there is no proposal."""
        if not self.generation_record or not self.generation_record[-1].get('proposal'):
            return None
        sets = [self.param_utils.recover_params(x) for x in self.generation_record[-1]['presented']]
        return np.mean(sets, axis=0)

    def in_ask_order(self, generation):
        """The measured conditions in ask order, as tell() needs them: their parameters as
        told (see update_params) and their costs; excluded conditions are left out. TPA reads
        the costs at told indices 0 and 1, the mirrored pair (ask indices 0 and 1):
        keep_sigma is True when a member of the pair is excluded."""
        costs = [None] * len(generation['told'])
        for pos, ask_idx in enumerate(generation['ask_order']):
            costs[ask_idx] = generation['costs'][pos]
        told = [k for k, c in enumerate(costs) if c is not None]
        keep_sigma = costs[0] is None or costs[1] is None
        return [np.array(generation['told'][k]) for k in told], [costs[k] for k in told], keep_sigma

    def cma_optimization_loop(self):
        """Main optimization loop that coordinates the entire process"""
        try:
            self.run_conditions()
        finally:
            # However the loop ended, no breaths are collected after it
            with self.data_lock:
                self.collecting = False
            self.dataCollectFlag = False

    def run_conditions(self):
        # The controller must acknowledge a PING before conditions run
        if self.client is not None and not self.client.check_link():
            print(f"[HILOMetCmaController] Link check failed: {self.link_fault}")
            return
        self.link_fault = None

        # Initialize optimizer or load previous state
        if not self.initialize_optimization():
            print("Failed to initialize optimization. Exiting.")
            return

        while self.isRun and (self.genCnt <= self.maxNumbGen) and not self.optimizer.is_optimization_complete():
            if self.stop_requested or self.link_fault:
                # Stop between conditions or before the first one: nothing more is sent
                print("[HILOMetCmaController] Stopped before the next condition. Exiting optimization loop.")
                return
            generation = self.generation_record[self.genCnt - 1]
            pos = self.condCnt - (self.genCnt - 1) * self.lambdaPopSize
            if pos >= self.lambdaPopSize:
                # Generation complete: tell CMA-ES the results and ask for new solutions
                if not self.update_cma(generation):
                    return
                continue

            cond_no = self.condCnt + 1
            print(f"\nCondition #{cond_no} (Generation #{self.genCnt})")
            result = self.measure_condition(generation['presented'][pos])
            if result is None:
                break  # the optimizer is closing, or Stop came before the parameters were sent
            if result['status'] == 'VALID':
                result = self.check_controller(result)
            stop = None
            if result['status'] == 'INVALID':
                stop = self.send_stop()  # no usable data: assistance to zero before saving
            excluded = self.record_condition(generation, pos, result)

            if result['status'] == 'STOPPED':
                print(f"[HILOMetCmaController] Condition stopped ({result['reason']}). Exiting optimization loop.")
                return
            if result['status'] == 'INVALID':
                # A Stop was sent above; wait for the operator (Start continues)
                if excluded:
                    then = "it is excluded from the update; press Start to continue"
                else:
                    then = "press Start to measure it again"
                if result['reason'] == CONTROLLER_FAULT_REASON:
                    check = "Reset the controller"
                else:
                    check = "Check the respirometry and the participant"
                # "Assistance set to zero" only once the controller acknowledged the Stop
                if stop is None or self.client.wait(stop):
                    self.operator_message = (f"Condition #{cond_no} INVALID: {result['reason']}. "
                                             f"Assistance set to zero. {check}; {then}.")
                else:
                    alert = self.link_fault or ("Stop not acknowledged: use the control-mode switch "
                                                "or the emergency stop")
                    self.operator_message = (f"{alert}. Condition #{cond_no} INVALID: {result['reason']}. "
                                             f"{check}; {then}.")
                print(f"\n[HILOMetCmaController] {self.operator_message}")
                return
            # A SKIPPED condition is measured again right away, unless it is excluded

        personalized = self.personalized_params()
        if personalized is not None:
            self.generation_record[-1]['personalized_params'] = [float(v) for v in personalized]
            formatted = [f"{p:.3f}" for p in personalized]
            print(f"\nPersonalized parameters (mean of the {len(self.generation_record[-1]['presented'])} proposed sets): {formatted}")

    def check_controller(self, result):
        """A VALID condition counts only if the controller still assisted at its end: a PING
        is answered with a fault status when a fault latched the assistance off during the
        condition (INVALID). Without an ACK the condition is measured again at Start
        (STOPPED)."""
        if self.client is None or self.client.check_link():
            return result
        if self.client.controller_faulted:
            return dict(result, status='INVALID', reason=CONTROLLER_FAULT_REASON, cost=None, fit_mse=None)
        return dict(result, status='STOPPED', reason=f'link fault: {self.link_fault}', cost=None, fit_mse=None)

    def measure_condition(self, solution):
        """Present one condition and collect its breaths until it ends with a state.

        Returns the condition result, or None when the optimizer is closing or a Stop came
        before the parameters were sent.
        """
        # Recover and set parameters
        self.current_6_params = self.param_utils.recover_params(solution)
        self.weighted_6_params = self.param_utils.weight_params(self.current_6_params)
        # Calculate and plot torque
        self.torqueDesired, self.torque_dots = self.param_utils.calculate_torque(self.weighted_6_params, self.gaitCycle)
        
        # The firmware consumes exactly the six optimized parameters, in this order:
        # extensionPeakTorque, flexionPeakTorque, extensionPeakTime, flexionPeakTime,
        # midTime, extensionRiseTime (torque_profile_update_continuous in
        # Controller/Core/Src/torque_profile.c).
        # It derives the profile's closing knot itself, so nothing further is sent.
        self.current_params = np.array(self.weighted_6_params, dtype=float)

        # Send the parameters and start collecting, unless a Stop came first: no parameters
        # are sent after a Stop
        with self.command_lock:
            if self.stop_requested or self.link_fault:
                return None
            if self.client is not None:
                self.client.send_params(self.current_params)
            else:
                formatted_params = [f"{value:.2f}" for value in self.current_params]
                print(f"Updated Parameters: {formatted_params}\n")

            with self.data_lock:
                self.met_list = [[] for _ in range(4)]
                self.breathCnt = 0
                self.condition_result = None
                self.collecting = True
            self.last_breath_clock = time.monotonic()
            self.dataCollectFlag = True

        # Metabolic Data Collection Handling: wait until the condition ends with a state
        while self.condition_result is None:
            time.sleep(0.1)
            if not self.isRun:
                with self.data_lock:
                    self.collecting = False
                self.dataCollectFlag = False
                return None
            if self.stop_requested:
                self.end_condition('STOPPED', 'stopped by the operator')
            elif self.link_fault:
                self.end_condition('STOPPED', f'link fault: {self.link_fault}')
            elif not self.respirometry_running():
                self.end_condition('INVALID', 'respirometry OCR stopped')
            elif time.monotonic() - self.last_breath_clock > self.breathTimeout:
                self.end_condition('INVALID', f'no valid breath for {self.breathTimeout} s')
        return self.condition_result

    def record_condition(self, generation, pos, result):
        """Write a measurement that ended to the condition log and the generation record.
        A VALID condition is done. An INVALID or SKIPPED condition is measured again with
        the same parameters, and excluded from the update after maxFailures such
        measurements. A STOPPED condition is measured again at Start. Returns True when
        the condition is now excluded."""
        condition = generation['conditions'][pos]
        attempt = len(condition['measurements']) + 1
        status, reason = result['status'], result['reason']
        failed = ('INVALID', 'SKIPPED')
        failures = sum(m['status'] in failed for m in condition['measurements']) + (status in failed)
        excluded = status in failed and failures >= self.maxFailures
        cost = result['cost'] if status == 'VALID' else None
        cond_no = self.condCnt + 1

        summary = status + (f' ({reason})' if reason else '')
        if status == 'VALID':
            summary += f', estimate {cost:.2f} W, fit MSE {result["fit_mse"]:.2f} W^2'
        with self.data_lock:
            n_breaths = len(self.met_list[0])

        if cost is not None or excluded:
            with self.data_lock:
                self.met_estimate.append(cost if cost is not None else float('nan'))

        # Log progress in CSV file and in the generation record
        with self.data_lock:
            self.met_list = [[] for _ in range(4)]
            self.breathCnt = 0

            condition['measurements'].append({
                'attempt': attempt,
                'status': status,
                'reason': reason,
                'cost': result['cost'],
                'fit_mse': result['fit_mse'],
                'breaths': n_breaths,
            })
            if cost is not None:
                generation['costs'][pos] = float(cost)
            if excluded:
                condition['excluded'] = True

            reasons = '; '.join(f"{m['attempt']} {m['status']}: {m['reason']}"
                                for m in condition['measurements'] if m['status'] != 'VALID')
            if excluded:
                reasons += '; excluded from the update'
            row = (self.genCnt - 1) * self.lambdaPopSize + pos
            df = self.read_log()
            df.at[row, 'cond'] = cond_no
            df.at[row, 'cost'] = round(cost, 2) if cost is not None else np.nan
            df.at[row, 'status'] = status
            df.at[row, 'reason'] = reasons if reasons else np.nan
            df.at[row, 'attempts'] = attempt
            df.at[row, 'fit_mse'] = round(result['fit_mse'], 2) if result['fit_mse'] is not None else np.nan
            df['cond'] = df['cond'].astype('Int64')
            df['attempts'] = df['attempts'].astype('Int64')
            df.to_csv(self.csvFileName, index=False)

        print(f"\nCondition #{cond_no}, attempt {attempt}: {summary}"
              + ("; excluded from the update" if excluded else ""))
        if cost is not None or excluded:
            self.condCnt += 1
        return excluded

    def update_cma(self, generation):
        """Tell CMA-ES the costs of the complete generation (unless an interrupted run told
        them already) and ask for the next generation. Returns False when too few
        conditions were measured for an update."""
        with self.data_lock:
            if self.optimizer.es.countiter < self.genCnt:
                solutions, f_values, keep_sigma = self.in_ask_order(generation)
                mu = self.optimizer.es.sp.weights.mu
                if len(solutions) < mu:
                    self.operator_message = (f"Generation #{self.genCnt} has {len(solutions)} measured "
                                             f"conditions; CMA-ES needs at least {mu} for an update. "
                                             f"The optimization stops.")
                    print(f"\n[HILOMetCmaController] {self.operator_message}")
                    return False
                self.optimizer.tell_results(solutions, f_values, keep_sigma=keep_sigma)
                print("Submitted solutions and f_values to CMA-ES."
                      + (" Step size kept: a member of the mirrored pair was excluded." if keep_sigma else ""))

            if self.genCnt <= self.maxNumbGen:
                self.genCnt += 1
                self.update_params()
        return True

    def close(self):
        """Clean shutdown of all components"""
        self.isRun = False
        
        if self.cmaThread is not None:
            self.cmaThread.join(timeout=1.0)
        if self.metabolicsThread is not None:
            if hasattr(self.metabolicsThread, 'stop') and callable(getattr(self.metabolicsThread, 'stop')):
                self.metabolicsThread.stop()
            self.metabolicsThread.join(timeout=1.0)
        print("HILOController is closed.")