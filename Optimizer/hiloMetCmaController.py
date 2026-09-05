import os
import numpy as np
import time
from threading import Lock, Thread
import pandas as pd
from datetime import datetime

from cma_optimizer import CMAOptimizer
from metabolics_collect import MetabolicsCollector
from metabolics_extract import metabolic_rate_estimation
from param_utils import ParameterUtils

class HILOMetCmaController:
    """
    Handles the CMA-based HILO logic for Metabolics.
    This is the central controller that coordinates all functionality.
    """

    def __init__(self, subject_number, max_generations=10, frame_path="frames", body_weight = 84, cma_seed = 0):
        self.client = None
        self.bleConnected = False

        self.subject_number = str(subject_number)

        self.metConnected = False
        self.hwnd = None
        self.final_boundaries = []
        self.dataCollectFlag = False

        self.simulationThread = None

        self.isRun = True
        self.sendFlag = False
        self.stop_requested = False

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

        self.param_utils = ParameterUtils(body_weight=body_weight)
        self.params_init = ((np.array(self.param_utils.min_params) + np.array(self.param_utils.max_params)) / 2.0).tolist()
        self.scaling_range = self.param_utils.scaling_range.copy()

        # Data for plotting:
        self.candidate_params_list = [[]]
        self.current_params = []
        self.current_6_params = []
        self.met_list = [[] for _ in range(4)]  # [time, VO2, VCO2, MetW]
        self.met_estimate = []
        self.torque_dots = None
        self.gaitCycle = np.linspace(0, 100, 1000)
        self.torqueDesired = np.zeros_like(self.gaitCycle)

        # Where to save frames or CSV
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.file_path = os.path.join(current_dir, 'outcmaes', f'subject_{self.subject_number}')
        self.frame_path = os.path.join(self.file_path, 'frames')

        self.csvFileName = os.path.join(self.file_path, f'subject_{self.subject_number}_cmaLogFile.csv')
        
        self.headers = ['time', 'gen', 'cond', 'cost', 'sigma'] + [f'par{i}' for i in range(6)]

        # Thread references
        self.cmaThread = None
        self.metabolicsThread = None
        self.optimizer = None
        self.metabolics_collector = None

        # For concurrency
        self.data_lock = Lock()
        self.frameSaveFlag = False

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

    def start_simulation_thread(self):
        """Start the metabolics simulation thread when metConnected is False"""
        if self.simulationThread is None:
            def run_met_simulation():
                """
                Simulate breath data and offload processing to buffer.
                """
                print("[Simulated Metabolics] Starting thread-based simulation.")
                while True:
                    try:
                        if self.dataCollectFlag:
                            t = time.time()
                            vo2 = 1000 + 50 * np.random.randn()
                            vco2 = 800 + 40 * np.random.randn()
                            met_cost = (16.48 * vo2 + 4.48 * vco2) / 60.0
                            self.process_breath(t, vo2, vco2, met_cost)
                            time.sleep(0.2)  # 1Hz sampling during active collection
                        else:
                            time.sleep(0.1)  # idle polling interval (reduce CPU usage)
                    except Exception as e:
                        print(f"[run_met_simulation] Exception: {e}")
                        time.sleep(0.5)
            
            # Start the simulation thread
            self.simulationThread = Thread(target=run_met_simulation, daemon=True)
            self.simulationThread.start()
            print("Metabolics simulation thread started.")
            
    def start_optimization(self):
        """Start the optimization process (called by button click)"""            
        print("[HILOMetCmaController] Starting optimization process...")
        self.isRun = True
        
        if not self.metConnected:
            self.start_simulation_thread()
        # Create and start the optimization thread
        optimization_thread = Thread(
            target=self.cma_optimization_loop,
            daemon=True
        )
        self.cmaThread = optimization_thread
        optimization_thread.start()
        print("CMA optimization thread started.")

    def stop_optimization(self): 
        print("[HILOMetCmaController] Stopping optimization of current condition...")

        # Tell loops to stop feeding/collecting this condition
        self.stop_requested = True
        self.dataCollectFlag = False   # simulation/metabolics loop will idle

        # Clear only per-condition buffers
        self.breathCnt = 0
        self.met_list = [[] for _ in range(4)]  # [time, VO2, VCO2, MetW]
        self.t_prev = None

        # Optionally stop parameter streaming over BLE (connection remains)
        if self.client is not None:
            self.client.sendFlag = False
        
        # Trigger parameter update in the plot
        if self.client is not None:
            zeroTorque_params = list(self.current_params)
            for i in range(2):
                zeroTorque_params[i] = 0
            self.client.current_params = zeroTorque_params

            self.client.sendFlag = True

        print("[HILOMetCmaController] Current condition aborted. BLE & metabolics preserved. Boundary preserved.")

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

    def skipCondition(self):
        with self.data_lock:
            self.met_estimate.append(1000.0)
            file_name = f'condition_{self.condCnt + 1}_metabolics.csv'
            file_path = os.path.join(self.frame_path, file_name)
            met_array = np.array([self.met_list[i] for i in range(4)]).T
            np.savetxt(
                file_path,
                met_array,
                delimiter=",",
                fmt="%s",
                header="Time (s), VO2 (mL/min), VCO2 (mL/min), Metabolic (W)",
                comments=''
            )
            print(f"[HILOController] Condition #{self.condCnt + 1} was skipped with cost=1000.")
            
            # Clear the metabolics lists to ensure plots are reset
            self.met_list = [[] for _ in range(4)]
            self.breathCnt = 0
            self.t_prev = None  # Reset breath tracking
            
        # Set dataCollectFlag to False outside the lock to avoid potential deadlocks
        self.dataCollectFlag = False
        print(f"[HILOController] Skipped condition #{self.condCnt + 1}")

    def process_breath(self, t, vo2, vco2, met_cost):
        """
        Process metabolic data for each breath.
        
        Args:
            t (float): Timestamp of the breath
            vo2 (float): VO2 value
            vco2 (float): VCO2 value
            met_cost (float): Calculated metabolic cost
        """
        if self.breathCnt < self.breathsToSkip:
            # Skip breath phase
            if self.breathCnt == 0:
                self.t_first = t
                print(f"########## Skipping Breath ##########")
            
            self.breathCnt += 1
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
                with self.data_lock:
                    # Store breath data
                    self.met_list[0].append(elapsed_time)
                    self.met_list[1].append(vo2)
                    self.met_list[2].append(vco2)
                    self.met_list[3].append(met_cost)
                
                print(f"Recording Breath #{self.breathCnt + 1 - self.breathsToSkip}: Time={elapsed_time:.2f}, VO2={vo2:.2f}, VCO2={vco2:.2f}, Met={met_cost:.2f}")
                
                # Check if we have enough data to estimate metabolic rate
                if elapsed_time > self.timeToEstimate and len(self.met_list[0]) >= 5:
                    with self.data_lock:
                        met_est, _, _ = metabolic_rate_estimation(
                            np.array(self.met_list[0]), 
                            np.array(self.met_list[3]),
                            tau=42
                        )
                        
                        self.met_estimate.append(met_est)
                        print(f"Estimated Metabolic Cost: {met_est:.2f} W")
                    
                    # Save metabolics data
                    file_name = f'condition_{self.condCnt + 1}_metabolics.csv'
                    file_path = os.path.join(self.frame_path, file_name)
                    
                    with self.data_lock:
                        met_array = np.array([self.met_list[i] for i in range(4)]).T
                        np.savetxt(
                            file_path,
                            met_array,
                            delimiter=",",
                            fmt="%s",
                            header="Time (s), VO2 (mL/min), VCO2 (mL/min), Metabolic (W)",
                            comments=''
                        )
                        print(f"Saved data for {len(self.met_list[0])} breaths.")
                    
                    # Signal that data collection is complete
                    self.dataCollectFlag = False
                    
                # Update breath counter and previous time
                self.breathCnt += 1
                
            else:
                print(f"Elapsed time ({elapsed_time:.2f}) is out of valid range [0, {self.timeToEstimate + 30}]")
                    
    
    def initialize_optimization(self):
        """Initialize or resume optimization from stored state"""
        if os.path.exists(self.csvFileName):
            print(f"\nFound previous data on subject #{self.subject_number}")
            df = pd.read_csv(self.csvFileName)

            if not df[self.headers[1]].dropna().empty:
                with self.data_lock:
                    # Extend met_estimate from CSV
                    self.met_estimate = df['cost'].dropna().astype(float).tolist()

                    # Extend candidate_params_list from CSV
                    all_scaled_values = []
                    for _, row in df.iterrows():
                        param_values = [row[f'par{i}'] for i in range(6)]
                        scaled_values = self.param_utils.scale_params(param_values)
                        all_scaled_values.append(scaled_values)

                    self.candidate_params_list = [all_scaled_values]

                    self.condCnt = int(len(df[self.headers[3]].dropna()))
                    self.genCnt = int(df[self.headers[1]].dropna().iloc[-1])

                    print(f"Resuming from condition #{self.condCnt + 1} in generation #{self.genCnt}")

                    # Load CMA-ES state
                    self.optimizer.load_cma_state()
        else:
            print(f"\nNo previous data on subject #{self.subject_number}")
            os.makedirs(self.file_path, exist_ok=True)
            os.chmod(self.file_path, 0o777)

            os.makedirs(self.frame_path, exist_ok=True)
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
    
    def update_params(self):
        """Generate new parameters for the next generation"""
        # Get raw candidate parameters from CMA-ES
        candidate_params_list = self.optimizer.ask_new_parameters()
        
        if candidate_params_list is None:
            print("Failed to get new parameters from optimizer.")
            return False
        
        # Inject best solution from previous generation as the first candidate
        if self.genCnt == 1:
            # Use initial params for generation 1
            candidate_params_list[0] = self.param_utils.scale_params(self.params_init)
        else:
            # Find best params from the previous generation
            met_estimate_prev = self.met_estimate[-self.lambdaPopSize:]
            if met_estimate_prev:  # Check if we have previous estimates
                best_idx = met_estimate_prev.index(min(met_estimate_prev))

                candidate_params_list_prev = self.candidate_params_list[0][-self.lambdaPopSize:]
                candidate_params_list[0] = candidate_params_list_prev[best_idx]

                best_params = self.param_utils.recover_params(candidate_params_list[0])
                formatted = [f"{p:.2f}" for p in best_params]
                print(f"\nAdded best params to next generation: {formatted}")


        recovered_params_list = [
            self.param_utils.recover_params(candidate_params)
            for candidate_params in candidate_params_list
        ]

        recovered_params_list = sorted(recovered_params_list, key=lambda params: -params[0] + params[1])

        constrained_params_list = [
            self.param_utils.constrain_params(recovered_params)
            for recovered_params in recovered_params_list
        ]
        scaled_params_list = [
            self.param_utils.scale_params(constrained_params)
            for constrained_params in constrained_params_list
        ]
    
        if len(self.candidate_params_list) != 0:
            self.candidate_params_list[0].extend(scaled_params_list)
        else:
            self.candidate_params_list.append(scaled_params_list)

        # Save new candidate parameters to CSV
        if os.path.exists(self.csvFileName):
            df = pd.read_csv(self.csvFileName)
        else:
            df = pd.DataFrame(columns=self.headers)

        new_rows = []
        for params in constrained_params_list:
            new_row = {
                'time': None,
                'gen': self.genCnt,
                'cond': None,
                'cost': None,
                'sigma': round(self.optimizer.get_sigma(), 2) if self.optimizer.get_sigma() is not None else None,
            }
            for i in range(6):
                param_name = f'par{i}'
                new_row[param_name] = round(params[i], 4)
            new_rows.append(new_row)

        new_df = pd.DataFrame(new_rows)
        df = pd.concat([df, new_df], ignore_index=True)
        df.to_csv(self.csvFileName, index=False)
        print("\nSaved data to CSV file")
        
        return True
    
    def cma_optimization_loop(self):
        """Main optimization loop that coordinates the entire process"""
        # Initialize optimizer or load previous state
        if not self.initialize_optimization():
            print("Failed to initialize optimization. Exiting.")
            return

        while self.isRun and (self.genCnt <= self.maxNumbGen) and not self.optimizer.is_optimization_complete():
            for _ in range(self.lambdaPopSize - (int(self.condCnt) % self.lambdaPopSize)):
                if not self.isRun:
                    break

                print(f"\nCondition #{self.condCnt + 1} (Generation #{self.genCnt})")

                with self.data_lock:
                    self.met_list = [[] for _ in range(4)]
                    self.breathCnt = 0
                    solution = self.candidate_params_list[0][len(self.met_estimate)]

                # Recover and set parameters
                self.current_6_params = self.param_utils.recover_params(solution)
                self.weighted_6_params = self.param_utils.weight_params(self.current_6_params)
                # Calculate and plot torque
                self.torqueDesired, self.torque_dots = self.param_utils.calculate_torque(self.weighted_6_params, self.gaitCycle)
                
                # The firmware consumes exactly the six optimized parameters, in this order:
                # extensionPeakTorque, flexionPeakTorque, extensionPeakTime, flexionPeakTime,
                # midTime, extensionRiseTime (Controller/Core/Src/torque_profile.c:61-66).
                # It derives the profile's closing knot itself, so nothing further is sent.
                self.current_params = np.array(self.weighted_6_params, dtype=float)

                # Trigger parameter update in the plot
                if self.client is not None:
                    self.client.current_params = self.current_params
                    self.client.sendFlag = True
                else:
                    formatted_params = [f"{value:.2f}" for value in self.current_params]
                    print(f"Updated Parameters: {formatted_params}\n")

                self.dataCollectFlag = True

                # Metabolic Data Collection Handling
                while self.isRun and self.dataCollectFlag:
                    time.sleep(0.1)
                    # If user pressed Stop, exit early without saving or incrementing condCnt
                    if self.stop_requested:
                        with self.data_lock:
                            self.met_list = [[] for _ in range(4)]
                            self.breathCnt = 0
                        print("[HILOMetCmaController] Condition aborted by user. Exiting optimization loop.")
                        self.stop_requested = False
                        return

                if self.isRun:
                    # Save metabolics data for the condition
                    file_name = f'condition_{self.condCnt + 1}_metabolics.csv'
                    file_path = os.path.join(self.frame_path, file_name)

                    met_array = np.array([self.met_list[i] for i in range(4)]).T
                    np.savetxt(
                        file_path,
                        met_array,
                        delimiter=",",
                        fmt="%s",
                        header="Time (s), VO2 (mL/min), VCO2 (mL/min), Metabolic (W)",
                        comments=''
                    )

                    time.sleep(0.2) # Give UI thread time to reflect the updated met_estimate before screenshot
                    self.frameSaveFlag = True
                    while self.isRun and self.frameSaveFlag:
                        time.sleep(0.01)

                    # Reset and save data
                    with self.data_lock:
                        self.met_list = [[] for _ in range(4)]
                        self.breathCnt = 0

                    # Log progress in CSV file
                    with self.data_lock:
                        # Read existing data
                        if os.path.exists(self.csvFileName):
                            df = pd.read_csv(self.csvFileName)
                        else:
                            df = pd.DataFrame(columns=self.headers)

                        df.at[self.condCnt, self.headers[0]] = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                        df.at[self.condCnt, self.headers[2]] = int(self.condCnt + 1)
                        df.at[self.condCnt, self.headers[3]] = round(self.met_estimate[-1], 2)
                        
                        df[self.headers[2]] = df[self.headers[2]].astype('Int64')                                        

                        df.to_csv(self.csvFileName, index=False)

                    print(f"\nCondition #{self.condCnt + 1} completed.")
                    self.condCnt += 1

            # Check if completed a generation, then update CMA (similar to reference code's update_cma)
            if self.isRun:
                # Time to tell CMA-ES about the results and ask for new solutions
                start_idx = self.lambdaPopSize * (self.genCnt - 1)
                end_idx = self.lambdaPopSize * self.genCnt

                with self.data_lock:
                    if end_idx <= len(self.met_estimate) and end_idx <= len(self.candidate_params_list[0]):
                        solutions = self.candidate_params_list[0][start_idx:end_idx]
                        f_values = self.met_estimate[start_idx:end_idx]
                        self.optimizer.tell_results(solutions, f_values)
                        
                        print("Submitted solutions and f_values to CMA-ES.")    
                        
                        if self.genCnt <= self.maxNumbGen:
                            self.genCnt += 1
                            self.update_params()

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