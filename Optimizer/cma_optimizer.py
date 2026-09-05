import json
import os
import numpy as np
import cma
import pickle
from threading import Thread
import shutil

from param_utils import ParameterUtils

import warnings
from cma.evolution_strategy import InjectionWarning
warnings.simplefilter("ignore", InjectionWarning)
warnings.filterwarnings("ignore", message="TPA: apparent inconsistency with mirrored samples")

class CMAOptimizer(Thread):
    """
    Handles CMA-ES specific optimization functionality.
    This class provides methods for the controller to use but doesn't run the optimization loop.
    """
    def __init__(self, controller, subject_number, max_generations, seed=0):
        super().__init__()
        self.controller = controller
        self.subject_number = subject_number
        self.max_generations = max_generations
        # Fixed seed so a run can be reproduced; recorded in cma_settings.json.
        self.seed = seed
        self.lambda_pop_size = 4 + int(np.floor(3 * np.log(6)))
        self.total_conditions = self.max_generations * self.lambda_pop_size
        
        # File paths
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.file_path = os.path.join(current_dir, 'outcmaes', f'subject_{self.subject_number}')
        self.cmaFileName = os.path.join(self.file_path, f'subject_{self.subject_number}_cmaLogFile')
        
        self.es = None  # CMAEvolutionStrategy instance
        self.param_utils = ParameterUtils()
    
    def initialize_cma_es(self, initial_params, scaling_range):
        """Initialize the CMA-ES optimizer"""
        initial_sigma = 1.5
        initial_params_scaled = self.param_utils.scale_params(initial_params)

        print(f"Initializing CMAEvolutionStrategy with bounds: {scaling_range}, seed: {self.seed}")
        self.es = cma.CMAEvolutionStrategy(
            initial_params_scaled, 
            initial_sigma, 
            {
                'bounds': scaling_range,           
                'AdaptSigma': cma.sigma_adaptation.CMAAdaptSigmaTPA,
                'seed': self.seed
            }
        )
        self.record_run_settings(initial_params, initial_params_scaled, initial_sigma, scaling_range)
        self.save_cma_state(source='init')
        print("CMAEvolutionStrategy initialized successfully.")
        return True

    def record_run_settings(self, initial_params, initial_params_scaled, initial_sigma, scaling_range):
        """Write the settings that fully determine this run, so it can be reproduced."""
        os.makedirs(self.file_path, exist_ok=True)
        settings = {
            'seed': self.seed,
            'sigma0': initial_sigma,
            'population_size': self.lambda_pop_size,
            'max_generations': self.max_generations,
            'bounds_scaled': list(scaling_range),
            'initial_params': [float(v) for v in initial_params],
            'initial_params_scaled': [float(v) for v in initial_params_scaled],
            'cma_version': getattr(cma, '__version__', 'unknown'),
        }
        path = os.path.join(self.file_path, f'subject_{self.subject_number}_cma_settings.json')
        with open(path, 'w') as f:
            json.dump(settings, f, indent=2)
        print(f"Run settings written to: {path}")

    def load_cma_state(self):
        """Load saved CMA-ES state if available"""
        if os.path.exists(self.cmaFileName):
            try:
                with open(self.cmaFileName, 'rb') as f:
                    previous_cma = f.read()
                    self.es = pickle.loads(previous_cma)
                return True
            except Exception as e:
                print(f"Error loading CMA-ES state: {e}")
                return False
        return False
    
    def save_cma_state(self, source=None):
        """Save current CMA-ES state"""
        if self.es is not None:
            try:
                with open(self.cmaFileName, 'wb') as f:
                    f.write(self.es.pickle_dumps())
                print("CMA-ES state saved successfully.")

                backup_dir = os.path.join(os.path.dirname(self.cmaFileName), 'cmaLogBackup')
                os.makedirs(backup_dir, exist_ok=True)

                label = source if source else 'unknown'
                gen = getattr(self.controller, 'genCnt', None)
                gen_str = f"gen{gen:02d}" if gen is not None else "genXX"
                base_name = os.path.basename(self.cmaFileName)
                backup_name = f"{base_name}_backup_{gen_str}_{label}"
                backup_path = os.path.join(backup_dir, backup_name)

                shutil.copyfile(self.cmaFileName, backup_path)
                print(f"Backup created at: {backup_path}")

                return True
            except Exception as e:
                print(f"Error saving CMA-ES state: {e}")
                return False
        return False
    
    def ask_new_parameters(self):
        """Ask CMA-ES for new candidate solutions"""
        candidate_params = self.es.ask()
        self.save_cma_state(source='ask')
        print(f"Generated {len(candidate_params)} candidate parameters.")
        return candidate_params
    
    def tell_results(self, solutions, costs):
        """Tell CMA-ES the evaluation results"""        
        self.es.tell(solutions, np.array(costs))
        self.save_cma_state(source='tell')        
        return True
            
    def is_optimization_complete(self):
        """Check if optimization should stop"""
        return self.es.stop()
    
    def get_sigma(self):
        """Get current sigma value"""
        return self.es.sigma