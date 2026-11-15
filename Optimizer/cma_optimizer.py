import numpy as np
import cma
from threading import Thread

from param_utils import ParameterUtils

class CMAOptimizer(Thread):
    """
    Handles CMA-ES specific optimization functionality.
    This class provides methods for the controller to use but doesn't run the optimization loop.
    """
    def __init__(self, controller, subject_number, max_generations, seed=None):
        super().__init__()
        self.controller = controller
        self.subject_number = subject_number
        self.max_generations = max_generations
        # Seed of numpy's random generator, which pycma samples from. None, as in the study,
        # gives a time-based seed; the tests and the worked example pass one. The same seed and
        # costs give the same candidates only with the same numpy build (version and BLAS/LAPACK
        # library) and pycma version.
        self.seed = seed
        self.lambda_pop_size = 4 + int(np.floor(3 * np.log(6)))
        self.total_conditions = self.max_generations * self.lambda_pop_size

        self.es = None  # CMAEvolutionStrategy instance
        self.param_utils = ParameterUtils()
    
    def initialize_cma_es(self, initial_params, scaling_range):
        """Initialize the CMA-ES optimizer"""
        initial_sigma = 1.5
        initial_params_scaled = self.param_utils.scale_params(initial_params)

        # CMA_active_injected 1: the injected solutions (the TPA pair and the re-measured
        # condition) enter the covariance update with their weights like the others. pycma
        # recognizes them only when told exactly as returned, which logged (rounded)
        # parameters are not, so this keeps a run continued from the condition log the same
        # as an uninterrupted one.
        options = {'bounds': scaling_range, 'AdaptSigma': cma.sigma_adaptation.CMAAdaptSigmaTPA,
                   'CMA_active_injected': 1}
        if self.seed is not None:
            options['seed'] = self.seed
        print(f"Initializing CMAEvolutionStrategy with bounds: {scaling_range}")
        self.es = cma.CMAEvolutionStrategy(initial_params_scaled, initial_sigma, options)
        print("CMAEvolutionStrategy initialized successfully.")
        return True

    def ask_new_parameters(self, reinject=None):
        """Ask CMA-ES for new candidate solutions.

        reinject: a solution returned by an earlier ask() to be evaluated again in this
        generation. It is injected before ask(), so pycma returns it after the TPA mirrored
        pair (ask indices 0 and 1, from the second generation on), or first in generation 1.
        """
        if reinject is not None:
            # inject() takes a genotype: map the solution back through the [0, 10] bound
            # transform, so ask() returns it, up to floating-point rounding, also near the bounds.
            self.es.inject([self.es.boundary_handler.inverse(np.array(reinject, dtype=float))], force=True)
        candidate_params = self.es.ask()
        print(f"Generated {len(candidate_params)} candidate parameters.")
        return candidate_params
    
    def tell_results(self, solutions, costs, keep_sigma=False):
        """Tell CMA-ES the evaluation results, in the order ask() returned the solutions, with
        the parameters as presented, so that CMA-ES learns from the conditions as measured.

        pycma folds a sample outside the bounds into them, and the folded one is presented;
        pycma would update from the unfolded sample, so that stored sample is dropped and
        pycma uses the presented one. The same holds for a solution that the timing
        constraint changed and for the logged solutions told when a run continues from the
        condition log, which pycma did not return. TPA adapts the step size from the costs at
        told indices 0 and 1, the mirrored pair; when either is not told exactly as sampled,
        TPA's check that the pair is exactly mirrored is skipped.

        keep_sigma: leave the step size unchanged in this update. When a member of the pair
        was excluded there is no pair to compare. pycma 3.4.0 has no option for this, so
        TPA's update and its pair check are switched off for this tell only. The mean and
        covariance are updated from the told solutions as usual.

        Solutions pycma injected (the TPA pair, the re-measured solution) that it did not get
        back are removed from its injection archive, which would otherwise report them as
        orphaned two generations later.
        """
        sent = self.es.sent_solutions
        exact_pair = True
        for i, x in enumerate(solutions):
            stored = sent.get(x)
            if stored is not None and np.max(np.abs(
                    stored['geno'] - self.es.boundary_handler.inverse(np.array(x, dtype=float)))) > 1e-9:
                sent.pop(x)  # folded into the bounds
                stored = None
            if stored is None and i < 2:
                exact_pair = False
        tpa = self.es.adapt_sigma
        is_tpa = isinstance(tpa, cma.sigma_adaptation.CMAAdaptSigmaTPA)
        keep_sigma = keep_sigma and is_tpa
        skip_pair_check = is_tpa and (keep_sigma or not exact_pair)
        if keep_sigma:
            tpa.update = lambda *args, **kwargs: None
        if skip_pair_check:
            tpa.check_consistency = lambda *args, **kwargs: None
        try:
            self.es.tell(solutions, np.array(costs))
        finally:
            if keep_sigma:
                del tpa.update
            if skip_pair_check:
                del tpa.check_consistency
        if keep_sigma or not exact_pair:
            archive = self.es._injected_solutions_archive
            for key in list(archive):
                archive.pop(key)
        return True
            
    def is_optimization_complete(self):
        """Check if optimization should stop"""
        return self.es.stop()
    
    def get_sigma(self):
        """Get current sigma value"""
        return self.es.sigma