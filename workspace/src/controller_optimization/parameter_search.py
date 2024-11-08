from typing import TypedDict
import os

import constants
from controller_benchmark import ControllerBenchmark

from utils.controller_parameters import MPPIControllerParameters
from utils.controller_parameters_dict

OUTPUT_PATH = constants.OPTIMIZATION_RESULTS_PATH


# for ensuring numpy types can go to yaml
class SearchOuputDict(TypedDict):
    num_of_runs: int
    successful_runs: int
    failed_runs: int
    loop_time: float  # [s]

    timeout: float  # [s]

    best_score: float
    best_parameters: dict
    best_metrics_path: str

    reference_score: float
    reference_parameters: dict
    reference_metrics_path: str


class OutputRowDict(TypedDict):
    id: str
    success: bool
    score: float
    time_elapsed: float
    scored_metrics: dict
    msg: dict


class ParameterSearch:
    def __init__(
            self, logger,
            benchmark: ControllerBenchmark,
            default_params: MPPIControllerParameters) -> None:
        self.logger = logger
        self.logger.info('Initializing ParameterSearch')
        self.benchmark = benchmark

    def _setup_directories(self):
        if not os.path.exists(OUTPUT_PATH):
            os.makedirs(OUTPUT_PATH)

    def _initialize_dataframe(self):
        pass

    def _run_reference_banchmark(self):
        pass

    def _score_metrics(self):
        pass

    def _save_results(self):
        pass

    def run_reference_benchmark(self):
        pass

    def run_random_search(self):

        pass

    def run_grid_search(self):
        pass
