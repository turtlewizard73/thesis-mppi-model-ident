from typing import TypedDict
import os

import constants
from controller_benchmark import ControllerBenchmark


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
