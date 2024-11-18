#! /usr/bin/env python3
import os
import time

import constants
from controller_benchmark import ControllerBenchmark
from controller_optimizer import ControllerOptimizer
from utils.util_functions import setup_run

BASE_PATH = constants.BASE_PATH
LAUNCH_PATH = constants.LAUNCH_PATH
OPTIMIZATION_OUTPUT_PATH = constants.OPTIMIZATION_OUTPUT_PATH


def main():
    NAME = 'test0'

    logger = setup_run('Search', os.path.join(BASE_PATH, 'logs'))
    stamp = time.strftime(constants.TIMESTAMP_FORMAT)
    WORK_DIR = os.path.join(OPTIMIZATION_OUTPUT_PATH, f'{NAME}_{stamp}')

    benchmark = ControllerBenchmark(
        logger=logger.getChild('Benchmark'),
        config_path=os.path.join(BASE_PATH, 'config/controller_benchmark_config.yaml'),
        save_path=WORK_DIR)

    optimizer = ControllerOptimizer(
        logger=logger.getChild('Optimizer'),
        controller_benchmark=benchmark,
        config_path=os.path.join(BASE_PATH, 'config/controller_optimizer_config.yaml'),
        work_dir=WORK_DIR)

    optimizer.setup_run(trial_name='test_trial')

    optimizer.run_reference()
    optimizer.run()

    time.sleep(5)
    return 0


if __name__ == '__main__':
    exit(main())
