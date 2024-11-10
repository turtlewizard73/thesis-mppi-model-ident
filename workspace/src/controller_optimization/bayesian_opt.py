#! /usr/bin/env python3

import os
import time

from skopt import gp_minimize

import constants
from utils.util_functions import setup_run
from utils.controller_parameters import ControllerParameters
from controller_benchmark import ControllerBenchmark
from run_optimizer import score_random_search

BASE_PATH = constants.BASE_PATH
LAUNCH_PATH = constants.LAUNCH_PATH
OPTIMIZATION_OUTPUT_PATH = constants.OPTIMIZATION_OUTPUT_PATH
stamp = time.strftime('%Y-%m-%d-%H-%M')
WORK_DIR = os.path.join(OPTIMIZATION_OUTPUT_PATH, f'bayes_{stamp}')
TIMEOUT = 60.0

global logger, benchmark


def f(x, noise_level=0.1):
    # function to be minimized
    global benchmark

    new_params = ControllerParameters()
    for i, x_i in enumerate(x):
        new_params.critics[i].cost_weight = x_i

    benchmark.update_parameters(new_params)
    result = benchmark.run_benchmark(timeout=TIMEOUT)
    if not result.success:
        return 1e6

    metric = benchmark.calculate_metric(result)
    # metric_path = benchmark.save_metric(metric)
    score = score_random_search(metric)

    return score


def main():
    global logger, benchmark
    logger = setup_run('BayesianOpt', os.path.join(BASE_PATH, 'logs'))

    benchmark = ControllerBenchmark(
        logger=logger.getChild('Benchmark'),
        config_path=os.path.join(BASE_PATH, 'config/controller_benchmark_config.yaml'),
        result_save_path=os.path.join(WORK_DIR, 'results'),
        metric_save_path=os.path.join(WORK_DIR, 'metrics'))

    res = gp_minimize(
        f,  # the function to minimize
        [(0.01, 100.0)] * 8,  # the bounds on each dimension of x
        acq_func="EI",  # the acquisition function
        n_calls=10,  # the number of evaluations of f
        n_random_starts=10,  # the number of random initialization points
        noise=0.1**2,  # the noise level (optional)
        random_state=1234  # the random seed
    )

    print(res)

    exit(0)


if __name__ == '__main__':
    main()
