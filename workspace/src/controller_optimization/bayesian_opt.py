#! /usr/bin/env python3

import os
import time
import pandas as pd
import pickle

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

global logger, benchmark, trials
trials = 0


def benchmark_function(x, noise_level=0.1):
    # function to be minimized
    global benchmark, trials
    trials += 1

    new_params = ControllerParameters()
    for i, x_i in enumerate(x):
        new_params.critics[i].cost_weight = x_i

    benchmark.update_parameters(new_params)
    result = benchmark.run_benchmark(timeout=TIMEOUT)
    if not result.success:
        output_dict = {
            'id': str(trials),
            'success': False,
            'score': 0.0,
            'time_elapsed': 0.0,
            'avg_cost': 0.0,
            'distance_to_goal': 0.0,
            'angle_to_goal': 0.0,
            'msg': result.status_msg,
            'metric_path': 'None'
        }
        new_row = {**output_dict, **new_params.to_dict()}
        pd.DataFrame([new_row]).to_csv(
            os.path.join(WORK_DIR, 'output_result.csv'), mode='a', header=False, index=False)
        return 1e6

    metric = benchmark.calculate_metric(result)
    metric_path = benchmark.save_metric(metric)
    score = score_random_search(metric)

    output_dict = {
        'id': str(trials),
        'success': True,
        'score': score,
        'time_elapsed': metric.time_elapsed,
        'avg_cost': metric.avg_cost,
        'distance_to_goal': metric.distance_to_goal,
        'angle_to_goal': metric.angle_to_goal,
        'msg': result.status_msg,
        'metric_path': metric_path
    }

    new_row = {**output_dict, **new_params.to_dict()}
    pd.DataFrame([new_row]).to_csv(
        os.path.join(WORK_DIR, 'output_result.csv'), mode='a', header=False, index=False)

    return score


def main():
    global logger, benchmark
    logger = setup_run('BayesianOpt', os.path.join(BASE_PATH, 'logs'))

    benchmark = ControllerBenchmark(
        logger=logger.getChild('Benchmark'),
        config_path=os.path.join(BASE_PATH, 'config/controller_benchmark_config.yaml'),
        save_path=WORK_DIR)

    # Initialize default controller parameters
    default_mppi_params = ControllerParameters()
    default_mppi_params.load_from_yaml(os.path.join(
        LAUNCH_PATH, 'config/nav2_params_benchmark.yaml'))
    benchmark.update_parameters(default_mppi_params)

    # run default benchmark
    benchmark.update_map('complex_ref')
    ref_result = benchmark.run_benchmark(run_uid='reference')

    if ref_result.success is False:
        logger.error(f"Failed to run benchmark: {ref_result.status_msg}")
        return 1

    ref_metric = benchmark.calculate_metric(ref_result)
    ref_score = score_random_search(ref_metric)
    ref_metric_path = benchmark.save_metric(ref_metric)

    output_dict = {
        'id': str(trials),
        'success': False,
        'score': ref_score,
        'time_elapsed': ref_metric.time_elapsed,
        'avg_cost': ref_metric.avg_cost,
        'distance_to_goal': ref_metric.distance_to_goal,
        'angle_to_goal': ref_metric.angle_to_goal,
        'msg': ref_metric.status_msg,
        'metric_path': ref_metric_path
    }
    new_row = {**output_dict, **default_mppi_params.to_dict()}
    pd.DataFrame([new_row]).to_csv(
        os.path.join(WORK_DIR, 'output_result.csv'), mode='a', header=True, index=False)

    res = gp_minimize(
        benchmark_function,  # the function to minimize
        [(0.01, 100.0)] * 8,  # the bounds on each dimension of x
        acq_func="EI",  # the acquisition function
        n_calls=1000,  # the number of evaluations of f
        n_random_starts=1,  # the number of random initialization points
        noise=0.1**2,  # the noise level (optional)
        random_state=1234  # the random seed
    )

    # save res
    with open(os.path.join(WORK_DIR, 'bayes_res.pkl'), 'wb') as file:
        pickle.dump(res, file)

    print(res)

    del benchmark
    time.sleep(2)

    exit(0)


if __name__ == '__main__':
    main()
