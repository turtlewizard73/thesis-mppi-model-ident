#! /usr/bin/env python3
import os
import numpy as np
import time
import yaml
import pickle
from copy import deepcopy

import constants
from controller_benchmark import ControllerBenchmark
from utils.util_functions import setup_run
from utils.controller_parameters import MPPIControllerParameters
from utils.controller_metrics import ControllerMetric

BASE_PATH = os.path.dirname(__file__)
LAUNCH_PATH = '/home/turtlewizard/thesis-mppi-model-ident/workspace/src/controller_launch'
global logger, default_mppi_params, controller_benchmark


def score_random_search(metric: ControllerMetric) -> float:
    # smaller is better
    max_time = 60.0
    max_cost = constants.OCCUPANCY_WALL
    max_distance = np.linalg.norm(metric.path_xy[-1] - metric.path_xy[0]) / 2
    max_angle = 180  # degrees

    weight_time = 0.35
    weight_cost = 0.35
    weight_distance = 0.25
    weight_angle = 0.25

    normalized_time_elapsed = metric.time_elapsed / max_time
    normalized_avg_cost = metric.avg_cost / max_cost
    normalized_distance_to_goal = metric.distance_to_goal / max_distance
    normalized_angle_to_goal = metric.angle_to_goal / max_angle

    score = (
        weight_time * normalized_time_elapsed +
        weight_cost * normalized_avg_cost +
        weight_distance * normalized_distance_to_goal +
        weight_angle * normalized_angle_to_goal
    )

    return score


def random_search():
    global default_mppi_params, controller_benchmark, logger
    stamp = time.strftime('%Y-%m-%d-%H-%M')
    working_dir = os.path.join(
        BASE_PATH, 'optimization_results', f'random_search_{stamp}')
    if not os.path.exists(working_dir):
        os.makedirs(working_dir)

    logger.info(f"Starting random search, output: {working_dir}.")
    controller_benchmark.launch_nodes()

    # run the benchmark with default config to get the reference metric
    result = controller_benchmark.run_benchmark()
    result.uid = 'reference'
    reference_metric = controller_benchmark.calculate_metric(result)
    reference_metric.uid = 'reference'
    reference_score = score_random_search(reference_metric)

    # setup the random search
    num_trials = 2
    timeout = reference_metric.time_elapsed * 2
    best_score = 0.0
    best_params = MPPIControllerParameters(name=default_mppi_params.name)
    best_metric_path = ''
    test_mppi_params = MPPIControllerParameters(name=default_mppi_params.name)

    for i in range(num_trials):
        start_time = time.time()
        logger.info(f'______ Start Trial {i}/{num_trials} ______')
        metric_path = os.path.join(working_dir, f'metric_{i}.pickle')

        # generate parameters
        test_mppi_params.randomize_weights(
            distribution='uniform', lower_bound=0.1, upper_bound=100.0)
        # run simulation and get metrics
        result = controller_benchmark.run_benchmark(
            parameters=test_mppi_params, timeout=timeout)
        result.uid = f'trial_{i}'
        metric = controller_benchmark.calculate_metric(result)
        metric.uid = f'trial_{i}'
        score = score_random_search(metric)

        # evaluate performance and update best parameters if needed
        if score < reference_score:
            best_score = score
            best_params = deepcopy(test_mppi_params)
            best_metric_path = metric_path

        # save the parameters and metrics
        with open(metric_path, 'wb+') as f:
            pickle.dump(reference_metric, f, pickle.HIGHEST_PROTOCOL)

        with open(os.path.join(working_dir, f'output_{i}.yaml'), 'w') as f:
            output_dict = {
                'score': score,
                'parameters': test_mppi_params.to_dict(),
                'metric_path': metric_path}
            yaml.dump(output_dict, f, default_flow_style=False)

        end_time = time.time()
        logger.info(
            f'______ Trial {i}/{num_trials} - Score: {score} finished in {end_time - start_time} s. ______')

    reference_metric_path = os.path.join(
        working_dir, f'reference_metric_{stamp}.pickle')
    with open(reference_metric_path, 'wb+') as f:
        pickle.dump(reference_metric, f, pickle.HIGHEST_PROTOCOL)

    with open(os.path.join(working_dir, f'search_result_{stamp}.yaml'), 'w') as f:
        result_dict = {
            'best_score': best_score,
            'best_parameters': best_params.to_dict(),
            'best_metric_path': best_metric_path,
            'reference_score': reference_score,
            'reference_parameters': controller_benchmark.default_controller_params.to_dict(),
            'reference_metric_path': reference_metric_path}
        yaml.dump(result_dict, f, default_flow_style=False)


def main():
    global logger, default_mppi_params, controller_benchmark
    logger = setup_run(
        logger_name='ControllerOptimizer',
        log_file_path=os.path.join(BASE_PATH, 'logs')
    )

    # Initialize default controller parameters
    default_mppi_params = MPPIControllerParameters()
    default_mppi_params.load_from_yaml(
        os.path.join(LAUNCH_PATH, 'config/nav2_params_benchmark.yaml'))

    logger.info("Default mppi parameters initialized: \n%s", default_mppi_params)

    controller_benchmark = ControllerBenchmark(
        logger=logger,
        config_path=os.path.join(BASE_PATH, 'config/controller_benchmark_config.yaml'),
        mppi_params=default_mppi_params)

    try:
        random_search()
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt, stopping.")

    exit(0)


if __name__ == '__main__':
    main()
