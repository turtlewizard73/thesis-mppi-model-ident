#! /usr/bin/env python3
import os
import numpy as np
import time
import yaml
import pickle
from copy import deepcopy
import pandas as pd
import re

import constants
from controller_benchmark import ControllerBenchmark
from utils.util_functions import setup_run
from utils.controller_parameters import ControllerParameters
from utils.controller_metrics import ControllerMetric

BASE_PATH = constants.BASE_PATH
LAUNCH_PATH = constants.LAUNCH_PATH
OPTIMIZATION_OUTPUT_PATH = constants.OPTIMIZATION_OUTPUT_PATH
global logger, benchmark


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

    return float(score)


def run_benchmark_trial(
        params: ControllerParameters, trial_id: str, timeout: float):
    """Runs a single benchmark trial with the given parameters."""
    global benchmark, logger

    benchmark.update_parameters(params)
    benchmark.update_map('complex_ref')
    result = benchmark.run_benchmark(run_uid=trial_id, timeout=timeout)

    if not result.success:
        logger.error(f"Trial {trial_id} failed: {result.status_msg}")
        return {
            'id': trial_id,
            'success': False,
            'score': 0.0,
            'time_elapsed': 0.0,
            'avg_cost': 0.0,
            'distance_to_goal': 0.0,
            'angle_to_goal': 0.0,
            'msg': result.status_msg,
            'metric_path': 'None'
        }

    metric = benchmark.calculate_metric(result)
    metric_path = benchmark.save_metric(metric)
    score = score_random_search(metric)

    logger.info(f"Trial {trial_id} finished with score: {score}.")
    return {
        'id': trial_id,
        'success': True,
        'score': score,
        'time_elapsed': metric.time_elapsed,
        'avg_cost': metric.avg_cost,
        'distance_to_goal': metric.distance_to_goal,
        'angle_to_goal': metric.angle_to_goal,
        'msg': result.status_msg,
        'metric_path': metric_path
    }


def main():
    NAME = 'distribution'

    global logger, default_mppi_params, benchmark
    logger = setup_run('Search', os.path.join(BASE_PATH, 'logs'))
    stamp = time.strftime('%Y-%m-%d-%H-%M')
    WORK_DIR = os.path.join(OPTIMIZATION_OUTPUT_PATH, f'{NAME}_{stamp}')
    output_csv = os.path.join(WORK_DIR, 'output_result.csv')
    output_rows = []

    benchmark = ControllerBenchmark(
        logger=logger.getChild('Benchmark'),
        config_path=os.path.join(BASE_PATH, 'config/controller_benchmark_config.yaml'),
        save_path=WORK_DIR)

    # Initialize default controller parameters
    default_mppi_params = ControllerParameters()
    default_mppi_params.load_from_yaml(os.path.join(
        LAUNCH_PATH, 'config/nav2_params_benchmark.yaml'))

    # run default benchmark
    benchmark.launch_nodes()
    benchmark.update_map('complex_ref')
    benchmark.update_parameters(default_mppi_params)
    ref_result = benchmark.run_benchmark(run_uid='reference')

    if ref_result.success is False:
        logger.error(f"Failed to run benchmark: {ref_result.status_msg}")
        return 1

    ref_metric = benchmark.calculate_metric(ref_result)
    ref_score = score_random_search(ref_metric)
    ref_metric_path = benchmark.save_metric(ref_metric)

    ref_output_dict = {
        'id': 'reference',
        'success': True,
        'score': ref_score,
        'time_elapsed': ref_metric.time_elapsed,
        'avg_cost': ref_metric.avg_cost,
        'distance_to_goal': ref_metric.distance_to_goal,
        'angle_to_goal': ref_metric.angle_to_goal,
        'msg': ref_result.status_msg,
        'metric_path': ref_metric_path,
        **default_mppi_params.to_dict()
    }
    output_rows.append(ref_output_dict)
    pd.DataFrame(output_rows).to_csv(output_csv, index=False)

    # setup search parameters
    TIMEOUT = ref_metric.time_elapsed * 2
    num_trials = 10

    best_score = ref_score
    best_params = deepcopy(default_mppi_params)
    best_metric_path = ref_metric_path

    test_params = ControllerParameters()
    benchmark.update_map('complex_ref')

    try:
        loop_start_time = time.time()
        successful_trials = 0
        for i in range(1, num_trials + 1):
            # get new parameters

            # test_params.randomize_weights(
            #     distribution='uniform', lower_bound=0.1, upper_bound=100.0,
            #     decimals=0)

            logger.info(f"___ Starting trial {i}/{num_trials} ___")

            trial_result = run_benchmark_trial(test_params, i, TIMEOUT)
            if trial_result['success'] is True:
                successful_trials += 1

            if trial_result['success'] is True and (trial_result['score'] < best_score):
                best_score = trial_result['score']
                best_params = deepcopy(test_params)
                best_metric_path = trial_result['metric_path']

            new_row = {**trial_result, **test_params.to_dict()}
            output_rows.append(new_row)
            pd.DataFrame([new_row]).to_csv(output_csv, mode='a', header=False, index=False)

            logger.info(
                f"___ Trial {i}/{num_trials} finished with score: {trial_result['score']} ___")

    except KeyboardInterrupt:
        logger.info("Keyboard interrupt, stopping.")
    except Exception as e:
        logger.error(f"Error in main: {e}")
    finally:
        pd.DataFrame(output_rows).to_csv(os.path.join(
            WORK_DIR, 'output_result_final.csv'), index=False)

        with open(os.path.join(WORK_DIR, 'output_0_result.yaml'), 'w') as f:
            yaml.dump({
                'num_trials': num_trials,
                'successful_trials': successful_trials,
                'timeout': TIMEOUT,
                'loop_time': time.time() - loop_start_time,
                'best_score': float(best_score),
                'best_parameters': best_params.to_dict(),
                'best_metric_path': best_metric_path,
                'reference_score': float(ref_score),
                'reference_parameters': default_mppi_params.to_dict(),
                'reference_metric_path': ref_metric_path
            }, f, default_flow_style=False)

        del benchmark

        time.sleep(5)
        return 0


if __name__ == '__main__':
    exit(main())
