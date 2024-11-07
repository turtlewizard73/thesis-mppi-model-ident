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
from utils.controller_parameters import MPPIControllerParameters
from utils.controller_metrics import ControllerMetric

BASE_PATH = os.path.dirname(__file__)
LAUNCH_PATH = '/home/turtlewizard/thesis-mppi-model-ident/workspace/src/controller_launch'
global logger, default_mppi_params, controller_benchmark


def get_db_from_old_metrics():
    input_metrics_path = os.path.join(
        BASE_PATH, 'optimization_results', 'random_search_2024-11-07-01-05')
    output_path = os.path.join(
        BASE_PATH, 'optimization_results', 'random_search_1000')
    if not os.path.exists(output_path):
        os.makedirs(output_path)
    output_metrics_path = os.path.join(output_path, 'metrics')
    if not os.path.exists(output_metrics_path):
        os.makedirs(output_metrics_path)
    output_rows = []

    # read all yamls
    for i, file in enumerate(os.listdir(input_metrics_path)):
        if file.startswith('output'):
            print(file)

            with open(os.path.join(input_metrics_path, file), 'r') as f:
                data = yaml.safe_load(f)

                id_ = re.findall(r'\d+', file)

                output_dict = {
                    'id': id_,
                }

                # if file exists data['metric_path']
                if not os.path.exists(data['metric_path']):
                    output_dict['success'] = False
                    output_dict['score'] = 0.0
                    output_dict['time_elapsed'] = 0.0
                    output_dict['avg_cost'] = 0.0
                    output_dict['distance_to_goal'] = 0.0
                    output_dict['angle_to_goal'] = 0.0
                    output_dict['msg'] = 'Failed'
                    output_dict['metric_path'] = ''
                    output_rows.append(output_dict)
                    continue

                with open(data['metric_path'], 'rb') as m:
                    metric: ControllerMetric = pickle.load(m)
                    new_metric_path = os.path.join(
                        output_metrics_path, f'metric_{id_}.pickle')
                    pickle.dump(metric, open(new_metric_path, 'wb+'), pickle.HIGHEST_PROTOCOL)

                    if metric.success is True:
                        output_dict['success'] = metric.success
                        output_dict['score'] = score_random_search(metric)
                        output_dict['time_elapsed'] = metric.time_elapsed
                        output_dict['avg_cost'] = metric.avg_cost
                        output_dict['distance_to_goal'] = metric.distance_to_goal
                        output_dict['angle_to_goal'] = metric.angle_to_goal
                        output_dict['msg'] = metric.msg
                        output_dict['metric_path'] = new_metric_path

                        output_rows.append(output_dict)


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
    # ____________ SETUP RANDOM SEARCH ____________
    global default_mppi_params, controller_benchmark, logger
    stamp = time.strftime('%Y-%m-%d-%H-%M')
    working_dir = os.path.join(
        BASE_PATH, 'optimization_results', f'random_search_{stamp}')
    if not os.path.exists(working_dir):
        os.makedirs(working_dir)
    working_metrics_dir = os.path.join(working_dir, 'metrics')
    if not os.path.exists(working_metrics_dir):
        os.makedirs(working_metrics_dir)

    # ____________ SETUP OUTPUT DATAFRAME ____________
    df_columns = [
        'id', 'success',
        'score', 'time_elapsed', 'avg_cost', 'distance_to_goal', 'angle_to_goal',
        'msg']
    df_columns = df_columns + [f'{c}.weight' for c in constants.DEFAULT_MPPI_CRITIC_NAMES]
    df_columns = df_columns.append('metric_path')
    output_rows = []

    # ____________ RUN REFERENCE BENCHMARK ____________
    logger.info(f"Starting random search, output: {working_dir}.")
    controller_benchmark.launch_nodes()
    result = controller_benchmark.run_benchmark(
        parameters=default_mppi_params)
    if result.success is False:
        logger.error(f"Failed to run benchmark: {result.status_msg}")
        return
    result.uid = 'reference'
    reference_metric = controller_benchmark.calculate_metric(result)
    reference_metric.uid = 'reference'
    reference_score = score_random_search(reference_metric)

    reference_metric_path = os.path.join(working_metrics_dir, 'metric_0.pickle')
    with open(reference_metric_path, 'wb+') as f:
        pickle.dump(reference_metric, f, pickle.HIGHEST_PROTOCOL)

    # add the reference to the database
    reference_data = {
        'id': 0,
        'success': result.success,
        'score': reference_score,
        'time_elapsed': reference_metric.time_elapsed,
        'avg_cost': reference_metric.avg_cost,
        'distance_to_goal': reference_metric.distance_to_goal,
        'angle_to_goal': reference_metric.angle_to_goal,
        'msg': result.status_msg,
        'metric_path': reference_metric_path,
    }
    reference_data.update(default_mppi_params.to_dict_critics())
    output_rows.append(reference_data)

    # setup the random search
    num_trials = 2
    timeout = reference_metric.time_elapsed * 2
    best_score = reference_score
    best_params = deepcopy(default_mppi_params)
    best_metric_path = ''
    test_mppi_params = deepcopy(default_mppi_params)

    loop_start_time = time.time()
    successful_trials = 0
    try:
        for i in range(1, num_trials + 1):
            logger.info(f'______ Start Trial {i}/{num_trials} ______')
            start_time = time.time()

            # generate parameters
            test_mppi_params.randomize_weights(
                distribution='uniform', lower_bound=0.1, upper_bound=100.0,
                decimals=0)

            # run simulation and get metrics
            result = controller_benchmark.run_benchmark(
                parameters=test_mppi_params, timeout=timeout)

            if result.success is False:
                logger.error(f"______ Trial {i}/{num_trials} failed: {result.status_msg} ______")
                output_dict = {
                    'id': i,
                    'success': False,
                    'score': 0.0,
                    'time_elapsed': 0.0,
                    'avg_cost': 0.0,
                    'distance_to_goal': 0.0,
                    'angle_to_goal': 0.0,
                    'msg': result.status_msg,
                    'metric_path': '',
                }
                output_dict.update(test_mppi_params.to_dict_critics())
                output_rows.append(output_dict)
                continue
            successful_trials += 1

            # calculate and save the metric
            metric = controller_benchmark.calculate_metric(result)
            metric_path = os.path.join(working_metrics_dir, f'metric_{i}.pickle')
            with open(metric_path, 'wb+') as f:
                pickle.dump(test_mppi_params, f, pickle.HIGHEST_PROTOCOL)

            score = score_random_search(metric)

            # evaluate performance and update best parameters if needed
            score = score_random_search(metric)
            if score < best_score:
                best_score = score
                best_params = deepcopy(test_mppi_params)
                best_metric_path = metric_path

            output_dict = {
                'id': i,
                'success': True,
                'score': score,
                'time_elapsed': metric.time_elapsed,
                'avg_cost': metric.avg_cost,
                'distance_to_goal': metric.distance_to_goal,
                'angle_to_goal': metric.angle_to_goal,
                'msg': result.status_msg,
                'metric_path': metric_path,
            }
            output_dict.update(test_mppi_params.to_dict_critics())
            output_rows.append(output_dict)

            end_time = time.time()
            logger.info(
                f'______ Trial {i}/{num_trials} - Score: {score} finished in {end_time - start_time} s. ______')

    except KeyboardInterrupt:
        logger.info("Keyboard interrupt, stopping.")
    except Exception as e:
        logger.error(f"Error in trial: {e}")

    # save the output dataframe
    output_df = pd.DataFrame(output_rows)
    output_df.to_csv(os.path.join(working_dir, 'output_0_result.csv'), index=False)

    with open(os.path.join(working_dir, 'output_0_result.yaml'), 'w') as f:
        result_dict = {
            'num_trials': num_trials,
            'successful_trials': successful_trials,
            'timeout': timeout,
            'loop_time': time.time() - loop_start_time,
            'best_score': float(best_score),
            'best_parameters': best_params.to_dict(),
            'best_metric_path': best_metric_path,
            'reference_score': float(reference_score),
            'reference_parameters': controller_benchmark.default_controller_params.to_dict(),
            'reference_metric_path': reference_metric_path}
        yaml.dump(result_dict, f, default_flow_style=False)
    controller_benchmark.stop_nodes()


def grid_search():
    # lets assume critics have nothing to do with each other
    # so go one critic from 1-100 by increment of 1 and rest stay default
    global default_mppi_params, controller_benchmark, logger
    stamp = time.strftime('%Y-%m-%d-%H-%M')
    working_dir = os.path.join(
        BASE_PATH, 'optimization_results', f'grid_search_{stamp}')
    if not os.path.exists(working_dir):
        os.makedirs(working_dir)
    working_metrics_dir = os.path.join(working_dir, 'metrics')
    if not os.path.exists(working_metrics_dir):
        os.makedirs(working_metrics_dir)

    # ____________ SETUP OUTPUT DATAFRAME ____________
    df_columns = [
        'id', 'success',
        'score', 'time_elapsed', 'avg_cost', 'distance_to_goal', 'angle_to_goal',
        'msg']
    df_columns = df_columns + [f'{c}.weight' for c in constants.DEFAULT_MPPI_CRITIC_NAMES]
    df_columns = df_columns.append('metric_path')
    output_df = pd.DataFrame(columns=df_columns)
    output_rows = []

    # ____________ RUN REFERENCE BENCHMARK ____________
    logger.info(f"Starting grid search, output: {working_dir}.")
    controller_benchmark.launch_nodes()
    result = controller_benchmark.run_benchmark(
        parameters=default_mppi_params)
    if result.success is False:
        logger.error(f"Failed to run benchmark: {result.status_msg}")
        return
    result.uid = 'reference'
    reference_metric = controller_benchmark.calculate_metric(result)
    reference_metric.uid = 'reference'
    reference_score = score_random_search(reference_metric)

    reference_metric_path = os.path.join(working_metrics_dir, 'metric_0.pickle')
    with open(reference_metric_path, 'wb+') as f:
        pickle.dump(reference_metric, f, pickle.HIGHEST_PROTOCOL)

    # add the reference to the database
    reference_data = {
        'id': 'reference',
        'success': result.success,
        'score': reference_score,
        'time_elapsed': reference_metric.time_elapsed,
        'avg_cost': reference_metric.avg_cost,
        'distance_to_goal': reference_metric.distance_to_goal,
        'angle_to_goal': reference_metric.angle_to_goal,
        'msg': result.status_msg,
        'metric_path': reference_metric_path,
    }
    reference_data.update(default_mppi_params.to_dict_critics())
    output_rows.append(reference_data)

    # setup the grid search
    critic_weights = np.arange(1, 101, 1)
    critic_weights = np.arange(1, 101, 20)  # demo run
    timeout = reference_metric.time_elapsed * 2
    best_score = reference_score
    best_params = deepcopy(default_mppi_params)
    best_metric_path = ''
    loop_start_time = time.time()
    successful_trials = 0
    try:
        for critic_name in constants.DEFAULT_MPPI_CRITIC_NAMES:
            for w in critic_weights:
                logger.info(f'______ Start Grid Search {critic_name} = {w} ______')
                start_time = time.time()

                # generate parameters
                test_mppi_params = deepcopy(default_mppi_params)
                test_mppi_params.set_critic_weight(critic_name, w)

                # run simulation and get metrics
                result = controller_benchmark.run_benchmark(
                    parameters=test_mppi_params, timeout=timeout)
                if result.success is False:
                    logger.error(
                        f"______ Grid Search {critic_name} {w} failed: {result.status_msg} ______")
                    output_dict = {
                        'id': f'{critic_name}_{w}',
                        'success': False,
                        'score': 0.0,
                        'time_elapsed': 0.0,
                        'avg_cost': 0.0,
                        'distance_to_goal': 0.0,
                        'angle_to_goal': 0.0,
                        'msg': result.status_msg,
                        'metric_path': 'None',
                    }
                    output_dict.update(test_mppi_params.to_dict_critics())
                    output_rows.append(output_dict)
                    continue
                successful_trials += 1

                # calculate and save the metric
                metric = controller_benchmark.calculate_metric(result)
                metric_path = os.path.join(
                    working_metrics_dir, f'metric_{critic_name.lower()}_{w}.pickle')
                with open(metric_path, 'wb+') as f:
                    pickle.dump(test_mppi_params, f, pickle.HIGHEST_PROTOCOL)

                # evaluate performance and update best parameters if needed
                score = score_random_search(metric)
                if score < best_score:
                    best_score = score
                    best_params = deepcopy(test_mppi_params)
                    best_metric_path = metric_path

                output_dict = {
                    'id': f'{critic_name}_{w}',
                    'success': True,
                    'score': score,
                    'time_elapsed': metric.time_elapsed,
                    'avg_cost': metric.avg_cost,
                    'distance_to_goal': metric.distance_to_goal,
                    'angle_to_goal': metric.angle_to_goal,
                    'msg': result.status_msg,
                    'metric_path': metric_path,
                }
                output_dict.update(test_mppi_params.to_dict_critics())
                output_rows.append(output_dict)

                end_time = time.time()
                logger.info(
                    f'______ Grid Search {critic_name} {w} - Score: {score} finished in {end_time - start_time} s. ______')
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt, stopping.")
    except Exception as e:
        logger.error(f"Error in grid search: {e}")

    # save the output dataframe
    output_df = pd.concat([output_df, pd.DataFrame(output_rows)])
    output_df.to_csv(os.path.join(working_dir, 'output_0_result.csv'), index=False)

    with open(os.path.join(working_dir, 'output_0_result.yaml'), 'w') as f:
        result_dict = {
            'num_trials': len(critic_weights) * len(constants.DEFAULT_MPPI_CRITIC_NAMES),
            'successful_trials': successful_trials,
            'critic_weights': critic_weights.tolist(),
            'timeout': timeout,
            'loop_time': time.time() - loop_start_time,
            'best_score': float(best_score),
            'best_parameters': best_params.to_dict(),
            'best_metric_path': best_metric_path,
            'reference_score': float(reference_score),
            'reference_parameters': controller_benchmark.default_controller_params.to_dict(),
            'reference_metric_path': reference_metric_path}
        yaml.dump(result_dict, f, default_flow_style=False)

    controller_benchmark.stop_nodes()


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

    del controller_benchmark
    time.sleep(5)
    exit(0)


if __name__ == '__main__':
    main()
