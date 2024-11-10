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


def run_benchmark_trial(
        params: ControllerParameters, trial_id: str, timeout: float):
    """Runs a single benchmark trial with the given parameters."""
    global benchmark, logger

    benchmark.update_parameters(params)
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
    global logger, default_mppi_params, benchmark
    logger = setup_run('Search', os.path.join(BASE_PATH, 'logs'))
    stamp = time.strftime('%Y-%m-%d-%H-%M')
    WORK_DIR = os.path.join(OPTIMIZATION_OUTPUT_PATH, f'random_search_{stamp}')
    output_csv = os.path.join(WORK_DIR, 'output_result.csv')
    output_rows = []

    benchmark = ControllerBenchmark(
        logger=logger.getChild('Benchmark'),
        config_path=os.path.join(BASE_PATH, 'config/controller_benchmark_config.yaml'),
        result_save_path=os.path.join(WORK_DIR, 'results'),
        metric_save_path=os.path.join(WORK_DIR, 'metrics'))

    # Initialize default controller parameters
    default_mppi_params = ControllerParameters()
    default_mppi_params.load_from_yaml(os.path.join(
        LAUNCH_PATH, 'config/nav2_params_benchmark.yaml'))

    # run default benchmark
    benchmark.launch_nodes()
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
    output_df = pd.DataFrame(output_rows).to_csv(output_csv, index=False)

    # setup search parameters
    TIMEOUT = ref_metric.time_elapsed * 2
    num_trials = 10

    best_score = ref_score
    best_params = deepcopy(default_mppi_params)
    best_metric_path = ref_metric_path

    test_params = ControllerParameters()

    try:
        loop_start_time = time.time()
        successful_trials = 0
        for i in range(1, num_trials + 1):
            # get new parameters
            test_params.randomize_weights(
                distribution='uniform', lower_bound=0.1, upper_bound=100.0,
                decimals=0)
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
