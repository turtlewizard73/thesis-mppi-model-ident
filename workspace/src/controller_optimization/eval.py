import os
from typing import Dict, List

import numpy as np
import matplotlib.pyplot as plt
import yaml
import pandas as pd

import constants
from utils.util_functions import setup_run, numpy_dict_to_list
from controller_benchmark import ControllerBenchmark
from utils.controller_metrics import ControllerMetric

WORK_DIR = os.path.join(constants.REPO_PATH, 'evaluation/default_params')


def main():
    logger = setup_run('Eval')
    plt.style.context('bmh')

    cb = ControllerBenchmark(
        logger=logger.getChild('Benchmark'),
        config_path=os.path.join(constants.BASE_PATH, 'config/controller_benchmark_config.yaml'),
        save_path=WORK_DIR
    )

    output_dict = {}

    for benchmark in os.listdir(WORK_DIR):
        benchmark_path = os.path.join(WORK_DIR, benchmark)
        if not os.path.isdir(benchmark_path):
            continue

        metrics_list: List[ControllerMetric] = []
        benchmark_output_dict = {}

        logger.info('___ Loading metrics for benchmark %s ___', benchmark)
        for metric in os.listdir(os.path.join(benchmark_path, 'metrics')):
            if not metric.endswith('.pickle'):
                continue

            metric_path = os.path.join(benchmark_path, 'metrics', metric)
            loaded_metric = cb.load_metric(metric_path)
            if loaded_metric.success is False:
                logger.error(f'Metric not used: {loaded_metric.status_msg}')
                continue
            metrics_list.append(loaded_metric)

        # _____ PLAN VS ODOM XY _____
        plot_save_path = os.path.join(benchmark_path, 'path_comparison.png')
        logger.info('Creating path comparison plot: %s', plot_save_path)
        plt.figure()
        plt.plot(
            metrics_list[0].path_xy[:, 0],
            metrics_list[0].path_xy[:, 1], label='path', color='green')
        map_name = metrics_list[0].map_name
        mapdata = cb.mapdata_dict[map_name]
        for metric in metrics_list:
            plt.plot(metric.odom_xy[:, 0], metric.odom_xy[:, 1], label=metric.uid)

        # plt.title('Path comparison')
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.grid(visible=True, which='both', axis='both')
        # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.savefig(plot_save_path, bbox_inches='tight', dpi=500)
        plt.close()

        logger.info('___ Calculating metrics for benchmark %s ___', benchmark)
        # _____ DISTANCE TO GOAL _____
        distances = np.array([metric.distance_to_goal for metric in metrics_list])
        benchmark_output_dict.update({
            'distance_to_goal_avg': np.mean(distances),
            'distance_to_goal_std': np.std(distances)
        })

        # _____ ANGLE TO GOAL _____
        angles = np.array([metric.angle_to_goal for metric in metrics_list])
        benchmark_output_dict.update({
            'angle_to_goal_avg': np.mean(angles),
            'angle_to_goal_std': np.std(angles)
        })

        # _____ LINEAR VELOCITY _____
        linear_velocities = np.concatenate([metric.linear_velocity for metric in metrics_list])
        maximums = np.array([metric.max_linear_velocity for metric in metrics_list])
        rmss = np.array([metric.rms_linear_velocity for metric in metrics_list])
        benchmark_output_dict.update({
            'linear_velocity_avg': np.mean(linear_velocities),
            'linear_velocity_std': np.std(linear_velocities),
            'linear_velocity_max': maximums,
            'linear_velocity_max_avg': np.mean(maximums),
            'linear_velocity_max_std': np.std(maximums),
            'linear_velocity_rms': rmss,
            'linear_velocity_rms_avg': np.mean(rmss),
        })

        # _____ LINEAR ACCELERATION _____
        linear_accelerations = np.concatenate(
            [metric.linear_acceleration for metric in metrics_list])
        maximums = np.array([metric.max_linear_acceleration for metric in metrics_list])
        rmss = np.array([metric.rms_linear_acceleration for metric in metrics_list])
        benchmark_output_dict.update({
            'linear_acc_avg': np.mean(linear_accelerations),
            'linear_acc_std': np.std(linear_accelerations),
            'linear_acc_max': maximums,
            'linear_acc_max_avg': np.mean(maximums),
            'linear_acc_max_std': np.std(maximums),
            'linear_acc_rms': rmss,
            'linear_acc_rms_avg': np.mean(rmss),
        })

        # _____ ANGULAR ACCELERATION _____
        maximums = np.array([metric.max_angular_acceleration for metric in metrics_list])
        benchmark_output_dict.update({
            'angular_acceleration_maximums': maximums
        })

        # _____ LINEAR JERK _____
        linear_jerks = np.concatenate([metric.linear_jerks for metric in metrics_list])
        rms_jerks = np.array([metric.rms_linear_jerk for metric in metrics_list])
        benchmark_output_dict.update({
            'linear_jerk_avg': np.mean(linear_jerks),
            'linear_jerk_std': np.std(linear_jerks),
            'rms_linear_jerk': rms_jerks,
            'rms_linear_jerk_avg': np.mean(rms_jerks),
        })

        # _____ ANGULAR JERK _____
        angular_jerks = np.concatenate([metric.angular_jerks for metric in metrics_list])
        rms_jerks = np.array([metric.rms_angular_jerk for metric in metrics_list])
        benchmark_output_dict.update({
            'angular_jerk_avg': np.mean(angular_jerks),
            'angular_jerk_std': np.std(angular_jerks),
            'rms_angular_jerk': rms_jerks,
            'rms_angular_jerk_avg': np.mean(rms_jerks),
        })

        # _____ COSTS _____
        costs = np.concatenate([metric.path_costs for metric in metrics_list])
        benchmark_output_dict.update({
            'costs_avg': np.mean(costs),
            'costs_std': np.std(costs),
        })

        output_dict[benchmark] = benchmark_output_dict
        logger.info('___ Finished benchmark %s ___', benchmark)
        logger.info('_' * 20)

    logger.info('Finished evaluation')

    # make output dict saveable, convert numpy arrays to lists
    output_dict = numpy_dict_to_list(output_dict)

    # Save output dict to file
    stamp = cb.get_timestamp()
    output_path = os.path.join(WORK_DIR, f'evaluation_output{stamp}.yaml')
    with open(output_path, 'w', encoding='utf-8') as file:
        yaml.safe_dump(output_dict, file, default_flow_style=None, sort_keys=False)

    output_csv = os.path.join(WORK_DIR, f'evaluation_output{stamp}.csv')

    for benchmark, data in output_dict.items():
        keys = list(data.keys())
        for key in keys:
            if isinstance(data[key], list):
                _ = data.pop(key)

        header = True if not os.path.exists(output_csv) else False
        pd.DataFrame([{'name': benchmark, **data}]).to_csv(
            output_csv, mode='a', header=header, index=False)


if __name__ == '__main__':
    main()
