#! /usr/bin/env python3
import sys
sys.path.append("/home/turtlewizard/thesis-mppi-model-ident/workspace/install/controller_benchmark/lib/controller_benchmark")
# print(sys.path)

# Common modules
import os
import glob
import pickle
import yaml
from typing import Dict, List
import matplotlib.pyplot as plt
import numpy as np
from numpy import linalg
import pandas as pd
from tabulate import tabulate

# Ros related modules
from rclpy import logging
from ament_index_python.packages import get_package_share_directory
from rclpy.time import Time

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from discrete import FastDiscreteFrechetMatrix, euclidean as euclidean_dist
from utils import ControllerResult, ControllerMetric

this_package_dir = get_package_share_directory('controller_benchmark')
map_file = os.path.join(this_package_dir, '10by10_empty.yaml')
config_file = os.path.join(
    os.path.dirname(os.path.dirname(__file__)), 'config', 'run_test_config.yaml')
params = yaml.safe_load(open(config_file))
params['output_dir'] = os.path.join(
    os.path.dirname(__file__), params['output_dir'])

logger = logging.get_logger('controller_benchmark')


def get_xy(p) -> np.ndarray:
    if isinstance(p, PoseStamped):
        return np.array([p.pose.position.x, p.pose.position.y])
    elif isinstance(p, PoseWithCovarianceStamped):
        return np.array([p.pose.pose.position.x, p.pose.pose.position.y])
    elif isinstance(p, Odometry):
        return np.array([p.pose.pose.position.x, p.pose.pose.position.y])


def time_diff(t1: Time, t2: Time) -> float:
    return np.abs(
        Time.from_msg(t1).nanoseconds - Time.from_msg(t2).nanoseconds) / 1e9


def dist_fullplan(plan: List) -> float:
    length = 0.0
    p_last = plan[0]
    for i in range(1, len(plan)):
        p = plan[i]
        length += linalg.norm(get_xy(p_last) - get_xy(p))
        p_last = p
    return length


def main():
    logger.info('Plotting results...')

    logger.info('Reading results from file')
    # filename = '/controller_benchmark_results.pickle'
    # Get the latest file in the directory
    list_of_result_files = glob.glob(params['output_dir'] + '/*.pickle')
    latest_file = max(list_of_result_files, key=os.path.getctime)
    logger.info(f'Using file: {latest_file}')

    with open(latest_file, 'rb') as file:
        controller_results: List[ControllerResult] = pickle.load(file)

    # collecting meterics
    logger.info('Collecting metrics from results')
    controller_metrics: List[ControllerMetric] = []
    for result in controller_results:
        distance_to_goal = linalg.norm(
            get_xy(result.plan.poses[-1]) - get_xy(result.odom[-1]))

        time = time_diff(
            result.cmd_vel[0].header.stamp, result.cmd_vel[-1].header.stamp)

        plan_length = dist_fullplan(result.plan.poses)
        traversed_length = dist_fullplan(result.odom)
        completion_ratio = (traversed_length - plan_length) / plan_length

        dt = [Time.from_msg(t.header.stamp).nanoseconds * 1e-9 for t in result.cmd_vel]
        dt = [t - dt[0] for t in dt]
        vel_lin = [v.twist.linear.x for v in result.cmd_vel]
        acc_lin = np.gradient(vel_lin, dt)
        jerk_lin = np.gradient(vel_lin, dt)

        vel_ang = [v.twist.angular.z for v in result.cmd_vel]
        acc_ang = np.gradient(vel_ang, dt)
        jerk_ang = np.gradient(vel_ang, dt)

        logger.info(f'{result.controller_name} - calculating frechet dist')
        fdfdm = FastDiscreteFrechetMatrix(euclidean_dist)
        plan = np.array([get_xy(p) for p in result.plan.poses])
        route = np.array([get_xy(p) for p in result.odom])
        frechet_dist = fdfdm.distance(plan, route)

        metric = ControllerMetric(
            controller_name=result.controller_name,
            plan_idx=result.plan_idx,
            result=result.result,
            distance_to_goal=distance_to_goal,
            time=time,
            plan_length=plan_length,
            traversed_length=traversed_length,
            completion_ratio=completion_ratio,
            frechet_dist=frechet_dist,
            avg_linear_vel=np.mean(vel_lin),
            avg_linear_acc=np.mean(acc_lin),
            ms_linear_jerk=np.sqrt(np.mean(jerk_lin**2)),
            avg_angular_vel=np.mean(vel_ang),
            avg_angular_acc=np.mean(acc_ang),
            ms_angular_jerk=np.sqrt(np.mean(jerk_ang**2)),
            plan_poses=plan,
            route_poses=route,
            time_steps=dt,
            linear_jerks=jerk_lin,  # TODO make it squared?
            angular_jerks=jerk_ang
        )

        controller_metrics.append(metric)
        print(f'{metric.controller_name} finished')

    # plotting
    logger.info('Creating plots')
    fig = plt.figure()
    fig.suptitle('Controller metrics')
    nrows = 1
    ncols = 3
    ax_plan = fig.add_subplot(nrows, ncols, 1)
    ax_plan.set_title('Plan vs Route')
    ax_plan.set_xlabel('x [m]')
    ax_plan.set_ylabel('y [m]')
    ax_plan.plot(
        controller_metrics[0].plan_poses[:, 0],
        controller_metrics[0].plan_poses[:, 1], label='Plan', color='g')

    ax_jerk_x = fig.add_subplot(nrows, ncols, 2)
    ax_jerk_x.set_title('RMS jerk x [m/s^3]')
    ax_jerk_x.set_xlabel('Time [s]')
    ax_jerk_x.set_ylabel('Jerk [m/s^3]')

    ax_jerk_theta = fig.add_subplot(nrows, ncols, 3)
    ax_jerk_theta.set_title('RMS jerk theta [rad/s^3]')
    ax_jerk_theta.set_xlabel('Time [s]')
    ax_jerk_theta.set_ylabel('Jerk [rad/s^3]')

    for metric in controller_metrics:
        label = f'{metric.controller_name} - frechet: {metric.frechet_dist:.2f} [m]'
        ax_plan.plot(
            metric.route_poses[:, 0],
            metric.route_poses[:, 1],
            label=label)

        label = f'{metric.controller_name}'
        ax_jerk_x.plot(metric.time_steps, metric.linear_jerks, label=label)
        ax_jerk_theta.plot(metric.time_steps, metric.angular_jerks, label=label)

    ax_plan.grid(visible=True, which='both', axis='both')
    ax_plan.legend()

    ax_jerk_x.grid(visible=True, which='both', axis='both')
    ax_jerk_x.legend()

    ax_jerk_theta.grid(visible=True, which='both', axis='both')
    ax_jerk_theta.legend()

    # separating metrics by controller
    single_controller_metrics: Dict[str, List[ControllerMetric]] = {}
    for metric in controller_metrics:
        if metric.controller_name not in single_controller_metrics:
            single_controller_metrics[metric.controller_name] = []
        single_controller_metrics[metric.controller_name].append(metric)

    # collecting metrics
    logger.info('Collecting metrics from results')
    table = {'Metrics': [
        'Succes rate', 'Distance to goal [m]', 'Time [s]', 'Frechet distance [m]',
        'Avarage lin speed [m/s]', 'Avarage ang speed [m/s]',
        'Avarage lin acc [m/s^2]',  # 'Avarage ang acc [rad/s^2]',
        'Avarage lin jerk rms [m/s^3]', 'Avarage ang jerk rms [rad/s^3]',
    ]}
    df = pd.DataFrame(table)
    results_str = 'Results: '
    for controller, controller_metrics in single_controller_metrics.items():
        n = len(controller_metrics)

        success_rate = sum(
            1 for m in controller_metrics if m.result is True) / n
        d_to_goal = sum(
            m.distance_to_goal for m in controller_metrics) / n
        time = sum(m.time for m in controller_metrics) / n
        frechet_dist = sum(m.frechet_dist for m in controller_metrics) / n
        avg_lin_vel = sum(
            m.avg_linear_vel for m in controller_metrics) / n
        avg_ang_vel = sum(
            m.avg_angular_vel for m in controller_metrics) / n
        avg_lin_acc = sum(
            m.avg_linear_acc for m in controller_metrics) / n
        # avg_ang_acc = sum(
        #     m.avg_angular_acc for m in controller_metrics) / n
        avg_lin_jerk = sum(
            m.ms_linear_jerk for m in controller_metrics) / n
        avg_ang_jerk = sum(
            m.ms_angular_jerk for m in controller_metrics) / n

        results_str += f'{controller}: {n} '
        df[controller] = [
            success_rate, d_to_goal, time, frechet_dist,
            avg_lin_vel, avg_ang_vel,
            avg_lin_acc,  # avg_ang_acc,
            avg_lin_jerk, avg_ang_jerk]

    logger.info(results_str)
    logger.info('\n' + tabulate(
        df, stralign="right",
        headers="keys", showindex="always", floatfmt=".5f", tablefmt="github"))

    plt.show()


if __name__ == '__main__':
    main()
