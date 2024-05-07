#! /usr/bin/env python3

# Common modules
import os
import glob
import pickle
import yaml
from typing import Dict, List
import matplotlib.pyplot as plt
import numpy as np
from numpy import linalg
from tabulate import tabulate

# Ros related modules
from rclpy import logging
from ament_index_python.packages import get_package_share_directory
from rclpy.time import Time

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped

from utils import ControllerResult, ControllerMetric

this_package_dir = get_package_share_directory('controller_benchmark')
map_file = os.path.join(this_package_dir, '10by10_empty.yaml')
config_file = os.path.join(this_package_dir, 'run_test_config.yaml')
params = yaml.safe_load(open(config_file))

params['output_dir'] = os.path.join(
    os.path.dirname(__file__), params['output_dir'])

logger = logging.get_logger('controller_benchmark')


def dist_2d(p1, p2) -> float:
    # if p1 nad p2 are PoseStamped
    if isinstance(p1, PoseStamped) and isinstance(p2, PoseStamped):
        return linalg.norm(
            np.array([p1.pose.position.x,
                      p1.pose.position.y]) -
            np.array([p2.pose.position.x,
                      p2.pose.position.y]))
    elif isinstance(p1, PoseWithCovarianceStamped) and isinstance(p2, PoseWithCovarianceStamped):
        return linalg.norm(
            np.array([p1.pose.pose.position.x,
                      p1.pose.pose.position.y]) -
            np.array([p2.pose.pose.position.x,
                      p2.pose.pose.position.y]))


def time_diff(t1: Time, t2: Time) -> float:
    return np.abs(
        Time.from_msg(t1).nanoseconds - Time.from_msg(t2).nanoseconds) / 1e9


def dist_fullplan(plan: List) -> float:
    length = 0.0
    p_last = plan[0]
    for i in range(1, len(plan)):
        p = plan[i]
        length += dist_2d(p_last, p)
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
    controller_metrics: List[ControllerMetric] = []
    for result in controller_results:
        distance_to_goal = dist_2d(
            result.plan.poses[-1], result.poses[-1])

        time = time_diff(
            result.twists[0].header.stamp, result.twists[-1].header.stamp)

        plan_length = dist_fullplan(result.plan.poses)
        traversed_length = dist_fullplan(result.poses)
        completion_ratio = (traversed_length - plan_length) / plan_length

        dt = [Time.from_msg(t.header.stamp) for t in result.twists]
        dt = [t - dt[0] for t in dt]
        vel_lin = [v.twist.linear.x for v in result.twists]
        acc_lin = np.gradient(vel_lin, dt)
        jerk_lin = np.gradient(vel_lin, dt)

        vel_ang = [v.twist.angular.z for v in result.twists]
        acc_ang = np.gradient(vel_ang, dt)
        jerk_ang = np.gradient(vel_ang, dt)

        metric = ControllerMetric(
            controller=result.controller_name,
            plan_idx=result.plan_idx,
            result=result.result,
            distance_to_goal=distance_to_goal,
            time=time,
            plan_length=plan_length,
            traversed_length=traversed_length,
            completion_ratio=completion_ratio,
            avg_linear_vel=np.mean(vel_lin),
            avg_linear_acc=np.mean(acc_lin),
            ms_linear_jerk=np.sqrt(np.mean(jerk_lin**2)),
            avg_angular_vel=np.mean(vel_ang),
            avg_angular_acc=np.mean(acc_ang),
            ms_angular_jerk=np.sqrt(np.mean(jerk_ang**2))
        )

        controller_metrics.append(metric)

    # separating metrics by controller
    single_controller_metrics: Dict[str, List[ControllerMetric]] = {}
    for metric in controller_metrics:
        if metric.controller_name not in single_controller_metrics:
            single_controller_metrics[metric.controller_name] = []
        single_controller_metrics[metric.controller_name].append(metric)

    table = [[
        'Controller', 'Succes rate', 'Distance to goal [m]', 'Time [s]'
        'Avarage lin speed [m/s]', 'Avarage ang speed [m/s]',
        'Avarage lin acc [m/s^2]', 'Avarage ang acc [rad/s^2]',
        'Avarage lin jerk rms [m/s^3]', 'Avarage ang jerk rms [rad/s^3]',
    ]]
    n = len(controller_metrics)
    for controller, controller_metrics in single_controller_metrics.items():
        success_rate = sum(
            1 for m in controller_metrics if m.result is True) / n
        d_to_goal = sum(
            m.distance_to_goal for m in controller_metrics) / n
        time = sum(m.time for m in controller_metrics) / n
        avg_lin_vel = sum(
            [m.avg_linear_vel for m in controller_metrics]) / n
        avg_ang_vel = sum(
            [m.avg_angular_vel for m in controller_metrics]) / n
        avg_lin_acc = sum(
            [m.avg_linear_acc for m in controller_metrics]) / n
        avg_ang_acc = sum(
            [m.avg_angular_acc for m in controller_metrics]) / n
        avg_lin_jerk = sum(
            [m.ms_linear_jerk for m in controller_metrics]) / n
        avg_ang_jerk = sum(
            [m.ms_angular_jerk for m in controller_metrics]) / n

        table.append([
            controller, success_rate, d_to_goal, time,
            avg_lin_vel, avg_ang_vel,
            avg_lin_acc, avg_ang_acc,
            avg_lin_jerk, avg_ang_jerk])

    logger.info('\n' + tabulate(
        np.array(table).T.tolist(),
        headers="firstrow", showindex="always", floatfmt=".5f", tablefmt="github"))


if __name__ == '__main__':
    main()
