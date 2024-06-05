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
from concurrent.futures import ProcessPoolExecutor, as_completed


# ROS related modules
from rclpy import logging
from ament_index_python.packages import get_package_share_directory
from rclpy.time import Time

# ROS message types
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from discrete import FastDiscreteFrechetMatrix, euclidean as euclidean_dist
from utils import ControllerResult, ControllerMetric, measure_resource_usage

this_package_dir = get_package_share_directory('controller_benchmark')
script_dir = os.path.dirname(__file__)

map_file = os.path.join(this_package_dir, '10by10_empty.yaml')
config_file = os.path.join(os.path.dirname(script_dir), 'config', 'run_test_config.yaml')
params = yaml.safe_load(open(config_file))
params['output_dir'] = os.path.join(script_dir, params['output_dir'])

logger = logging.get_logger('controller_benchmark')
fdfm: FastDiscreteFrechetMatrix = None


def get_xy(p) -> np.ndarray:
    if isinstance(p, PoseStamped):
        return np.array([p.pose.position.x, p.pose.position.y])
    elif isinstance(p, PoseWithCovarianceStamped):
        return np.array([p.pose.pose.position.x, p.pose.pose.position.y])
    elif isinstance(p, Odometry):
        return np.array([p.pose.pose.position.x, p.pose.pose.position.y])
    else:
        raise ValueError('Unknown type')


def time_diff(t1: Time, t2: Time) -> float:
    return np.abs(
        Time.from_msg(t1).nanoseconds - Time.from_msg(t2).nanoseconds) / 1e9


def dist_fullplan(plan: list) -> float:
    if len(plan) < 2:
        return 0.0
    xy_positions = np.array([get_xy(p) for p in plan])
    deltas = np.diff(xy_positions, axis=0)
    return np.sum(np.linalg.norm(deltas, axis=1))


@measure_resource_usage
def calc_frechet_distance(plan, route):
    global fdfdm
    return fdfdm.distance(plan, route)


# def reject_outliers(data, m=2.0):
#     d = np.abs(data - np.median(data))
#     mdev = np.median(d)
#     s = d/mdev if mdev else np.zeros(len(d))
#     return data[s < m]

def newton_diff(y, dt):
    derivative = np.zeros_like(y)
    # Central differences
    derivative[2:-2] = (-y[4:] + 8*y[3:-1] - 8*y[1:-3] + y[0:-4]) / (12 * dt)
    # Use lower-order methods at the boundaries
    derivative[0] = (y[1] - y[0]) / dt  # Forward difference for the first point
    derivative[1] = (y[2] - y[0]) / (2 * dt)  # Central difference for the second point
    derivative[-2] = (y[-1] - y[-3]) / (2 * dt)  # Central difference for the second last point
    derivative[-1] = (y[-1] - y[-2]) / dt  # Backward difference for the last point
    return derivative


def reject_outliers(data, m=0.6745, axis=None):
    median = np.median(data, axis=axis)
    deviation = np.abs(data - median)
    median_deviation = np.median(deviation, axis=axis)
    modified_z_score = m * deviation / median_deviation
    return data[np.any(modified_z_score > m, axis=axis)]


def calculate_metrics(result: ControllerResult) -> ControllerMetric:
    global fdfdm

    distance_to_goal = linalg.norm(get_xy(result.plan.poses[-1]) - get_xy(result.odom[-1]))
    time = time_diff(result.cmd_vel[0].header.stamp, result.cmd_vel[-1].header.stamp)
    plan_length = dist_fullplan(result.plan.poses)
    traversed_length = dist_fullplan(result.odom)
    completion_ratio = (traversed_length - plan_length) / plan_length

    dt = np.array([Time.from_msg(t.header.stamp).nanoseconds * 1e-9 for t in result.cmd_vel])
    dt_array = dt - dt[0]
    dt = np.mean(np.diff(dt_array))  # TODO: make dt stay an array differences are big
    #  0.05612588 0.0438931  0.0548203  0.04536557 0.05174136 0.05203557
    vel_lin = np.array([v.twist.linear.x for v in result.cmd_vel])
    vel_ang = np.array([v.twist.angular.z for v in result.cmd_vel])

    acc_lin = newton_diff(vel_lin, dt)
    jerk_lin = newton_diff(acc_lin, dt)

    acc_ang = newton_diff(vel_ang, dt)
    jerk_ang = newton_diff(acc_ang, dt)

    # fdfdm = FastDiscreteFrechetMatrix(euclidean_dist)
    plan = np.array([get_xy(p) for p in result.plan.poses])
    route = np.array([get_xy(p) for p in result.odom])
    print(f"calculating frechet dist for {result.controller_name}")
    frechet_dist = calc_frechet_distance(plan, route)
    print(f"frechet dist for {result.controller_name} is {frechet_dist}")

    if result.critic_scores is not None:
        critic_scores_: Dict[str, List[float]] = {}
        for cs in result.critic_scores:
            for name, score in zip(cs.critic_names, cs.critic_scores):
                if name.data not in critic_scores_:
                    critic_scores_[name.data] = []
                critic_scores_[name.data].append(score.data)
    else:
        critic_scores_ = None

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
        time_steps=dt_array,
        linear_acc=acc_lin,
        linear_jerks=jerk_lin,
        angular_acc=acc_ang,
        angular_jerks=jerk_ang,
        critic_scores=critic_scores_,
    )
    return metric


def main():
    global fdfdm
    fdfdm = FastDiscreteFrechetMatrix(euclidean_dist)

    logger.info('Plotting results...')
    logger.info('Reading results from file')

    # Get the latest file in the directory
    result_files = glob.glob(os.path.join(params['output_dir'] + '/*.pickle'))
    latest_file = max(result_files, key=os.path.getctime)
    logger.info(f'Using file: {latest_file}')

    with open(latest_file, 'rb') as file:
        controller_results: List[ControllerResult] = pickle.load(file)

    logger.info('Collecting metrics from results')
    controller_metrics: List[ControllerMetric] = []
    # fdfdm = FastDiscreteFrechetMatrix(euclidean_dist)

    with ProcessPoolExecutor() as executor:
        futures = {executor.submit(calculate_metrics, result): result for result in controller_results}
        for future in as_completed(futures):
            result = futures[future]
            try:
                metric = future.result()
                if metric:
                    controller_metrics.append(metric)
                    logger.info(f'{metric.controller_name} finished')
            except Exception as e:
                logger.error(f'Error processing {result.controller_name}: {e}')

    logger.info('Creating plots')
    fig, ((ax_plan, ax_jerk_x), (ax_critics, ax_jerk_theta)) = plt.subplots(ncols=2, nrows=2, sharey=False, sharex=False)
    fig.suptitle('Controller metrics')

    ax_plan.set_title('Plan vs Route')
    ax_plan.set_xlabel('x [m]')
    ax_plan.set_ylabel('y [m]')
    ax_plan.plot(
        controller_metrics[0].plan_poses[:, 0],
        controller_metrics[0].plan_poses[:, 1], label='Plan', color='g')

    ax_jerk_x.set_title('RMS jerk x [m/s^3]')
    ax_jerk_x.set_xlabel('Time [s]')
    ax_jerk_x.set_ylabel('Jerk [m/s^3]')

    ax_jerk_theta.set_title('RMS jerk theta [rad/s^3]')
    ax_jerk_theta.set_xlabel('Time [s]')
    ax_jerk_theta.set_ylabel('Jerk [rad/s^3]')

    ax_critics.set_title('Critic scores')
    ax_critics.set_xlabel('Time [s]')
    ax_critics.set_ylabel('Score')

    for metric in controller_metrics:
        label = f'{metric.controller_name} - frechet: {metric.frechet_dist:.2f} [m]'
        ax_plan.plot(metric.route_poses[:, 0], metric.route_poses[:, 1], label=label)
        ax_jerk_x.plot(metric.time_steps, metric.linear_jerks, label=f"{metric.controller_name} lin jerk")
        ax_jerk_x.scatter(metric.time_steps[-1], metric.linear_jerks[-1], marker='x', color='red', zorder=2)
        ax_jerk_x.plot(metric.time_steps, metric.linear_acc, label=f"{metric.controller_name} lin acc")

        ax_jerk_theta.plot(metric.time_steps, metric.angular_jerks, label=f"{metric.controller_name} ang jerk")
        ax_jerk_theta.scatter(metric.time_steps[-1], metric.angular_jerks[-1], marker='x', color='red', zorder=2)
        ax_jerk_theta.plot(metric.time_steps, metric.angular_acc, label=f"{metric.controller_name} ang acc")

        if metric.critic_scores is not None and metric.controller_name == "Maneuver":
            score_times = np.linspace(0, metric.time, len(next(iter(metric.critic_scores.values()))))
            for name, scores in metric.critic_scores.items():
                ax_critics.plot(score_times, scores, label=f"{metric.controller_name} {name}")

    ax_plan.grid(visible=True, which='both', axis='both')
    ax_plan.legend()

    ax_jerk_x.grid(visible=True, which='both', axis='both')
    ax_jerk_x.legend()

    ax_jerk_theta.grid(visible=True, which='both', axis='both')
    ax_jerk_theta.legend()

    ax_critics.grid(visible=True, which='both', axis='both')
    ax_critics.legend()

    # separating metrics by controller
    single_controller_metrics: Dict[str, List[ControllerMetric]] = {}
    for metric in controller_metrics:
        if metric.controller_name not in single_controller_metrics:
            single_controller_metrics[metric.controller_name] = []
        single_controller_metrics[metric.controller_name].append(metric)

    # collecting metrics
    logger.info('Collecting metrics from results')
    metrics_summary = {
        'Metrics': [
            'Succes rate', 'Distance to goal [m]', 'Time [s]', 'Frechet dist [m]',
            'Avarage lin speed [m/s]', 'Avarage ang speed [m/s]',
            'Avarage lin acc [m/s^2]',  # 'Avarage ang acc [rad/s^2]',
            'Avarage lin jerk rms [m/s^3]', 'Avarage ang jerk rms [rad/s^3]']}
    df = pd.DataFrame(metrics_summary)

    results_str = 'Results: '
    for controller, metrics in single_controller_metrics.items():
        n = len(metrics)
        success_rate = sum(1 for m in metrics if m.result is True) / n
        d_to_goal = sum(m.distance_to_goal for m in metrics) / n
        time = sum(m.time for m in metrics) / n
        frechet_dist = sum(m.frechet_dist for m in metrics) / n
        avg_lin_vel = sum(m.avg_linear_vel for m in metrics) / n
        avg_ang_vel = sum(m.avg_angular_vel for m in metrics) / n
        avg_lin_acc = sum(m.avg_linear_acc for m in metrics) / n
        # avg_ang_acc = sum(m.avg_angular_acc for m in metrics) / n
        avg_lin_jerk = sum(m.ms_linear_jerk for m in metrics) / n
        avg_ang_jerk = sum(m.ms_angular_jerk for m in metrics) / n

        results_str += f'{controller}: {n} '
        df[controller] = [
            success_rate, d_to_goal, time, frechet_dist,
            avg_lin_vel, avg_ang_vel,
            avg_lin_acc,  # avg_ang_acc,
            avg_lin_jerk, avg_ang_jerk]

    logger.info('\n' + tabulate(
        df, stralign="right",
        headers="keys", showindex="always", floatfmt=".5f", tablefmt="github"))

    plt.show()


if __name__ == '__main__':
    main()
