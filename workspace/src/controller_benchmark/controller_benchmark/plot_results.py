#! /usr/bin/env python3

# Common modules
import os
import pickle
from typing import Dict, List
import matplotlib.pyplot as plt
import numpy as np
from numpy import linalg

# Ros related modules
from rclpy import logging
from rclpy.time import Time

from controller_benchmark.utils import ControllerResult, ControllerMetric


logger = logging.get_logger('controller_benchmark')


def main():
    logger.info('Plotting results...')

    logger.info('Reading results from file')
    filename = '/controller_benchmark_results.pickle'
    with open(os.getcwd() + filename, 'rb') as file:
        controller_results: List[ControllerResult] = pickle.load(file)

    # separating results by plan
    plan_results: Dict[int, List] = {}
    for result in controller_results:
        if result.plan_idx not in plan_results:
            plan_results[result.plan_idx] = []
        plan_results[result.plan_idx].append(result)

    # print how many results each plan has
    for plan_idx, results in plan_results.items():
        logger.info(f'Plan {plan_idx} has {len(results)} results')

    logger.info('Collecting metrics from results')
    # get data for calculated metrics / plan:
    # - avarage linear velocity in x
    # - avarage angular velocity in theta
    # - length of generated plan
    # - length of controller rout
    # (- avarage cost / min distance from obstacles)
    # - avarage integrated linear jerk
    # - avarage integrated angular jerk
    controller_metrics: List[ControllerMetric] = []
    for result in controller_results:
        # calculate original plans length
        generated_plan_length = 0.0
        p_last = np.array([
            result.plan.poses[0].pose.position.x,
            result.plan.poses[0].pose.position.y])
        for i in range(1, len(result.plan.poses)):
            p = np.array([
                result.plan.poses[i].pose.position.x,
                result.plan.poses[i].pose.position.y])
            generated_plan_length += linalg.norm(p - p_last)

        # calculating the traveled distance
        traveled_plan_length = 0.0
        p_last = np.array([
            result.poses[0].pose.pose.position.x,
            result.poses[0].pose.pose.position.y])
        for i in range(1, len(result.plan.poses)):
            p = np.array([
                result.poses[i].pose.pose.position.x,
                result.poses[i].pose.pose.position.y])
            traveled_plan_length += linalg.norm(p - p_last)

        dt_odom = [Time.from_msg(i.header.stamp).nanoseconds *
                   1e-9 for i in result.poses]
        dt_odom = [t - dt_odom[0] for t in dt_odom]

        dt_twist = [Time.from_msg(i.header.stamp).nanoseconds *
                    1e-9 for i in result.twists]
        dt_twist = [t - dt_twist[0] for t in dt_twist]

        # getting velocitties
        vel_linear = [ts.twist.linear.x for ts in result.twists]
        vel_angular = [ts.twist.angular.z for ts in result.twists]

        # getting time steps
        t_end = Time.from_msg(result.twists[-1].header.stamp).nanoseconds
        t_start = Time.from_msg(result.twists[0].header.stamp).nanoseconds
        dt = (t_end - t_start) / len(result.twists)

        # calculating mean squared jerk of velocities
        acc_linear = np.gradient(vel_linear, dt)
        jerk_linear = np.gradient(acc_linear, dt)

        acc_angular = np.gradient(vel_angular, dt)
        jerk_angular = np.gradient(acc_angular, dt)

        controller_metric = ControllerMetric(
            plan_idx=result.plan_idx,
            plan_length=generated_plan_length,
            plan=result.plan,
            controller_name=result.controller_name,
            length=traveled_plan_length,
            diff_from_plan=traveled_plan_length - generated_plan_length,
            x=[i.pose.pose.position.x for i in result.poses],
            y=[i.pose.pose.position.y for i in result.poses],
            dt_odom=dt_odom,
            dt_twist=dt_twist,
            lin_vel=vel_linear,
            lin_vel_avg=np.mean(vel_linear),
            lin_acc=acc_linear,
            lin_acc_avg=np.mean(acc_linear),
            lin_jerk=jerk_linear,
            lin_jerk_avg=np.mean(jerk_linear),
            lin_jerk_rms=np.sqrt(np.mean(jerk_linear**2)),
            ang_vel=vel_angular,
            ang_vel_avg=np.mean(vel_angular),
            ang_acc=acc_angular,
            ang_acc_avg=np.mean(acc_angular),
            ang_jerk=jerk_angular,
            ang_jerk_avg=np.mean(jerk_angular),
            ang_jerk_rms=np.sqrt(np.mean(jerk_angular**2)))

        controller_metrics.append(controller_metric)

    logger.info('Writing metrics')
    filename = '/controller_benchmark_metrics.pickle'
    with open(os.getcwd() + filename, 'wb+') as f:
        pickle.dump(controller_metrics, f, pickle.HIGHEST_PROTOCOL)

    # separating metrics by plan
    plan_metrics: Dict[int, List] = {}
    for metric in controller_metrics:
        if metric.plan_idx not in plan_metrics:
            plan_metrics[metric.plan_idx] = []
        plan_metrics[metric.plan_idx].append(metric)

    logger.info('Plotting results, metrics')
    # get data and plot results
    plan_figures = []
    for plan_idx, single_plan_metrics in plan_metrics.items():
        # calculated metrics
        fig = plt.figure(plan_idx)
        fig.suptitle(f'Plan {plan_idx}')

        # fig = plt.subplots(nrows=1, ncols=2)
        ax_plan = fig.add_subplot(131)
        ax_vel_x = fig.add_subplot(132)
        ax_vel_theta = fig.add_subplot(133)

        ax_plan.set_title('Global plan')
        ax_plan.set_xlabel('X [m]')
        ax_plan.set_ylabel('Y [m]')
        fig.text(.5, .05, 'txt', ha='center')

        x_plan, y_plan = [], []
        for pose_stamped in single_plan_metrics[0].plan.poses:
            x_plan.append(pose_stamped.pose.position.x)
            y_plan.append(pose_stamped.pose.position.y)
        ax_plan.plot(x_plan, y_plan, label='generated_plan')
        # ax_vel_x.legend()

        ax_vel_x.set_title('Linear velocities')
        ax_vel_x.set_xlabel('Time [s]')
        ax_vel_x.set_ylabel('Velocity [m/s]')

        ax_vel_theta.set_title('Angular velocities')
        ax_vel_theta.set_xlabel('Time [s]')
        ax_vel_theta.set_ylabel('Velocity [rad/s]')

        # plot measured data for each plan:
        # - velocity x theta vs time vs controller
        for metric in single_plan_metrics:
            ax_plan.plot(metric.x, metric.y, label=metric.controller_name)
            ax_plan.legend()

            ax_vel_x.plot(
                metric.dt_twist, metric.lin_vel, label=metric.controller_name)
            ax_vel_x.legend()

            ax_vel_theta.plot(
                metric.dt_twist, metric.ang_vel, label=metric.controller_name)
            ax_vel_theta.legend()

            plan_figures.append(fig)

        # fig.tight_layout()

    plt.show()


if __name__ == '__main__':
    main()