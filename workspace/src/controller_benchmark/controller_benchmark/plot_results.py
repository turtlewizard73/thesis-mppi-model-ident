#! /usr/bin/env python3

# Common modules
import os
import pickle
from typing import Dict, List
import matplotlib.pyplot as plt

# Ros related modules
from rclpy import logging
from rclpy.time import Time
# from controller_benchmark.utils import ControllerResult


from dataclasses import dataclass
from typing import List

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path, OccupancyGrid

@dataclass
class ControllerResult:
    plan_idx: int
    plan: Path
    controller_name: str
    start_time: float  # nanoseconds
    end_time: float  # nanoseconds
    result: bool
    poses: List[PoseStamped]
    twists: List[TwistStamped]
    costmaps: List[OccupancyGrid]


logger = logging.get_logger('controller_benchmark')


def plot_figures(figures, nrows=1, ncols=1):
    """Plot a dictionary of figures.

    Parameters
    ----------
    figures : <title, figure> dictionary
    ncols : number of columns of subplots wanted in the display
    nrows : number of rows of subplots wanted in the figure
    """

    fig, axeslist = plt.subplots(ncols=ncols, nrows=nrows)
    for idx, title in enumerate(figures):
        axeslist.ravel()[idx].imshow(figures[title], cmap=plt.gray())
        axeslist.ravel()[idx].set_title(title)
        axeslist.ravel()[idx].set_axis_off()
    plt.tight_layout() # optional


def main():
    logger.info('Plotting results...')
    logger.info('Reading results from file')

    filename = '/controller_benchmark_results.pickle'
    with open(os.getcwd() + filename, 'rb') as file:
        controller_results = pickle.load(file)

    # separating results by plan
    plan_results: Dict[int, List] = {}
    for result in controller_results:
        if result.plan_idx not in plan_results:
            plan_results[result.plan_idx] = []
        plan_results[result.plan_idx].append(result)

    # print how many results each plan has
    for plan_idx, results in plan_results.items():
        logger.info(f'Plan {plan_idx} has {len(results)} results')

    # plot results
    plan_figures = []
    for plan_idx, controller_results in plan_results.items():
        fig = plt.figure(plan_idx)
        fig.suptitle(f'Plan {plan_idx}')

        # fig = plt.subplots(nrows=1, ncols=2)
        ax_vel_x = fig.add_subplot(121)
        ax_vel_theta = fig.add_subplot(122)

        ax_vel_x.set_title('Linear velocities')
        ax_vel_x.set_xlabel('Time [s]')
        ax_vel_x.set_ylabel('Velocity [m/s]')
        ax_vel_theta.set_title('Angular velocities')
        ax_vel_theta.set_xlabel('Time [s]')
        ax_vel_theta.set_ylabel('Velocity [rad/s]')

        # plot measured data for each plan:
        # - velocity x theta vs time vs controller
        for idx, result in enumerate(controller_results):
            time = []
            vel_x = []
            vel_theta = []
            for twiststamped in result.twists:
                t_ = Time.from_msg(twiststamped.header.stamp)

                vel_x.append(twiststamped.twist.linear.x)
                vel_theta.append(twiststamped.twist.angular.z)
                time.append(t_.nanoseconds * 1e-9)

            time = [t - time[0] for t in time]

            ax_vel_x.plot(time, vel_x, label=result.controller_name)
            ax_vel_x.legend()
            ax_vel_theta.plot(time, vel_theta, label=result.controller_name)
            ax_vel_theta.legend()
            plan_figures.append(fig)

        fig.tight_layout()

    plt.show()


if __name__ == '__main__':
    main()
