#! /usr/bin/env python3

from dataclasses import dataclass, field
import numpy as np


@dataclass
class ControllerMetric:
    # represent a result of a parametrized controller run on a single plan/map
    controller_name: str
    map_name: str
    uid: str = ''  # unique identifier, could be a timestamp, id or params

    # common properties from result needed for plotting
    time_elapsed: float = 0.0  # [s]
    success: bool = False  # if the controller reached the goal approximately
    status_msg: str = ''  # reason of fail or extra info
    path_xy: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    odom_xy: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    cmd_vel_t: np.ndarray = field(default_factory=lambda: np.empty((0, 1)))

    distance_to_goal: float = 0.0  # [m] stopping distance from the goal
    angle_to_goal: float = 0.0  # [rad] stopping angle difference from the goal

    # linear velocity metrics
    linear_velocity: np.ndarray = field(
        default_factory=lambda: np.empty((0, 1)))  # [m/s] velocity
    avg_linear_velocity: float = 0.0  # [m/s] average linear velocity
    max_linear_velocity: float = 0.0  # [m/s] maximum linear velocity
    rms_linear_velocity: float = 0.0  # [m/s] root mean squared linear velocity

    # linear acceleration metrics
    linear_acceleration: np.ndarray = field(
        default_factory=lambda: np.empty((0, 1)))  # [m/s^2] acceleration
    avg_linear_acceleration: float = 0.0  # [m/s^2] average linear acceleration
    max_linear_acceleration: float = 0.0  # [m/s^2] maximum linear acceleration
    rms_linear_acceleration: float = 0.0  # [m/s^2] root mean squared linear acceleration

    # angular acceleration metrics
    max_angular_acceleration: float = 0.0  # [rad/s^2] maximum angular acceleration
    avg_angular_acceleration: float = 0.0  # [rad/s^2] average angular acceleration

    # jerk metrics
    linear_jerks: np.ndarray = field(
        default_factory=lambda: np.empty((0, 1)))  # [m/s^3] jerk
    rms_linear_jerk: float = 0.0  # [m/s^3] root mean squared linear jerk

    angular_jerks: np.ndarray = field(
        default_factory=lambda: np.empty((0, 1)))  # [rad/s^3] angular jerk
    rms_angular_jerk: float = 0.0  # [rad/s^3] root mean squared angular jerk

    # costs
    path_costs: np.ndarray = field(  # cost of cells along the traversed path
        default_factory=lambda: np.empty((0, 1)))
    sum_of_costs: float = 0.0  # sum of all costs
    avg_cost: float = 0.0  # average cost
    rms_cost: float = 0.0  # root mean squared cost
