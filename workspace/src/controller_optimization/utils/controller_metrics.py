#! /usr/bin/env python3

# common imports
from dataclasses import dataclass, field
import numpy as np
# from typing import Dict


@dataclass
class ControllerMetric:
    # represent a result of a parametrized controller run on a single plan/map
    controller_name: str
    map_name: str
    uid: str = ''  # unique identifier, could be a timestamp, id or params

    time_elapsed: float = 0.0  # nanoseconds
    success: bool = False  # if the controller reached the goal approximately
    distance_to_goal: float = 0.0  # [m] stopping distance from the goal

    # linear velocity metrics
    linear_velocity: np.ndarray = field(
        default_factory=lambda: np.empty((0, 1)))  # [m/s] velocity
    avg_linear_velocity: float = 0.0  # [m/s] average linear velocity
    max_linear_velocity: float = 0.0  # [m/s] maximum linear velocity
    ms_linear_velocity: float = 0.0  # [m/s] mean squared linear velocity

    # linear acceleration metrics
    linear_acceleration: np.ndarray = field(
        default_factory=lambda: np.empty((0, 1)))  # [m/s^2] acceleration
    avg_linear_acceleration: float = 0.0  # [m/s^2] average linear acceleration
    max_linear_acceleration: float = 0.0  # [m/s^2] maximum linear acceleration
    ms_linear_acceleration: float = 0.0  # [m/s^2] mean squared linear acceleration

    # angular acceleration metrics
    max_angular_acceleration: float = 0.0  # [rad/s^2] maximum angular acceleration
    avg_angular_acceleration: float = 0.0  # [rad/s^2] average angular acceleration

    # jerk metrics
    linear_jerks: np.ndarray = field(
        default_factory=lambda: np.empty((0, 1)))  # [m/s^3] jerk
    ms_linear_jerk: float = 0.0  # [m/s^3] mean squared linear jerk

    angular_jerks: np.ndarray = field(
        default_factory=lambda: np.empty((0, 1)))  # [rad/s^3] angular jerk
    ms_angular_jerk: float = 0.0  # [rad/s^3] mean squared angular jerk

    def to_table_string(self) -> str:
        # Create a header and rows with values aligned
        table = (
            f"{'Attribute':<30} | {'Value'}\n"
            f"{'-' * 44}\n"
            f"{'Maximum Linear Velocity':<30} | {self.max_linear_velocity} m/s\n"
            f"{'Average Linear Velocity':<30} | {self.avg_linear_velocity} m/s\n"
            f"{'Maximum Linear Acceleration':<30} | {self.max_linear_acceleration} m/s²\n"
            f"{'Average Linear Acceleration':<30} | {self.avg_linear_acceleration} m/s²\n"
            f"{'Maximum Angular Acceleration':<30} | {self.max_angular_acceleration} rad/s²\n"
            f"{'Average Angular Acceleration':<30} | {self.avg_angular_acceleration} rad/s²\n"
            f"{'Root Mean Squared Linear Jerk':<30} | {self.ms_linear_jerk} m/s³\n"
            f"{'Root Mean Squared Angular Jerk':<30} | {self.ms_angular_jerk} rad/s³\n"
        )
        return table
