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

    time_elapsed: float = 0.0  # nanoseconds
    success: bool = False  # if the controller reached the goal approximately

    max_linear_velocity: float = 0.0  # [m/s] maximum velocity
    avg_linear_velocity: float = 0.0  # [m/s] average velocity
    max_linear_acceleration: float = 0.0  # [m/s^2] maximum acceleration
    avg_linear_acceleration: float = 0.0  # [m/s^2] average acceleration
    max_angular_acceleration: float = 0.0  # [rad/s^2] maximum angular acceleration
    avg_angular_acceleration: float = 0.0  # [rad/s^2] average angular acceleration

    distance_to_goal: float = 0.0  # [m] stopping distance from the goal

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


# @dataclass
# class ControllerMetricOld:
#     controller_name: str
#     plan_idx: int

#     # success metrics
#     result: bool
#     distance_to_goal: float  # [m]
#     time: float  # [s]

#     # trajectory metrics
#     plan_length: float  # [m]
#     traversed_length: float  # [m]
#     completion_ratio: float
#     frechet_dist: float

#     # dynamic metrics
#     avg_linear_vel: float  # [m/s]
#     avg_linear_acc: float  # [m/s^2]
#     ms_linear_jerk: float  # [m/s^3]

#     avg_angular_vel: float  # [rad/s]
#     avg_angular_acc: float  # [rad/s^2]
#     ms_angular_jerk: float  # [rad/s^3]

#     # raw data in case of further analysis
#     plan_poses: np.ndarray
#     route_poses: np.ndarray

#     time_steps: List[float]
#     linear_acc: List[float]
#     linear_jerks: List[float]
#     angular_acc: List[float]
#     angular_jerks: List[float]

#     critic_scores: Dict[str, List[float]]
