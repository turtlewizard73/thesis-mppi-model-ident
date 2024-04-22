#! /usr/bin/env python3

from dataclasses import dataclass
from typing import List

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path, OccupancyGrid


@dataclass
class ControllerResult:
    plan_idx: int
    plan: Path  # containes starting position?
    controller_name: str
    start_time: float  # nanoseconds
    end_time: float  # nanoseconds
    result: bool
    poses: List[PoseStamped]
    twists: List[TwistStamped]
    costmaps: List[OccupancyGrid]


@dataclass
class ControllerMetric:
    plan_idx: int
    plan_length: int  # length of the generated plan [m]
    plan: Path
    result: bool

    controller_name: str
    length: float  # [m]
    diff_from_plan: float  # [m]

    x: List[float]  # [m]
    y: List[float]  # [m]
    dt_odom: List[float]  # [s]
    dt_twist: List[float]  # [s]

    lin_vel: List[float]  # [m/s]
    lin_vel_avg: float
    lin_acc: List[float]  # [m/s^2]
    lin_acc_avg: float
    lin_jerk: List[float]  # [m/s^3]
    lin_jerk_avg: float
    lin_jerk_rms: float

    ang_vel: List[float]
    ang_vel_avg: float
    ang_acc: List[float]
    ang_acc_avg: float
    ang_jerk: List[float]
    ang_jerk_avg: float
    ang_jerk_rms: float
