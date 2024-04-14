#! /usr/bin/env python3

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