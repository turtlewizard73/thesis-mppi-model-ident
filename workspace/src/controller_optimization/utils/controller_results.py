#! /usr/bin/env python3

# common imports
from dataclasses import dataclass, field
import numpy as np
from typing import Dict


@dataclass
class ControllerResult:
    # represent a result of a parametrized controller run on a single plan/map
    controller_name: str
    map_name: str
    uid: str = ''  # unique identifier, could be a timestamp, id or params

    start_time: float = 0.0  # nanoseconds (to substract from time arrays)
    time_elapsed: float = 0.0  # nanoseconds
    success: bool = False  # if the controller reached the goal approximately
    status_msg: str = ''  # reason of fail or extra info

    # path xy from
    # nav_msg/Path https://docs.ros.org/en/hydro/api/nav_msgs/html/msg/Path.html
    #  -> poses - geometry_msgs/PoseStamped[]
    #    -> pose - geometry_msgs/Pose
    #      -> position - geometry_msgs/Point
    #        -> x, y - float64
    # TODO -> orientation - geometry_msgs/Quaternion
    #        -> x, y, z, w - float64
    path_xy: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    path_omega: np.ndarray = field(default_factory=lambda: np.empty((0, 1)))

    # odometry xy from
    # nav_msg/Odometry https://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html
    #  -> pose - geometry_msgs/PoseWithCovariance
    #    -> pose - geometry_msgs/Pose
    #      -> position - geometry_msgs/Point
    #        -> x, y - float64
    # TODO -> orientation - geometry_msgs/Quaternion
    #        -> x, y, z, w - float64
    odom_xy: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    # odometry t from
    # nav_msg/Odometry
    #  -> header - std_msgs/Header
    #    -> stamp - time [nanoseconds]
    odom_t: np.ndarray = field(default_factory=lambda: np.empty((0, 1)))

    # odometry vx vy from
    # TODO

    # cmd_vel vx vy from
    # geometry_msgs/TwistStamped https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TwistStamped.html
    #  -> twist - geometry_msgs/Twist
    #    -> linear - geometry_msgs/Vector3
    #      -> x, y - float64
    cmd_vel_xy: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    # cmd_vel vomega from
    # geometry_msgs/TwistStamped
    #  -> twist - geometry_msgs/Twist
    #    -> angular - geometry_msgs/Vector3
    #      -> z - float64
    cmd_vel_omega: np.ndarray = field(default_factory=lambda: np.empty((0, 1)))
    # cmd_vel t from
    # geometry_msgs/TwistStamped
    #  -> header - std_msgs/Header
    #    -> stamp - time [nanoseconds]
    cmd_vel_t: np.ndarray = field(default_factory=lambda: np.empty((0, 1)))

    # mppi critic scores
    # nav2_mppi_controller/msg/CriticScores
    #  -> critic_names - string[]
    #  -> critic_scores - float32[]
    critic_scores: Dict[str, np.ndarray] = field(default_factory=dict)
    critic_scores_t: np.ndarray = field(default_factory=lambda: np.empty((0, 1)))

    # avarage cost of robot's center from
    # nav_msgs/OccupancyGrid https://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
    # -> data - int8[] -> 0-100
    # -> info - MapMetaData
    #    -> resolution - float32
    #    -> width - uint32
    #    -> height - uint32
    costmap: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))

    # cost of every cell in the path
    path_costs: np.ndarray = field(default_factory=lambda: np.empty((0, 1)))
