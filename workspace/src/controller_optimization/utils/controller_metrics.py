#! /usr/bin/env python3

from dataclasses import dataclass, field
import numpy as np
from typing import Dict


@dataclass
class ControllerMetric:
    # ___________ IDENTIFIERS ___________
    # represent a result of a parametrized controller run on a single plan/map
    controller_name: str
    map_name: str
    uid: str = ''  # unique identifier, could be a timestamp, id or params

    # ___________ TIME & RESULT ___________
    # common properties from result needed for plotting
    time_elapsed: float = 0.0  # [s]
    success: bool = False  # if the controller reached the goal approximately
    status_msg: str = ''  # reason of fail or extra info

    # ___________ COLLECTED DATA ___________
    # path xy from
    # nav_msg/Path https://docs.ros.org/en/hydro/api/nav_msgs/html/msg/Path.html
    #  -> poses - geometry_msgs/PoseStamped[]
    #    -> pose - geometry_msgs/Pose
    #      -> position - geometry_msgs/Point
    #        -> x, y - float64
    #    -> orientation - geometry_msgs/Quaternion
    #        -> x, y, z, w - float64
    path_xy: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    path_omega: np.ndarray = field(default_factory=lambda: np.empty((1)))

    # odometry xy from
    # nav_msg/Odometry https://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html
    #  -> pose - geometry_msgs/PoseWithCovariance
    #    -> pose - geometry_msgs/Pose
    #      -> position - geometry_msgs/Point
    #        -> x, y - float64
    #      -> orientation - geometry_msgs/Quaternion
    #        -> x, y, z, w - float64
    odom_xy: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    odom_omega: np.ndarray = field(default_factory=lambda: np.empty((1)))
    odom_t: np.ndarray = field(default_factory=lambda: np.empty((1)))

    # cmd_vel vx vy from
    # geometry_msgs/TwistStamped https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TwistStamped.html
    #  -> twist - geometry_msgs/Twist
    #    -> linear - geometry_msgs/Vector3
    #      -> x, y - float64
    #    -> angular - geometry_msgs/Vector3
    #      -> z - float64
    cmd_vel_xy: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    cmd_vel_omega: np.ndarray = field(default_factory=lambda: np.empty((1)))
    cmd_vel_t: np.ndarray = field(default_factory=lambda: np.empty((1)))

    # mppi critic scores
    # nav2_mppi_controller/msg/CriticScores
    #  -> critic_names - string[]
    #  -> critic_scores - float32[]
    critic_scores: Dict[str, np.ndarray] = field(default_factory=dict)
    critic_scores_t: np.ndarray = field(default_factory=lambda: np.empty((1)))

    # avarage cost of robot's center from
    # nav_msgs/OccupancyGrid https://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
    # -> data - int8[] -> 0-100
    # -> info - MapMetaData
    #    -> resolution - float32
    #    -> width - uint32
    #    -> height - uint32
    costmap: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    costmap_resolution: float = 0.0
    costmap_origin_x: float = 0.0
    costmap_origin_y: float = 0.0

    # ___________ CALCULATED METRICS ___________
    frechet_distance: float = 0.0  # [m] Frechet distance between path and goal
    distance_to_goal: float = 0.0  # [m] stopping distance from the goal
    angle_to_goal: float = 0.0  # [rad] stopping angle difference from the goal

    # linear velocity metrics
    avg_linear_velocity: float = 0.0  # [m/s] average linear velocity
    max_linear_velocity: float = 0.0  # [m/s] maximum linear velocity
    rms_linear_velocity: float = 0.0  # [m/s] root mean squared linear velocity

    # linear acceleration metrics
    linear_acceleration: np.ndarray = field(
        default_factory=lambda: np.empty((1)))  # [m/s^2] acceleration
    avg_linear_acceleration: float = 0.0  # [m/s^2] average linear acceleration
    max_linear_acceleration: float = 0.0  # [m/s^2] maximum linear acceleration
    rms_linear_acceleration: float = 0.0  # [m/s^2] root mean squared linear acceleration

    # angular acceleration metrics
    avg_angular_acceleration: float = 0.0  # [rad/s^2] average angular acceleration
    max_angular_acceleration: float = 0.0  # [rad/s^2] maximum angular acceleration

    # jerk metrics
    linear_jerks: np.ndarray = field(
        default_factory=lambda: np.empty((1)))  # [m/s^3] jerk
    rms_linear_jerk: float = 0.0  # [m/s^3] root mean squared linear jerk

    angular_jerks: np.ndarray = field(
        default_factory=lambda: np.empty((1)))  # [rad/s^3] angular jerk
    rms_angular_jerk: float = 0.0  # [rad/s^3] root mean squared angular jerk

    # cost metrics
    # cost of cells along the traversed path in robot radius
    costmap_masked: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    path_costs: np.ndarray = field(default_factory=lambda: np.empty(
        (1)))  # cost of cells along the traversed path
    sum_of_costs: float = 0.0  # sum of all costs in footprint along path
    avg_cost: float = 0.0  # average cost
