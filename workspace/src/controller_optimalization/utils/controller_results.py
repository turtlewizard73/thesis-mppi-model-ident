#! /usr/bin/env python3

# common imports
from dataclasses import dataclass, field
import numpy as np
from typing import List, Tuple


# ROS2 imports
from rclpy.time import Time
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TwistStamped


@dataclass
class ControllerResult:
    # represent a result of a parametrized controller run on a single plan/map
    controller_name: str
    start_time: float = 0.0  # nanoseconds (to substract from time arrays)
    time_elapsed: float = 0.0  # nanoseconds
    success: bool = False  # if the controller reached the goal approximately

    # path xy from
    # nav_msg/Path https://docs.ros.org/en/hydro/api/nav_msgs/html/msg/Path.html
    #  -> poses - geometry_msgs/PoseStamped[]
    #    -> pose - geometry_msgs/Pose
    #      -> position - geometry_msgs/Point
    #        -> x, y - float64
    # TODO -> orientation - geometry_msgs/Quaternion
    #        -> x, y, z, w - float64
    path_xy: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))

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

    # avarage cost of robot's center from
    # nav_msgs/OccupancyGrid https://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
    # -> data - int8[] -> 0-100
    # -> info - MapMetaData
    #    -> resolution - float32
    #    -> width - uint32
    #    -> height - uint32
    avg_costs: np.ndarray = field(default_factory=lambda: np.empty((0, 1)))


def xy_from_path(path: Path) -> np.ndarray:
    return np.array(
        [[pose_.position.x, pose_.position.y] for pose_ in path.poses])


def xyt_from_odom(odom: List[Odometry], start_time: float) -> Tuple[np.ndarray, np.ndarray]:
    valid_odom = [
        o for o in odom if Time.from_msg(o.header.stamp).nanoseconds >= start_time]
    n = len(valid_odom)
    xy = np.empty((n, 2))
    t = np.empty((n, 1))
    for i, odom_ in enumerate(valid_odom):
        timestamp_ns = Time.from_msg(odom_.header.stamp).nanoseconds
        xy[i] = [odom_.pose.pose.position.x, odom_.pose.pose.position.y]
        t[i] = timestamp_ns - start_time
    return xy, t


def vxvytomega_from_twist(twist: List[TwistStamped], start_time: float) -> Tuple[np.ndarray, np.ndarray]:
    valid_twist = [
        t for t in twist if Time.from_msg(t.header.stamp).nanoseconds >= start_time]
    n = len(valid_twist)
    xy = np.empty((n, 2))
    omega = np.empty((n, 1))
    t = np.empty((n, 1))
    for i, twist_ in enumerate(valid_twist):
        xy[i] = [twist_.twist.linear.x, twist_.twist.linear.y]
        omega[i] = twist_.twist.angular.z
        timestamp_ns = Time.from_msg(twist_.header.stamp).nanoseconds
        t[i] = timestamp_ns - start_time
    return xy, omega


