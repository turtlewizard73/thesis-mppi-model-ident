#! /usr/bin/env python3

# Common modules
from typing import List, Any, Type, Tuple, Dict, Deque
import numpy as np
from collections import deque

# ROS related modules
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RosTime

# ROS message types
from std_msgs.msg import Header
from std_srvs.srv import Empty, Trigger
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import MarkerArray, Marker
from gazebo_msgs.srv import SetEntityState, GetEntityState
from rcl_interfaces.srv import GetParameters, SetParameters
from nav2_mppi_controller.msg import CriticScores

# Custom modules
import utils.util_functions as util_funs
class MarkerServer(Node):
    """
    This class represents a marker server that publishes markers and paths.

    Args:
        Node (_type_): The base class for creating a ROS node.
    """

    def __init__(self):
        super().__init__('marker_server')
        self.marker_publisher = self.create_publisher(
            MarkerArray, 'waypoints', 10)
        self.path_publisher = self.create_publisher(
            Path, '/plan', 10)

    def publish_markers(
            self,
            poses: List[PoseStamped],
            namespace: str,
            rgb: Tuple[float, float, float]):
        """
        Publishes markers based on the given poses.

        Args:
            poses (PoseStamped): The list of poses to create markers from.
        """
        marker_array = MarkerArray()
        for i, pose in enumerate(poses):
            marker = Marker()
            marker.ns = namespace
            marker.header.frame_id = pose.header.frame_id
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.scale.x = 0.5
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = rgb[0]
            marker.color.g = rgb[1]
            marker.color.b = rgb[2]
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

    def publish_generated_goals(self, x: np.ndarray, y: np.ndarray, yaws: np.ndarray):
        poses: List[PoseStamped] = []
        for i in range(len(x)):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x[i]
            pose.pose.position.y = y[i]
            pose.pose.orientation = util_funs.yaw2quat(yaws[i])
            poses.append(pose)

        self.publish_markers(poses, namespace='generated_goals', rgb=(0.0, 0.0, 1.0))

    def publish_path(self, path: Path):
        """
        Publishes path.

        Args:
            path (Path): Path to publish.
        """
        self.path_publisher.publish(path)


class GazeboInterface(Node):
    def __init__(self):
        super().__init__('gazebo_interface')
        self.reset_world_client = self.create_client(Empty, 'reset_world')
        self.set_entity_client = self.create_client(
            SetEntityState, 'set_entity_state')
        self.get_entity_client = self.create_client(
            GetEntityState, 'get_entity_state')
        self.get_logger().info('Gazebo interface initialized')

    def reset_world(self):
        req = Empty.Request()
        return self.make_client_async_call(self.reset_world_client, req)

    def get_entity_state(self, name):
        req = GetEntityState.Request()
        req.name = name
        return self.make_client_async_call(self.get_entity_client, req)

    def set_entity_state(self, name, pose, twist=None):
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose = pose
        req.state.twist = twist if twist is not None else Twist()
        return self.make_client_async_call(self.set_entity_client, req)

    def make_client_async_call(self, client, req):
        max_attempts = 6
        for i in range(max_attempts):
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    f'service {client.srv_name} not available '
                    f'[{i}/{max_attempts}], waiting again...')

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


class OdomSubscriber(Node):
    def __init__(self, topic: str) -> None:
        super().__init__('odom_subscriber')
        # Initialize deques
        self.odom_xy = deque()
        self.odom_t_ns = deque()

        self.collect_data = False

        # self.start_stop_service = self.create_service(
        #     Trigger, 'start_stop_collecting', self.start_stop_collecting)

        self.subscription = self.create_subscription(
            Odometry, topic, self.callback, 10)
        self.get_logger().info('odom_subscriber initialized')

    # def start_stop_collecting(self, request, response):
    #     self.collect_data = not self.collect_data
    #     response.success = True
    #     response.message = 'Started collecting data' if self.collect_data else 'Stopped collecting data'
    #     return response

    def start_stop_collecting(self) -> str:
        self.collect_data = not self.collect_data
        return 'started collecting data' if self.collect_data else 'stopped collecting data'

    def callback(self, msg: Odometry) -> None:
        if self.collect_data is False:
            return
        # Append position (x, y) and timestamp (in nanoseconds) to deques
        self.odom_t_ns.append(self.get_clock().now().nanoseconds)
        self.odom_xy.append([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def get_data_between(self, start_time_ns, end_time_ns) -> Tuple[np.ndarray, np.ndarray]:
        xy_values = []
        t_values = []

        self.get_logger().debug(f'Getting data between {start_time_ns} and {end_time_ns}')

        # theoretically deque is thread-safe, but we are not sure about it
        for _ in range(len(self.odom_t_ns)):
            xy = self.odom_xy.popleft()
            t = self.odom_t_ns.popleft()

            if start_time_ns <= t <= end_time_ns:
                xy_values.append(xy)
                t_values.append(t - start_time_ns)  # Wrap t to match desired ndarray shape (0, 1))

        # Convert lists to NumPy arrays
        return np.array(xy_values), np.array(t_values)


class CmdVelSubscriber(Node):
    def __init__(self, topic: str) -> None:
        super().__init__('cmd_vel_subscriber')
        # Initialize deques
        self.cmd_vel_xy = deque()
        self.cmd_vel_omega = deque()
        self.cmd_vel_t_ns = deque()

        self.collect_data = False

        self.subscription = self.create_subscription(
            TwistStamped, topic, self.callback, 10)
        self.get_logger().info('cmd_vel_subscriber initialized')

    def callback(self, msg: TwistStamped) -> None:
        if self.collect_data is False:
            return
        # Append velocity (vx, vy, omega) and timestamp (in nanoseconds) to deques
        self.cmd_vel_t_ns.append(self.get_clock().now().nanoseconds)
        self.cmd_vel_xy.append([msg.twist.linear.x, msg.twist.linear.y])
        self.cmd_vel_omega.append(msg.twist.angular.z)

    def get_data_between(self, start_time_ns, end_time_ns) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        xy_values = []
        omega_values = []
        t_values = []

        self.get_logger().debug(f'Getting data between {start_time_ns} and {end_time_ns}')

        # theoretically deque is thread-safe, but we are not sure about it
        for _ in range(len(self.cmd_vel_t_ns)):
            xy = self.cmd_vel_xy.popleft()
            omega = self.cmd_vel_omega.popleft()
            t = self.cmd_vel_t_ns.popleft()

            if start_time_ns <= t <= end_time_ns:
                xy_values.append(xy)
                omega_values.append(omega)
                t_values.append(t - start_time_ns)

        return np.array(xy_values), np.array(omega_values), np.array(t_values)


class MPPICriticSubscriber(Node):
    def __init__(self):
        super().__init__('mppi_critic_subscriber')
        self.critic_scores = deque()
        self.critic_scores_t_ns: Dict[str, Deque] = deque()

        self.collect_data = False

    def callback(self, msg: CriticScores):
        if self.collect_data is False:
            return

        if len(msg.critic_names) != len(msg.critic_scores):
            raise ValueError('Critic names and scores have different lengths')

        self.critic_scores_t_ns.append(self.get_clock().now().nanoseconds)
        for name, critic in zip(msg.critic_names, msg.critic_scores):
            if name not in self.critic_scores:
                self.critic_scores[name] = deque()
            self.critic_scores[name].append(critic)

    def get_data_between(self, start_time_ns, end_time_ns) -> Dict[str, np.ndarray]:
        critic_scores = {}
        for name, scores in self.critic_scores.items():
            critic_scores[name] = []
            for _ in range(len(scores)):
                score = scores.popleft()
                t = self.critic_scores_t_ns.popleft()
                if start_time_ns <= t <= end_time_ns:
                    critic_scores[name].append(score)
        return critic_scores


class CostmapSubscriber(Node):
    def __init__(self, topic: str):
        super().__init__('costmap_subscriber')
        pass

    def callback(self, msg):
        pass

    def get_msgs(self, start_time: RosTime, end_time: RosTime):
        pass


