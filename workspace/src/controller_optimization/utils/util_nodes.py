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
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry, OccupancyGrid
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
        self.odom_omega = deque()
        self.odom_t_ns = deque()

        self.collect_data = False

        self.subscription = self.create_subscription(
            Odometry, topic, self.callback, 10)
        self.get_logger().info('odom_subscriber initialized')

    def callback(self, msg: Odometry) -> None:
        if self.collect_data is False:
            return
        # Append position (x, y) and timestamp (in nanoseconds) to deques
        self.odom_t_ns.append(self.get_clock().now().nanoseconds)
        # TODO: quaternion to radian
        self.odom_xy.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.odom_omega.append(msg.pose.pose.orientation.z)

    def get_data_between(self, start_time_ns, end_time_ns) -> Tuple[np.ndarray, np.ndarray]:
        self.get_logger().debug(f'Getting data between {start_time_ns} and {end_time_ns}')
        # Assert that the deques are the same length
        assert len(self.odom_xy) == len(self.odom_omega) == len(
            self.odom_t_ns), 'Length of odom_xy and odom_t_ns should be the same'

        # Convert deques to NumPy arrays for efficient slicing
        odom_xy = np.array(self.odom_xy)
        odom_omega = np.array(self.odom_omega)
        odom_t_ns = np.array(self.odom_t_ns)

        # Create a boolean mask to filter values between start_time_ns and end_time_ns
        mask = (odom_t_ns >= start_time_ns) & (odom_t_ns <= end_time_ns)

        # Apply the mask to get the desired values
        xy_values = odom_xy[mask]
        omega_values = odom_omega[mask]
        t_values = odom_t_ns[mask] - start_time_ns  # Wrap t to match desired ndarray shape
        t_values = t_values / 1e9  # Convert nanoseconds to seconds

        # Assert that the lengths of the arrays are the same after operation
        assert len(xy_values) == len(omega_values) == len(
            t_values), 'Length of xy_values and t_values should be the same'

        # Empty the deques
        self.odom_xy.clear()
        self.odom_omega.clear()
        self.odom_t_ns.clear()

        return xy_values, omega_values, t_values


class CmdVelSubscriber(Node):
    def __init__(self, topic: str) -> None:
        super().__init__('cmd_vel_subscriber')
        # Initialize deques
        self.cmd_vel_xy = deque()
        self.cmd_vel_omega = deque()
        self.cmd_vel_t_ns = deque()

        self.collect_data = False

        self.subscription = self.create_subscription(
            Twist, topic, self.callback, 10)
        self.get_logger().info('cmd_vel_subscriber initialized')

    def callback(self, msg: Twist) -> None:
        if self.collect_data is False:
            return
        # Append velocity (vx, vy, omega) and timestamp (in nanoseconds) to deques
        self.cmd_vel_t_ns.append(self.get_clock().now().nanoseconds)
        self.cmd_vel_xy.append([msg.linear.x, msg.linear.y])
        self.cmd_vel_omega.append(msg.angular.z)

    def get_data_between(self, start_time_ns, end_time_ns) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        self.get_logger().debug(f'Getting data between {start_time_ns} and {end_time_ns}')
        # Assert that the deques are the same length
        assert len(self.cmd_vel_xy) == len(self.cmd_vel_omega) == len(self.cmd_vel_t_ns), \
            'Length of cmd_vel_xy, cmd_vel_omega, and cmd_vel_t_ns should be the same'

        # Convert deques to NumPy arrays for efficient slicing
        cmd_vel_xy: np.ndarray = np.array(self.cmd_vel_xy)
        cmd_vel_omega = np.array(self.cmd_vel_omega)
        cmd_vel_t_ns = np.array(self.cmd_vel_t_ns)

        # Create a boolean mask to filter values between start_time_ns and end_time_ns
        mask = (cmd_vel_t_ns >= start_time_ns) & (cmd_vel_t_ns <= end_time_ns)

        # Apply the mask to get the desired values
        xy_values = cmd_vel_xy[mask]
        omega_values = cmd_vel_omega[mask]
        t_values = cmd_vel_t_ns[mask] - start_time_ns  # Wrap t to match desired ndarray shape
        t_values = t_values / 1e9  # Convert nanoseconds to seconds

        # Assert that the lengths of the arrays are the same after operation
        assert len(xy_values) == len(t_values) == len(omega_values), \
            'Length of xy_values, omega_values, and t_values should be the same'

        # Empty the deques
        self.cmd_vel_xy.clear()
        self.cmd_vel_omega.clear()
        self.cmd_vel_t_ns.clear()

        return xy_values, omega_values, t_values


class MPPICriticSubscriber(Node):
    def __init__(self, topic: str):
        super().__init__('mppi_critic_subscriber')
        self.critic_scores: Dict[str, Deque] = {}
        self.critic_scores_t_ns: Deque = deque()

        self.collect_data = False

        self.subscription = self.create_subscription(
            CriticScores, topic, self.callback, 10)
        self.get_logger().info('mppi_critic_subscriber initialized')

    def callback(self, msg: CriticScores):
        if self.collect_data is False:
            return

        self.critic_scores_t_ns.append(self.get_clock().now().nanoseconds)
        for critic_score in msg.critic_scores:
            critic_name = critic_score.name.data
            if critic_name not in self.critic_scores:
                self.critic_scores[critic_name] = deque()
            self.critic_scores[critic_name].append(critic_score.score.data)

    def get_data_between(self, start_time_ns, end_time_ns) -> Dict[str, np.ndarray]:
        self.get_logger().debug(f'Getting data between {start_time_ns} and {end_time_ns}')

        # Assert that the deques are the same length
        for critic in self.critic_scores:
            assert len(self.critic_scores[critic]) == len(self.critic_scores_t_ns), \
                f'Length of critic scores {critic} and critic_scores_t_ns should be the same'

        # Convert deques to NumPy arrays for efficient slicing
        critic_scores: Dict[str, np.ndarray] = {
            critic: np.array(scores) for critic, scores in self.critic_scores.items()}
        critic_scores_t_ns = np.array(self.critic_scores_t_ns)

        # Create a boolean mask to filter values between start_time_ns and end_time_ns
        mask = (critic_scores_t_ns >= start_time_ns) & (critic_scores_t_ns <= end_time_ns)

        # Apply the mask to get the desired values
        critic_score_values_dict = {critic: scores[mask]
                                    for critic, scores in critic_scores.items()}
        # Wrap t to match desired ndarray shape
        t_values = critic_scores_t_ns[mask] - start_time_ns
        t_values = t_values / 1e9  # Convert nanoseconds to seconds

        # Assert that the lengths of the arrays are the same after operation
        for critic in critic_score_values_dict:
            assert len(critic_score_values_dict[critic]) == len(t_values), \
                f'Length of critic scores {critic} and critic_scores_t_ns should be the same'

        # Empty the deques
        for critic in self.critic_scores:
            self.critic_scores[critic].clear()
        self.critic_scores_t_ns.clear()

        # Return the desired values
        return critic_score_values_dict, t_values

    def get_data_between_old(self, start_time_ns, end_time_ns) -> Dict[str, np.ndarray]:
        self.get_logger().debug(f'Getting data between {start_time_ns} and {end_time_ns}')
        critic_score_values_dict: Dict[str, List[np.ndarray]] = {
            critic: [] for critic in self.critic_scores}
        t_values: List[float] = []

        for _ in range(len(self.critic_scores_t_ns)):
            t = self.critic_scores_t_ns.popleft()

            # for each critic
            for critic, critic_score_q in self.critic_scores.items():
                score = critic_score_q.popleft()  # get the next score for the critic

                if t < start_time_ns or t > end_time_ns:
                    continue

                # if the critic is not in the dict, add it
                if critic not in critic_score_values_dict:
                    critic_score_values_dict[critic] = []

                critic_score_values_dict[critic].append(score)

            t_values.append(t - start_time_ns)

        t_values = t_values / 1e9  # Convert nanoseconds to seconds
        for critic in critic_score_values_dict:
            assert len(critic_score_values_dict[critic]) == len(t_values), \
                'Length of critic scores and t_values should be the same'
        return critic_score_values_dict, np.array(t_values)
