#! /usr/bin/env python3

# Common modules
from typing import List, Any, Type, Tuple
import numpy as np
from scipy.spatial.transform import Rotation

# ROS related modules
import rclpy
from rclpy.node import Node
from rclpy.time import Time

# ROS message types
from std_msgs.msg import Header
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import MarkerArray, Marker
from gazebo_msgs.srv import SetEntityState, GetEntityState
from rcl_interfaces.srv import GetParameters, SetParameters


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
            namespace: str = 'goals',
            rgb: Tuple[float, float, float] = (0.0, 1.0, 0.0)):
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
            q = Rotation.from_euler('XYZ', [0., 0., yaws[i]], degrees=False).as_quat()
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
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


class ListenerBase(Node):
    def __init__(self, node_name: str, topic_name: str, msg_type: Type):
        super().__init__(node_name)
        self.msgs = []
        self.msg_type = msg_type
        self.topic_name = topic_name

        self.subscription = self.create_subscription(
            self.msg_type,
            self.topic_name,
            self.callback,
            10)
        self.get_logger().info(f'{node_name} initialized')

    def callback(self, msg):
        # do some unpacking here from ROS msg to numpy array
        # self.msgs.append(msg)
        pass

    def get_msgs(self, start_time: Time, end_time: Time):
        self.get_logger().info(f'Received msgs: {len(self.msgs)}')

        start = start_time.nanoseconds
        end = end_time.nanoseconds
        filtered_msgs = []
        for msg in self.msgs:
            if hasattr(self.msg_type, 'header'):
                msg_time = Time.from_msg(msg.header.stamp).nanoseconds
            else:
                msg_time = Time.from_msg(msg[0].stamp).nanoseconds
                msg = msg[1]

            if start <= msg_time <= end:
                filtered_msgs.append(msg)

        if len(filtered_msgs) == 0:
            self.get_logger().warn(
                f'No msgs found within the time range: {start} - {end}')
            return None

        self.msgs = []
        return filtered_msgs


class OdomSubscriber(ListenerBase):
    def __init__(self, topic: str):
        super().__init__('odom_subscriber', topic, Odometry)

        self.subscription = self.create_subscription(
            Odometry, self.topic_name, self.callback, 10)
        self.get_logger().info(f'{node_name} initialized')

    def callback(self, msg):
        xy = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        t = Time.from_msg(msg.header.stamp).nanoseconds
        odom_xy = np.append(odom_xy, [xy], axis=0)


class CmdVelSubscriber(ListenerBase):
    def __init__(self):
        super().__init__('cmd_vel_subscriber', '/cmd_vel', TwistStamped)