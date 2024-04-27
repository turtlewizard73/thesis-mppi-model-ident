#! /usr/bin/env python3

from dataclasses import dataclass
from typing import List, Any, Type

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import parameter_value_to_python

from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from gazebo_msgs.srv import SetEntityState, GetEntityState
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters


@dataclass
class ControllerResult:
    controller_name: str
    plan_idx: int
    plan: Path  # containes starting position?
    run: int
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

    def publish_markers(self, poses: PoseStamped):
        """
        Publishes markers based on the given poses.

        Args:
            poses (PoseStamped): The list of poses to create markers from.
        """
        marker_array = MarkerArray()
        for i, pose in enumerate(poses):
            marker = Marker()
            marker.header.frame_id = pose.header.frame_id
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            # TODO: no orientation
            marker.pose = pose.pose
            marker.scale.x = 0.5
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

    def publish_path(self, path: Path):
        """
        Publishes path.

        Args:
            path (Path): Path to publish.
        """
        self.path_publisher.publish(path)


class ListenerBase(Node):
    def __init__(self, node_name: str, topic_name: str, msg_type: Type):
        super().__init__(node_name)
        self.msgs = []
        self.subscription = self.create_subscription(
            msg_type,
            topic_name,
            self.callback,
            10)
        self.get_logger().info(f'{node_name} initialized')

    def callback(self, msg):
        # Rewrite stamp from msg generation time to capture time
        if hasattr(msg, 'header'):
            msg.header.stamp = self.get_clock().now().to_msg()
        self.msgs.append(msg)

    def get_msgs(self, start_time: Time, end_time: Time):
        self.get_logger().info(f'Received msgs: {len(self.msgs)}')

        start = start_time.nanoseconds
        end = end_time.nanoseconds
        filtered_msgs = []
        for msg in self.msgs:
            msg_time = Time.from_msg(msg.header.stamp).nanoseconds
            if start <= msg_time <= end:
                filtered_msgs.append(msg)
        error_msg = f'No msgs found within the time range: {start} - {end}'
        assert len(filtered_msgs) > 0, error_msg

        self.msgs = []
        return filtered_msgs


class CmdVelListener(ListenerBase):
    def __init__(self):
        super().__init__(
            node_name='cmd_vel_subscriber',
            topic_name='/cmd_vel',
            msg_type=Twist)

    def callback(self, msg):
        msg_stamped = TwistStamped()
        msg_stamped.header.stamp = self.get_clock().now().to_msg()
        msg_stamped.twist = msg
        self.msgs.append(msg_stamped)


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

    def set_entity_state(self, name, pose, twist):
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose = pose
        req.state.twist = twist
        return self.make_client_async_call(self.set_entity_client, req)

    def make_client_async_call(self, client, req):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f'service {client.srv_name} not available, waiting again...')

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


class ParamInterface(Node):
    def __init__(self) -> None:
        super().__init__('param_interface')

    def set_param(self, server_name: str, param_name: str, param_value: Any):
        set_cli = self.create_client(
            SetParameters, '/' + server_name + '/set_parameters')
        while not set_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f'{server_name} service not available, waiting again...')

        param_ = rclpy.parameter.Parameter(name=param_name, value=param_value)
        req = SetParameters.Request()
        req.parameters = [param_.to_parameter_msg()]
        future = set_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        assert response.results[0].successful
        self.get_logger().info(f'Set {param_name} to {param_value}')
        return True

    def get_param(self, server_name: str, param_name: str) -> List:
        # ros2 service call /local_costmap/local_costmap/get_parameters
        # rcl_interfaces/srv/GetParameters  "{names: ["footprint_padding"]}"
        get_cli = self.create_client(
            GetParameters, '/' + server_name + '/' + 'get_parameters')
        while not get_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f'{server_name} service not available, waiting again...')

        req = GetParameters.Request()
        req.names = [param_name]
        future = get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f'Got {param_name} as {response}')

        param_msg = response.values[0]
        param_value = parameter_value_to_python(param_msg)

        self.get_logger().info(f'Got {param_name} as {param_value}')
        return param_value
