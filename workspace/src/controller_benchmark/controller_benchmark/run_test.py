#!/usr/bin/env python3

# Common modules
import os
from threading import Thread
import numpy as np
import time
from random import randint, getrandbits, uniform
from scipy.spatial.transform import Rotation
from typing import List, Type, Dict
import pickle

# Ros related modules
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Ros message types
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState, GetEntityState

from controller_benchmark.utils import ControllerResult


this_package_dir = get_package_share_directory('controller_benchmark')
map_file = os.path.join(this_package_dir, '10by10_empty.yaml')
logger = rclpy.logging.get_logger('controller_benchmark')


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


def spin_executor(executor):
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass


def main():
    logger.info('Controller benchmark started')
    rclpy.init()

    nav = BasicNavigator()
    nav.waitUntilNav2Active('planner_server', 'controller_server')

    marker_server_node = MarkerServer()
    vis_executor = rclpy.executors.MultiThreadedExecutor()
    vis_executor.add_node(marker_server_node)
    vis_executor_thread = Thread(target=vis_executor.spin, daemon=True)
    vis_executor_thread.start()

    # nav.changeMap(map_file)
    # time.sleep(2)

    # Get global costmap for goal generation
    # size_x: Number of cells in the horizontal direction
    # size_y: Number of cells in the vertical direction
    global_costmap_msg = nav.getGlobalCostmap()
    global_costmap = np.asarray(global_costmap_msg.data)
    global_costmap.resize(
        global_costmap_msg.metadata.size_y, global_costmap_msg.metadata.size_x)
    resolution = global_costmap_msg.metadata.resolution
    logger.info(
        f'Global costmap size: {global_costmap.shape}, resolution: {resolution}')

    # Generate random goals
    logger.info('Generating goals...')
    # goals needs to be on global costmap
    # goals needs to be on free space
    # robot footprint around goal should be free
    # has to be at least min_distance away from starting point
    number_of_goals = 2
    goals = []
    min_distance = 3.0  # [m]
    robot_length = 0.2  # along the x-axis of the robot [m]
    robot_width = 0.2  # along the y-axis [m]

    start = PoseStamped()
    start.header.frame_id = 'map'
    start.header.stamp = nav.get_clock().now().to_msg()
    start.pose.position.x = 0.0
    start.pose.position.y = 0.0
    q = Rotation.from_euler(
        'zyx', [0., 0., 0.], degrees=False).as_quat()
    start.pose.orientation.w = q[0]
    start.pose.orientation.x = q[1]
    start.pose.orientation.y = q[2]
    start.pose.orientation.z = q[3]

    number_of_generation = 0
    while len(goals) < number_of_goals:
        number_of_generation += 1

        if number_of_generation > 100 * number_of_goals:
            logger.error('Cannot find enough goals')
            break

        x_buffer = int(robot_width / 2 / resolution)
        # d_buffer = int(min_distance / resolution)
        # remove distance buffer from random gen because one axes is enough for distance
        goal_x = randint(x_buffer, global_costmap.shape[1] / 2 - x_buffer)
        sign_x = getrandbits(1)
        goal_x = goal_x if sign_x == 0 else -goal_x

        y_buffer = int(robot_length / 2 / resolution)
        goal_y = randint(y_buffer, global_costmap.shape[0] / 2 - y_buffer)
        sign_y = getrandbits(1)
        goal_y = goal_y if sign_y == 0 else -goal_y

        yaw = uniform(0, 1) * 2 * np.pi

        logger.info(f'Randomized goal: {goal_x}, {goal_y}, {yaw/np.pi*180}˚')

        # check if goal is on global costmap
        if global_costmap.shape[0] <= goal_y or global_costmap.shape[1] <= goal_x:
            continue

        # check if robot can reach goal
        if global_costmap[goal_y, goal_x] != 0:
            continue

        # check if footprint is free
        roi = global_costmap[goal_y - y_buffer:goal_y +
                             y_buffer, goal_x - x_buffer:goal_x + x_buffer]
        if np.average(roi) != 0:
            continue

        # check if min distance is kept
        goal_x *= resolution
        goal_y *= resolution
        distance = np.sqrt((goal_x - start.pose.position.x) ** 2 +
                           (goal_y - start.pose.position.y) ** 2)
        if distance < min_distance:
            continue

        # make goal msg
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = nav.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y

        q = Rotation.from_euler(
            'zyx', [yaw, 0., 0.], degrees=False).as_quat()
        goal.pose.orientation.w = q[0]
        goal.pose.orientation.x = q[1]
        goal.pose.orientation.y = q[2]
        goal.pose.orientation.z = q[3]

        goals.append(goal)

    marker_server_node.publish_markers(goals)
    logger.info(f'Generated goals, from {number_of_generation} tries: {goals}')

    # Generating global plans
    logger.info('Generating global plans...')
    planner = 'GridBased'
    global_plans: Dict[int, Path] = {}
    number_of_generation = 0
    while len(global_plans) < number_of_goals:
        number_of_generation += 1
        goal = goals[len(global_plans)]

        if number_of_generation > 10 * number_of_goals:
            logger.error(f'Cannot generate global plan for goal: {goal}')

        global_plan = nav.getPath(start, goal, planner, use_start=True)
        if global_plan is None:
            continue

        global_plans[len(global_plans)] = global_plan

    logger.info(f'Generated global plans, from {number_of_generation}')

    # Start navigation for each plan
    # we need to collect:
    # - poses
    # - cmd_vel
    # - costmap
    logger.info('Running controllers through plans...')
    controllers = ['MPPI_example', 'MPPI_Maneuver']

    cmd_vel_sub_node = CmdVelListener()
    odom_sub_node = ListenerBase(
        'odom_subscriber', '/odom', Odometry)
    costmap_sub_node = ListenerBase(
        'costmap_subscriber', '/local_costmap/costmap', OccupancyGrid)
    gazebo_interface_node = GazeboInterface()

    sub_executor = rclpy.executors.MultiThreadedExecutor()
    sub_executor.add_node(cmd_vel_sub_node)
    sub_executor.add_node(odom_sub_node)
    sub_executor.add_node(costmap_sub_node)
    sub_executor.add_node(gazebo_interface_node)
    sub_executor_thread = Thread(
        target=spin_executor, args=(sub_executor, ), daemon=True)
    sub_executor_thread.start()
    time.sleep(0.5)  # wait for nodes to start

    controller_results: List[ControllerResult] = []
    for i, plan in global_plans.items():
        marker_server_node.publish_path(plan)
        logger.info(f'Starting plan: {i}')
        for controller in controllers:
            gazebo_interface_node.reset_world()
            time.sleep(0.5)

            nav.clearLocalCostmap()
            start_time = rclpy.clock.Clock().now()
            nav.followPath(plan, controller_id=controller)
            logger.info(
                f'Starting controller: {controller}, '
                f'started at time: {start_time.seconds_nanoseconds}')
            while not nav.isTaskComplete():
                time_ = rclpy.clock.Clock().now().seconds_nanoseconds()
                # logger.info(f'Measuring at: {time_}')
                time.sleep(0.1)
            end_time = rclpy.clock.Clock().now()

            nav_result = False
            if (nav.getResult() == TaskResult.SUCCEEDED):
                nav_result = True
            elif (nav.getResult() == TaskResult.FAILED):
                nav_result = False
            else:
                logger.info(
                    f'{controller} unexpected result: {nav.getResult()}')
                nav_result = False

            logger.info(
                f'{controller} finished with result: {nav_result}, '
                f'at: {end_time.seconds_nanoseconds},')

            controller_results.append(ControllerResult(
                plan_idx=i,
                plan=plan,
                controller_name=controller,
                start_time=start_time.nanoseconds,
                end_time=end_time.nanoseconds,
                result=nav_result,
                poses=odom_sub_node.get_msgs(start_time, end_time),
                twists=cmd_vel_sub_node.get_msgs(start_time, end_time),
                costmaps=costmap_sub_node.get_msgs(start_time, end_time)
            ))
            logger.info(f'{controller} finished plan: {i}')
            gazebo_interface_node.reset_world()

    logger.info('Controllers finished')

    logger.info("Write Results...")
    # stamp = time.strftime("%Y-%m-%d-%H-%M")
    # filename = f'/controller_benchmark_results_{stamp}.pickle'
    filename = '/controller_benchmark_results.pickle'
    with open(os.getcwd() + filename, 'wb+') as f:
        pickle.dump(controller_results, f, pickle.HIGHEST_PROTOCOL)

    logger.info('Controller benchmark finished')

    marker_server_node.destroy_node()
    cmd_vel_sub_node.destroy_node()
    odom_sub_node.destroy_node()
    costmap_sub_node.destroy_node()
    gazebo_interface_node.destroy_node()

    vis_executor.shutdown()
    vis_executor_thread.join()
    sub_executor.shutdown()
    sub_executor_thread.join()

    rclpy.shutdown()
    exit(0)


if __name__ == '__main__':
    main()
