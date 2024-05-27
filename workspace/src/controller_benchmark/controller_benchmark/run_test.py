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
import yaml

# ROS related modules
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ROS message types
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from nav2_mppi_controller.msg import CriticScores

from utils import (
    ControllerResult, MarkerServer, ParamInterface,
    ListenerBase, GazeboInterface)


this_package_dir = get_package_share_directory('controller_benchmark')
# TODO: make config file importable before
config_file = os.path.join(
    os.path.dirname(os.path.dirname(__file__)), 'config', 'run_test_config.yaml')
params = yaml.safe_load(open(config_file))
map_file = os.path.join(
    os.path.dirname(os.path.dirname(__file__)), 'maps', params['map_file'])
params['map'] = yaml.safe_load(open(map_file))

# make directory for results
params['output_dir'] = os.path.join(
    os.path.dirname(__file__), params['output_dir'])
os.makedirs(params['output_dir'], exist_ok=True)
params['controller_runs'] = \
    1 if params['number_of_goals'] == 1 else params['controller_runs']
params['robot_radius'] = np.sqrt(params['robot_width'] ** 2 + params['robot_length'] ** 2)

logger = rclpy.logging.get_logger('controller_benchmark')

logger.info(f'{params}')

marker_server_node: MarkerServer = None


def spin_executor(executor):
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass


def get_random_free_pose(
        global_costmap: np.ndarray,
        resolution: float,
        robot_width: float,
        robot_length: float) -> PoseStamped:
    x_buffer = int(robot_width / 2 / resolution)
    y_buffer = int(robot_length / 2 / resolution)

    max_retry = 100
    for i in range(max_retry):
        goal_x = randint(x_buffer, int(global_costmap.shape[1] / 2 - x_buffer))
        sign_x = getrandbits(1)
        goal_x = goal_x if sign_x == 0 else -goal_x

        goal_y = randint(y_buffer, int(global_costmap.shape[0] / 2 - y_buffer))
        sign_y = getrandbits(1)
        goal_y = goal_y if sign_y == 0 else -goal_y

        # check if footprint is free
        roi = global_costmap[
            goal_y - y_buffer:goal_y + y_buffer,
            goal_x - x_buffer:goal_x + x_buffer]
        if np.average(roi) != 0:
            break

    yaw = uniform(0, 1) * 2 * np.pi
    q = Rotation.from_euler('XYZ', [0., 0., yaw], degrees=False).as_quat()

    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = goal_x * resolution
    pose.pose.position.y = goal_y * resolution
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]

    return pose


def check_pose_free(x: float, y: float, costmap: np.ndarray, res: float) -> bool:
    robot_radius = np.sqrt(
        params['robot_width'] ** 2 + params['robot_length'] ** 2)

    # check if footprint is free by using turn radius
    roi = costmap[
        int((y - robot_radius) / res):int((y + robot_radius) / res),
        int((x - robot_radius) / res):int((x + robot_radius) / res)]
    return np.average(roi) == 0


def check_poses_free(
        x: np.ndarray, y: np.ndarray, costmap: np.ndarray, res: float) -> np.ndarray:
    turning_radius = params['robot_radius']
    # convert xy to costmap origin
    x = x - params['map']['origin'][0]
    y = y - params['map']['origin'][1]

    # Calculate the bounding box coordinates for each (x, y) pair
    x_min = ((x - turning_radius) / res).astype(int)
    x_max = ((x + turning_radius) / res).astype(int)
    y_min = ((y - turning_radius) / res).astype(int)
    y_max = ((y + turning_radius) / res).astype(int)

    # Get costmap dimensions
    costmap_height, costmap_width = costmap.shape

    # Check if the coordinates are within bounds of the costmap
    within_bounds = (x_min > 0) & (x_max < costmap_width) & \
        (y_min > 0) & (y_max < costmap_height)

    # Initialize result array to False
    results = np.zeros(x.shape, dtype=bool)

    # Create a mask for coordinates that are within bounds
    valid_indices = np.where(within_bounds)[0]

    for i in valid_indices:
        roi = costmap[y_min[i]:y_max[i], x_min[i]:x_max[i]]
        results[i] = np.average(roi) == 0

    return results


def get_random_free_pose_polar(
        global_costmap: np.ndarray,
        resolution: float,
        start_pose: PoseStamped = None,
        min_distance: float = None) -> PoseStamped:
    global marker_server_node

    max_distance = resolution * (
        global_costmap.shape[0] + global_costmap.shape[1]) / 2 - params['robot_radius']

    if min_distance is not None:
        if min_distance > max_distance:
            logger.error('Min distance is larger than max distance')
            return None

        min_radius = min_distance
        points_to_generate = int(min_radius * 10)
    else:
        min_radius = 0.0
        points_to_generate = 1

    start_x = 0.0 if start_pose is None else start_pose.pose.position.x
    start_y = 0.0 if start_pose is None else start_pose.pose.position.y

    MAX_RETRY = 1000
    retries = 0
    while True:
        print(f'point to generate: {points_to_generate}')
        print(f'min_radius: {min_radius}')
        retries += 1
        if retries > MAX_RETRY:
            logger.error('Cannot find free pose')
            return None

        angles = np.random.rand(points_to_generate) * 2 * np.pi
        radius = np.random.rand(points_to_generate) * params['robot_radius'] + min_radius

        x = radius * np.cos(angles) + start_x
        y = radius * np.sin(angles) + start_y

        free = check_poses_free(x, y, global_costmap, resolution)
        if np.any(free):
            index = np.argmax(free)
            x = x[index]
            y = y[index]
            yaw = angles[index]
            break

        if min_radius + 2 * params['robot_radius'] < max_distance:
            min_radius += params['robot_radius']

        if points_to_generate < 1000:
            points_to_generate = int(min_radius * 10)

        marker_server_node.publish_generated_goals(x, y)

    logger.info(f'Generated pose at: {x}, {y}, {yaw}, from {retries} retries')

    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    q = Rotation.from_euler('XYZ', [0., 0., yaw], degrees=False).as_quat()
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


def main():
    global marker_server_node
    logger.info('Controller benchmark started')
    rclpy.init()

    nav = BasicNavigator()
    nav.waitUntilNav2Active('planner_server', 'controller_server')

    param_interface = ParamInterface()
    controller_plugins = param_interface.get_param(
        server_name='controller_server', param_name='controller_plugins')
    assert set(params['controllers']).issubset(controller_plugins), \
        f'Present controllers: {controller_plugins}'

    logger.info('Changing map...')
    nav.changeMap(os.path.join(this_package_dir, params['map_file']))
    nav.clearAllCostmaps()
    time.sleep(2)

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

    logger.info('Starting marker server...')
    marker_server_node = MarkerServer()
    vis_executor = rclpy.executors.MultiThreadedExecutor()
    vis_executor.add_node(marker_server_node)
    vis_executor_thread = Thread(target=vis_executor.spin, daemon=True)
    vis_executor_thread.start()

    # Generate random start
    logger.info('Generating starting pose...')
    start_pose = get_random_free_pose_polar(global_costmap, resolution)
    marker_server_node.publish_markers([start_pose], 'start')

    # Generate random goals and plans
    logger.info('Generating goals and plans...')
    random_goals = []
    global_plans: Dict[int, Path] = {}

    number_of_generation = 0
    while len(random_goals) < params['number_of_goals']:
        number_of_generation += 1
        if number_of_generation > 1000 * params['number_of_goals']:
            logger.error('Cannot find enough goals')
            break

        goal_pose = get_random_free_pose_polar(
            global_costmap, resolution, start_pose, params['min_distance'])

        # check if min distance is kept
        # distance = np.sqrt(
        #     (goal_pose.pose.position.x - start_pose.pose.position.x) ** 2 +
        #     (goal_pose.pose.position.y - start_pose.pose.position.y) ** 2)
        # if distance < params['min_distance']:
        #     continue

        nav.clearGlobalCostmap()
        time.sleep(0.5)

        global_plan = nav.getPath(start_pose, goal_pose, params['planner'], use_start=True)
        if global_plan is None:
            logger.error(f'Cannot generate global plan for goal: {goal_pose}')
            continue

        random_goals.append(goal_pose)
        global_plans[len(random_goals) - 1] = global_plan

    logger.info(f'Generated global plans, from {number_of_generation}')
    marker_server_node.publish_markers(random_goals)

    logger.info('Preparing navigation...')
    cmd_vel_sub_node = ListenerBase(
        'cmd_vel_subscriber', params['cmd_vel_topic'], Twist)
    odom_sub_node = ListenerBase(
        'odom_subscriber', params['odom_topic'], Odometry)
    costmap_sub_node = ListenerBase(
        'costmap_sub', params['costmap_topic'], OccupancyGrid)
    mppi_critics_sub_node = ListenerBase(
        'mppi_critics_sub', params['mppi_critic_topic'], CriticScores)

    gazebo_interface_node = GazeboInterface()

    sub_executor = rclpy.executors.MultiThreadedExecutor()
    sub_executor.add_node(cmd_vel_sub_node)
    sub_executor.add_node(odom_sub_node)
    sub_executor.add_node(costmap_sub_node)
    sub_executor.add_node(gazebo_interface_node)
    sub_executor.add_node(mppi_critics_sub_node)
    sub_executor_thread = Thread(
        target=spin_executor, args=(sub_executor, ), daemon=True)
    sub_executor_thread.start()
    time.sleep(0.5)  # wait for nodes to start

    gazebo_interface_node.set_entity_state(
        'turtlebot3_waffle', start_pose.pose, None)
    nav.setInitialPose(start_pose)
    nav.clearAllCostmaps()
    time.sleep(0.5)

    logger.info('Running controllers through plans...')
    controller_results: List[ControllerResult] = []
    for i, plan in global_plans.items():
        marker_server_node.publish_path(plan)
        logger.info(f'Starting plan: {i}')
        for controller in params['controllers']:
            for j in range(params['controller_runs']):
                gazebo_interface_node.reset_world()
                gazebo_interface_node.set_entity_state(
                    'turtlebot3_waffle', start_pose.pose, None)

                nav.clearLocalCostmap()
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
                    time.sleep(1)
                end_time = rclpy.clock.Clock().now()

                nav_result = False
                if (nav.getResult() == TaskResult.SUCCEEDED):
                    nav_result = True

                logger.info(
                    f'{controller} finished with result: {nav_result}, '
                    f'at: {end_time.seconds_nanoseconds},')

                controller_results.append(ControllerResult(
                    controller_name=controller,
                    plan_idx=i,
                    plan=plan,
                    run=j,
                    start_time=start_time.nanoseconds,
                    end_time=end_time.nanoseconds,
                    result=nav_result,
                    odom=odom_sub_node.get_msgs(start_time, end_time),
                    cmd_vel=cmd_vel_sub_node.get_msgs(start_time, end_time),
                    costmaps=costmap_sub_node.get_msgs(start_time, end_time),
                    critic_scores=mppi_critics_sub_node.get_msgs(start_time, end_time)
                ))
                gazebo_interface_node.reset_world()
            logger.info(f'{controller} finished plan: {i}')
    logger.info('Controllers finished')

    logger.info("Write Results...")
    stamp = time.strftime(params['timestamp_format'])
    filename = f'/controller_benchmark_results_{stamp}.pickle'
    # filename = '/controller_benchmark_results.pickle'
    with open(params['output_dir'] + filename, 'wb+') as f:
        pickle.dump(controller_results, f, pickle.HIGHEST_PROTOCOL)
    logger.info(f'Written results to: {params["output_dir"] + filename}')

    logger.info('Controller benchmark finished')

    marker_server_node.destroy_node()
    cmd_vel_sub_node.destroy_node()
    odom_sub_node.destroy_node()
    costmap_sub_node.destroy_node()
    mppi_critics_sub_node.destroy_node()
    gazebo_interface_node.destroy_node()

    vis_executor.shutdown()
    vis_executor_thread.join()
    sub_executor.shutdown()
    sub_executor_thread.join()

    rclpy.shutdown()
    exit(0)


if __name__ == '__main__':
    main()
