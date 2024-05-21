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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry, OccupancyGrid

from utils import (
    ControllerResult, MarkerServer, ParamInterface,
    CmdVelListener, ListenerBase, GazeboInterface)


this_package_dir = get_package_share_directory('controller_benchmark')
# TODO: make config file importable before
config_file = os.path.join(
    os.path.dirname(os.path.dirname(__file__)), 'config', 'run_test_config.yaml')
params = yaml.safe_load(open(config_file))

# make directory for results
params['output_dir'] = os.path.join(
    os.path.dirname(__file__), params['output_dir'])
os.makedirs(params['output_dir'], exist_ok=True)
params['controller_runs'] = \
    1 if params['number_of_goals'] == 1 else params['controller_runs']

logger = rclpy.logging.get_logger('controller_benchmark')

logger.info(f'{params}')


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


def generate_random_goals(
        number_of_goals: int,
        min_distance: float,
        global_costmap: np.ndarray,
        resolution: float,
        robot_width: float,
        robot_length: float):

    start = PoseStamped()
    start.header.frame_id = 'map'
    # start.header.stamp = nav.get_clock().now().to_msg()
    start.pose.position.x = 0.0
    start.pose.position.y = 0.0

    q = Rotation.from_euler('XYZ', [0., 0., 0.], degrees=False).as_quat()
    start.pose.orientation.x = q[0]
    start.pose.orientation.y = q[1]
    start.pose.orientation.z = q[2]
    start.pose.orientation.w = q[3]

    random_goals = []
    number_of_generation = 0
    while len(random_goals) < number_of_goals:
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
        # goal.header.stamp = nav.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y

        q = Rotation.from_euler('XYZ', [0., 0., yaw], degrees=False).as_quat()
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        random_goals.append(goal)

    return start, random_goals


def main():
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
    start_pose = get_random_free_pose(
        global_costmap, resolution, params['robot_width'], params['robot_length'])
    marker_server_node.publish_markers([start_pose], 'start')

    # Generate random goals and plans
    logger.info('Generating goals and plans...')
    random_goals = []
    global_plans: Dict[int, Path] = {}

    number_of_generation = 0
    while len(random_goals) < params['number_of_goals']:
        number_of_generation += 1
        if number_of_generation > 100 * params['number_of_goals']:
            logger.error('Cannot find enough goals')
            break

        goal_pose = get_random_free_pose(
            global_costmap, resolution, params['robot_width'], params['robot_length'])

        # check if min distance is kept
        distance = np.sqrt(
            (goal_pose.pose.position.x - start_pose.pose.position.x) ** 2 +
            (goal_pose.pose.position.y - start_pose.pose.position.y) ** 2)
        if distance < params['min_distance']:
            continue

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

    nav.setInitialPose(start_pose)

    # Start navigation for each plan
    logger.info('Running controllers through plans...')
    cmd_vel_sub_node = CmdVelListener()
    odom_sub_node = ListenerBase(
        'odom_subscriber', params['odom_topic'], Odometry)
    costmap_sub_node = ListenerBase(
        'costmap_sub', params['costmap_topic'], OccupancyGrid)
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
        for controller in params['controllers']:
            for j in range(params['controller_runs']):
                gazebo_interface_node.reset_world()
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
                    time.sleep(0.1)
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
                    costmaps=costmap_sub_node.get_msgs(start_time, end_time)
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
    gazebo_interface_node.destroy_node()

    vis_executor.shutdown()
    vis_executor_thread.join()
    sub_executor.shutdown()
    sub_executor_thread.join()

    rclpy.shutdown()
    exit(0)


if __name__ == '__main__':
    main()
