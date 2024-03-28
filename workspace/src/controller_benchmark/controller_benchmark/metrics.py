#! /usr/bin/env python3
# Copyright (c) 2022 Enrico Sutera
# Copyright (c) 2022 Samsung R&D Institute Russia
# Copyright (c) 2022 Joshua Wallace
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState, GetEntityState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node

import math
import os
import pickle
import glob
import time
import numpy as np
import math
import copy
from random import seed
from random import randint
from random import uniform
from threading import Thread
from transforms3d.euler import euler2quat

OBSTACLE_SPEED = 0.1  # m/s
OBSTACLE_DISTANCE = 0.5  # m


def getPlannerResults(navigator, initial_pose, goal_pose, planners):
    results = []
    for planner in planners:
        navigator.clearGlobalCostmap()
        path = navigator._getPathImpl(
            initial_pose, goal_pose, planner, use_start=True)
        if path is not None and len(path.path.poses) > 0:
            results.append(path)
        else:
            return results
    return results


def getRandomStart(costmap, max_cost, side_buffer, time_stamp, res):
    start = PoseStamped()
    start.header.frame_id = 'map'
    start.header.stamp = time_stamp
    while True:
        row = randint(side_buffer, costmap.shape[0] - side_buffer)
        col = randint(side_buffer, costmap.shape[1] - side_buffer)

        if costmap[row, col] < max_cost:
            start.pose.position.x = col * res
            start.pose.position.y = row * res

            yaw = uniform(0, 1) * 2 * math.pi
            quad = euler2quat(0.0, 0.0, yaw)
            start.pose.orientation.w = quad[0]
            start.pose.orientation.x = quad[1]
            start.pose.orientation.y = quad[2]
            start.pose.orientation.z = quad[3]
            break
    return start


def getRandomGoal(costmap, start, max_cost, side_buffer, time_stamp, res):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = time_stamp
    while True:
        row = randint(
            side_buffer, costmap.shape[0] - side_buffer) - int(costmap.shape[0] / 2)
        col = randint(
            side_buffer, costmap.shape[1] - side_buffer) - int(costmap.shape[1] / 2)
        start_x = start.pose.position.x
        start_y = start.pose.position.y
        goal_x = col * res
        goal_y = row * res
        x_diff = goal_x - start_x
        y_diff = goal_y - start_y
        dist = math.sqrt(x_diff ** 2 + y_diff ** 2)
        if costmap[row, col] < max_cost and dist > 4.0:
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y

            yaw = uniform(0, 1) * 2 * math.pi
            quad = euler2quat(0.0, 0.0, yaw)
            goal.pose.orientation.w = quad[0]
            goal.pose.orientation.x = quad[1]
            goal.pose.orientation.y = quad[2]
            goal.pose.orientation.z = quad[3]
            break
    return goal


class CmdVelListener(Node):

    def __init__(self):
        super().__init__('benchmark_cmd_vel_node')
        self.twist_msg = Twist()
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning

    def cmd_vel_callback(self, msg):
        self.twist_msg = msg


class OdomListener(Node):

    def __init__(self):
        super().__init__('benchmark_odom_node')
        self.odom_msg = Odometry()
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

    def odom_callback(self, msg):
        self.odom_msg = msg


class GazeboInterface(Node):

    def __init__(self):
        super().__init__('gazebo_interface')
        self.reset_world_client = self.create_client(Empty, 'reset_world')
        self.set_entity_client = self.create_client(
            SetEntityState, 'set_entity_state')
        self.get_entity_client = self.create_client(
            GetEntityState, 'get_entity_state')

    # def reset_world(self):
    #     while not self.reset_world_client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info('service not available, waiting again...')
    #     self.req = Empty.Request()
    #     self.future = self.reset_world_client.call_async(self.req)
    #     rclpy.spin_until_future_complete(self, self.future)
    #     return self.future.result()

    def reset_world(self):
        req = Empty.Request()
        return self.make_client_aync_call(self.reset_world_client, req)

    def get_entity_state(self, name):
        req = GetEntityState.Request()
        req.name = name
        return self.make_client_aync_call(self.get_entity_client, req)

    def set_entity_state(self, name, pose, twist):
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose = pose
        req.state.twist = twist
        return self.make_client_aync_call(self.set_entity_client, req)

    def make_client_aync_call(self, client, req):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'service {} not available, waiting again...'.format(client.srv_name))

        self.future = client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def spin_executor(executor):
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass


def ActivateObstacle(gazebo_interface_node, start_pose, goal_pose):
    print("Activating obstacle")
    # Compute direction from goal to start for obstacle direction
    x_diff = start_pose.pose.position.x - goal_pose.pose.position.x
    y_diff = start_pose.pose.position.y - goal_pose.pose.position.y
    yaw = math.atan2(y_diff, x_diff)
    quad = euler2quat(0.0, 0.0, yaw)
    twist = Twist()
    twist.linear.x = OBSTACLE_SPEED * math.cos(yaw)
    twist.linear.y = OBSTACLE_SPEED * math.sin(yaw)
    twist.angular.z = 0.0
    pose = Pose()
    pose.position.x = goal_pose.pose.position.x
    pose.position.y = goal_pose.pose.position.y
    pose.position.z = 0.0
    pose.orientation.w = quad[0]
    pose.orientation.x = quad[1]
    pose.orientation.y = quad[2]
    pose.orientation.z = quad[3]
    # result = gazebo_interface_node.set_entity_state('person_standing', pose, twist)
    result = gazebo_interface_node.set_entity_state('obstacle', pose, twist)
    print(result)
    return


def UpdateObstacle(gazebo_interface_node, robot_pose):
    # Update obstacle position
    result = gazebo_interface_node.get_entity_state('obstacle')
    obstacle_pose = result.state.pose

    # Check distance from obstacle to robot
    x_diff = robot_pose.pose.position.x - obstacle_pose.position.x
    y_diff = robot_pose.pose.position.y - obstacle_pose.position.y
    dist = math.sqrt(x_diff * x_diff + y_diff * y_diff)
    if dist < OBSTACLE_DISTANCE:
        # Stop obstacle
        twist = Twist()
        result = gazebo_interface_node.set_entity_state(
            'obstacle', obstacle_pose, twist)
    return


def getControllerResults(navigator, path, controllers, start_pose, dynamic_obstacles=False):
    # Initialize results
    task_twists = []
    task_poses = []
    task_controller_results = []
    task_local_costmaps = []

    cmd_vel_subscriber_node = CmdVelListener()
    odom_subscriber_node = OdomListener()
    gazebo_interface_node = GazeboInterface()
    sub_executor = rclpy.executors.MultiThreadedExecutor()
    sub_executor.add_node(cmd_vel_subscriber_node)
    sub_executor.add_node(odom_subscriber_node)
    sub_executor.add_node(gazebo_interface_node)
    sub_thread = Thread(target=spin_executor,
                        args=(sub_executor, ), daemon=True)
    sub_thread.start()
    # Wait for clock to be received
    time.sleep(0.3)

    after_reset_pose = odom_subscriber_node.odom_msg.pose.pose
    gazebo_interface_node.reset_world()
    after_reset_pose = odom_subscriber_node.odom_msg.pose.pose
    # Since /reset_world service has not boolen result, hand check world has been reset...
    while (abs(start_pose.pose.position.x - after_reset_pose.position.x) > 0.01 and
           abs(start_pose.pose.position.y - after_reset_pose.position.y) > 0.01):
        print("Resetting again world...")
        gazebo_interface_node.reset_world()
        after_reset_pose = odom_subscriber_node.odom_msg.pose.pose
        time.sleep(0.5)

    for controller in controllers:

        if dynamic_obstacles:
            ActivateObstacle(gazebo_interface_node,
                             start_pose, path.path.poses[-1])

        print("Getting controller: ", controller)
        i = 0
        navigator.clearLocalCostmap()
        navigator.followPath(path.path, controller_id=controller)
        task_controller_twists = []
        task_controller_poses = []
        task_controller_local_costmaps = []
        while not navigator.isTaskComplete():
            # feedback does not provide both linear and angular
            # we get velocities from cmd_vel

            # Update "virtual" pose considering twist
            pose = PoseStamped()
            pose.pose = odom_subscriber_node.odom_msg.pose.pose
            pose.header.stamp = odom_subscriber_node.get_clock().now().to_msg()
            task_controller_poses.append(copy.deepcopy(pose))

            if dynamic_obstacles:
                # Update obstacle position
                UpdateObstacle(gazebo_interface_node, pose)

            # Get the local costmap for future metrics
            costmap_msg = navigator.getLocalCostmap()
            costmap = np.asarray(costmap_msg.data)
            costmap.resize(costmap_msg.metadata.size_y,
                           costmap_msg.metadata.size_x)

            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = cmd_vel_subscriber_node.get_clock().now().to_msg()
            twist_stamped.twist = cmd_vel_subscriber_node.twist_msg
            task_controller_twists.append(twist_stamped)
            task_controller_local_costmaps.append(costmap)
            # Do something with the feedback
        if (navigator.getResult() == TaskResult.SUCCEEDED):
            task_controller_results.append(True)
        elif (navigator.getResult() == TaskResult.FAILED):
            task_controller_results.append(False)
        else:
            print("Unexpected result: \n", navigator.getResult())
            task_controller_results.append(False)
        task_twists.append(task_controller_twists)
        task_poses.append(task_controller_poses)
        task_local_costmaps.append(task_controller_local_costmaps)
        gazebo_interface_node.reset_world()
    print("Cleaning resources ...")
    cmd_vel_subscriber_node.destroy_node()
    odom_subscriber_node.destroy_node()
    gazebo_interface_node.destroy_node()
    sub_executor.shutdown()
    sub_thread.join()
    print("Thread joint")
    return task_controller_results, task_twists, task_poses, task_local_costmaps


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Wait for planner and controller to fully activate
    print("Waiting for planner and controller servers to activate")
    navigator.waitUntilNav2Active('planner_server', 'controller_server')

    map_name = navigator.declare_parameter('map_name', "10by10_empty")
    dynamic_obstacles_parameter = navigator.declare_parameter(
        'dynamic_obstacles', False)
    dynamic_obstacles = dynamic_obstacles_parameter.value
    print("Selected map: ", map_name.value)
    # Set map to use, other options: 100by100_15, 100by100_10
    map_path = os.getcwd() + '/' + glob.glob('**/maps/' +
                                             map_name.value + '.yaml', recursive=True)[0]

    navigator.changeMap(map_path)
    time.sleep(2)

    # Get the costmap for start/goal validation
    costmap_msg = navigator.getGlobalCostmap()
    costmap = np.asarray(costmap_msg.data)
    costmap.resize(costmap_msg.metadata.size_y, costmap_msg.metadata.size_x)

    local_costmap_msg = navigator.getLocalCostmap()
    local_costmap_resolution = local_costmap_msg.metadata.resolution

    planners = ['GridBased']
    controllers = ["DWB_benchmark", "RPP_benchmark"]

    max_cost = 210
    side_buffer = 20
    time_stamp = navigator.get_clock().now().to_msg()

    planner_results = []
    # Will collect all controller all task data
    tasks_controller_results = []
    tasks_controller_twists = []
    tasks_controller_poses = []
    # List with  local costamap of each controller for each task
    tasks_controller_local_costmaps = []
    seed(33)

    random_pairs = 4
    res = costmap_msg.metadata.resolution
    i = 0
    while len(planner_results) != random_pairs:
        print("Cycle: ", i, "out of: ", random_pairs)
        # start = getRandomStart(costmap, max_cost, side_buffer, time_stamp, res)
        start = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = time_stamp
        start.pose.position.x = 0.0
        start.pose.position.y = 0.0
        goal = getRandomGoal(costmap, start, max_cost,
                             side_buffer, time_stamp, res)
        # print("Start", start)
        # print("Goal", goal)
        time.sleep(2)
        result = getPlannerResults(navigator, start, goal, planners)
        if len(result) == len(planners):
            planner_results.append(result)
        else:
            print("One of the planners was invalid")
            continue
        # Change back map, planner will no more use this. Local costmap uses this map
        # with obstalce info

        task_controller_results, task_twists, task_poses, task_local_costmaps = getControllerResults(
            navigator, result[0], controllers, start, dynamic_obstacles)
        tasks_controller_results.append(task_controller_results)
        tasks_controller_twists.append(task_twists)
        tasks_controller_poses.append(task_poses)
        tasks_controller_local_costmaps.append(task_local_costmaps)
        i = i + 1

    print("Write Results...")
    with open(os.getcwd() + '/tasks_controller_results.pickle', 'wb+') as f:
        pickle.dump(tasks_controller_results, f, pickle.HIGHEST_PROTOCOL)

    with open(os.getcwd() + '/tasks_controller_twists.pickle', 'wb+') as f:
        pickle.dump(tasks_controller_twists, f, pickle.HIGHEST_PROTOCOL)

    with open(os.getcwd() + '/tasks_controller_poses.pickle', 'wb+') as f:
        pickle.dump(tasks_controller_poses, f, pickle.HIGHEST_PROTOCOL)

    with open(os.getcwd() + '/controllers.pickle', 'wb+') as f:
        pickle.dump(controllers, f, pickle.HIGHEST_PROTOCOL)

    with open(os.getcwd() + '/local_costmaps.pickle', 'wb+') as f:
        pickle.dump(tasks_controller_local_costmaps,
                    f, pickle.HIGHEST_PROTOCOL)

    with open(os.getcwd() + '/local_costmap_resolution.pickle', 'wb+') as f:
        pickle.dump(local_costmap_resolution, f, pickle.HIGHEST_PROTOCOL)

    with open(os.getcwd() + '/planner_results.pickle', 'wb+') as f:
        pickle.dump(planner_results, f, pickle.HIGHEST_PROTOCOL)
    # TODO save results once we have them
    print("Write Complete")
    exit(0)


if __name__ == '__main__':
    main()
