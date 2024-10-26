#! /usr/bin/env python3
# Copyright 2022 Joshua Wallace
# Copyright 2022 Enrico Sutera
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

import numpy as np
import math

import os
import pickle

from tabulate import tabulate
import nav_msgs.msg

FATAL_COST = 253


def getPaths(results):
    paths = []
    for result in results:
        for path in result:
            paths.append(path.path)
    return paths


def getTaskAvgLinearSpeed(controller_twists):
    linear_x = []
    for twist_stamped in controller_twists:
        linear_x.append(twist_stamped.twist.linear.x)
    return np.average(linear_x)


def getControllerPath(tasks_poses):
    controller_paths = []
    for task_poses in tasks_poses:
        for controller_poses in task_poses:
            path = nav_msgs.msg.Path()
            path.poses = controller_poses
            controller_paths.append(path)
    return controller_paths


def getObstacleDistances(tasks_local_costmaps, local_costmap_resolution):
    print("local_costmap_resolution ", local_costmap_resolution)
    # Will contains minimum obstacle distance for each controller
    controllers_obstacle_distances_min = []
    controllers_min_obstacle_distances_std = []
    controllers_min_obstacle_distances_avg = []
    # for each task / navigation
    for task_local_costamps in tasks_local_costmaps:
        # for each controller
        for controller_local_costmaps in task_local_costamps:
            min_obst_dist = 1e10
            # Will contain history of nearest obstacle distance of the controller
            # for this task, used to compute man and std
            controller_task_min_obstacle_distances = []
            for local_costmap in controller_local_costmaps:

                if local_costmap.max() >= FATAL_COST:
                    # Look for obstacle
                    obstacles_indexes = np.where(local_costmap > FATAL_COST)
                    # Shift indexes to centre of costmap since they
                    # will be used to compute distance from there
                    obstacles_indexes_x = np.add(
                        obstacles_indexes[0], -np.round(local_costmap.shape[0]))
                    obstacles_indexes_y = np.add(
                        obstacles_indexes[1], -np.round(local_costmap.shape[1]))

                    iteration_minimum_distance = 1e10
                    for x, y in zip(obstacles_indexes_x, obstacles_indexes_y):

                        obstacle_distance = math.sqrt(
                            x**2 + y**2) * local_costmap_resolution
                        if obstacle_distance < iteration_minimum_distance:
                            iteration_minimum_distance = obstacle_distance

                        if obstacle_distance < min_obst_dist:
                            # if obstacle_distance < 0.1:
                            #     print("New min obstacle distance found: ",obstacle_distance)
                            #     # Heat map
                            #     plt.imshow( local_costmap, cmap = 'rainbow' , interpolation = 'bilinear')
                            #     # Add Title
                            #     plt.title( "Heat Map" )
                            #     # Display
                            #     plt.show()
                            min_obst_dist = obstacle_distance
                    if iteration_minimum_distance == 1e10:
                        # TODO (@Enrico) maybe avoid appending at all
                        iteration_minimum_distance = np.nan
                    controller_task_min_obstacle_distances.append(
                        iteration_minimum_distance)

            if (min_obst_dist == 1e10):
                # TODO (@Enrico) maybe avoid appending at all
                min_obst_dist = np.nan

            controllers_min_obstacle_distances_avg.append(
                np.nanmean(controller_task_min_obstacle_distances))
            controllers_min_obstacle_distances_std.append(
                np.nanstd(controller_task_min_obstacle_distances))

            controllers_obstacle_distances_min.append(min_obst_dist)
    return controllers_obstacle_distances_min, controllers_min_obstacle_distances_avg, controllers_min_obstacle_distances_std


def getControllerMSJerks(controller_twists):
    linear_x = []
    angular_z = []
    time_passed = 0.0
    for twist_stamped in controller_twists:
        linear_x.append(twist_stamped.twist.linear.x)
        angular_z.append(twist_stamped.twist.angular.z)
        time_passed += twist_stamped.header.stamp.nanosec / \
            1e09 + twist_stamped.header.stamp.sec
    end = controller_twists[-1].header.stamp.nanosec / \
        1e09 + controller_twists[-1].header.stamp.sec
    start = controller_twists[0].header.stamp.nanosec / \
        1e09 + controller_twists[0].header.stamp.sec
    dt = (end - start) / len(controller_twists)
    # print(dt)
    linear_acceleration_x = np.gradient(linear_x, dt)
    angular_acceleration_z = np.gradient(angular_z, dt)
    linear_jerk_x = np.gradient(linear_acceleration_x, dt)
    angular_jerk_z = np.gradient(angular_acceleration_z, dt)
    # Mean Squared jerk Wininger, Kim, & Craelius (2009)
    ms_linear_jerk_x = 0
    for jerk in linear_jerk_x:
        ms_linear_jerk_x += jerk**2
    ms_angular_jerk_x = 0
    for jerk in angular_jerk_z:
        ms_angular_jerk_x += jerk**2
    return ms_linear_jerk_x, ms_angular_jerk_x


def getTaskTimes(controller_poses):

    return (controller_poses[-1].header.stamp.nanosec / 1e09 + controller_poses[-1].header.stamp.sec) -\
        (controller_poses[0].header.stamp.nanosec /
         1e09 + controller_poses[0].header.stamp.sec)


def getPathLength(path):
    # print(len(path.poses))
    path_length = 0
    x_prev = path.poses[0].pose.position.x
    y_prev = path.poses[0].pose.position.y
    for i in range(1, len(path.poses)):
        x_curr = path.poses[i].pose.position.x
        y_curr = path.poses[i].pose.position.y
        path_length = path_length + \
            math.sqrt((x_curr - x_prev)**2 + (y_curr - y_prev)**2)
        x_prev = x_curr
        y_prev = y_curr
    return path_length


def refactorArray(flat_array: list, n_paths: int, n_controllers: int) -> np.array:
    flat_array = np.asarray(flat_array)
    flat_array = flat_array[np.isfinite(flat_array)]
    flat_array.resize((int(n_paths / n_controllers), n_controllers))
    flat_array = np.transpose(flat_array)
    # Remove np.nan
    for controller_index in range(len(flat_array)):
        flat_array[controller_index] = (flat_array[controller_index])[
            np.isfinite(flat_array[controller_index])]
    return flat_array


def main():

    # Read data
    print("Read data")
    with open(os.getcwd() + '/tasks_controller_results.pickle', 'rb') as f:
        tasks_results = pickle.load(f)

    with open(os.getcwd() + '/tasks_controller_twists.pickle', 'rb') as f:
        tasks_twists = pickle.load(f)

    with open(os.getcwd() + '/tasks_controller_poses.pickle', 'rb') as f:
        tasks_poses = pickle.load(f)

    with open(os.getcwd() + '/controllers.pickle', 'rb') as f:
        controllers = pickle.load(f)

    with open(os.getcwd() + '/local_costmaps.pickle', 'rb') as f:
        tasks_controller_local_costmaps = pickle.load(f)

    with open(os.getcwd() + '/local_costmap_resolution.pickle', 'rb') as f:
        local_costmap_resolution = pickle.load(f)

    with open(os.getcwd() + '/planner_results.pickle', 'rb') as f:
        planner_results = pickle.load(f)

    # Compute metrics

    #
    flat_tasks_results = []
    for results in tasks_results:
        flat_tasks_results += results
    # Planner path lenght
    paths = getPaths(planner_results)
    path_lengths = []

    for path in paths:
        path_lengths.append(getPathLength(path))
    path_lengths = np.asarray(path_lengths)
    # total_paths = len(paths)

    # speeds = getSpeeds(task_twists)

    # Linear Speed
    # Contains all twists, flat
    controllers_twists = []
    for task_twists in tasks_twists:
        for controller_twists in task_twists:
            controllers_twists.append(controller_twists)
    speeds_x = []
    for controller_twists, success in zip(controllers_twists, flat_tasks_results):
        # print("success ",success)
        # print(len(controller_twists))
        if success:
            speeds_x.append(getTaskAvgLinearSpeed(controller_twists))
        else:
            speeds_x.append(np.nan)
    total_paths = len(speeds_x)

    speeds_x = refactorArray(speeds_x, total_paths, len(controllers))

    # Controllet path lenght
    # for task_pose in tasks_poses:
    #    print(list(map(len,task_pose)))
    # print(tasks_poses[1][1])
    controller_paths = getControllerPath(tasks_poses)
    controller_paths_lenght = []
    for controller_path, success in zip(controller_paths, flat_tasks_results):
        # print(success)
        # print("controller_path " ,controller_path)
        if success:
            controller_paths_lenght.append(getPathLength(controller_path))
        else:
            controller_paths_lenght.append(np.nan)
    controller_paths_lenght = refactorArray(
        controller_paths_lenght, total_paths, len(controllers))
    # Distance from obstacles

    # TODO take into account failed navigations (no error is produced)
    # Minimum distance
    controller_obstacles_distances, controller_obstacles_distances_avg, controller_obstacles_distances_std = getObstacleDistances(
        tasks_controller_local_costmaps, local_costmap_resolution)
    controller_obstacles_distances = refactorArray(
        controller_obstacles_distances, total_paths, len(controllers))
    controller_obstacles_distances_avg = refactorArray(
        controller_obstacles_distances_avg, total_paths, len(controllers))
    controller_obstacles_distances_std = refactorArray(
        controller_obstacles_distances_std, total_paths, len(controllers))

    # Smoothness
    controller_ME_linear_jerk = []
    controller_ME_angular_jerk = []
    for controller_twists, success in zip(controllers_twists, flat_tasks_results):
        # print("success ",success)
        # print(len(controller_twists))
        if success:
            me_linear_jerk, me_angular_jerk = getControllerMSJerks(
                controller_twists)
            controller_ME_linear_jerk.append(me_linear_jerk)
            controller_ME_angular_jerk.append(me_angular_jerk)
        else:
            controller_ME_linear_jerk.append(np.nan)
            controller_ME_angular_jerk.append(np.nan)

    controller_ME_linear_jerk = refactorArray(
        controller_ME_linear_jerk, total_paths, len(controllers))
    controller_ME_angular_jerk = refactorArray(
        controller_ME_angular_jerk, total_paths, len(controllers))

    # Single axis array
    flat_poses = []
    for task_poses in tasks_poses:
        for controller_poses in task_poses:
            flat_poses.append(controller_poses)

    task_times = []
    for controller_poses, success in zip(flat_poses, flat_tasks_results):
        # print("success ",success)
        # print(len(controller_poses))
        if success:
            task_times.append(getTaskTimes(controller_poses))
        else:
            task_times.append(np.nan)
    task_times = refactorArray(task_times, total_paths, len(controllers))
    # Remove np.nan
    task_times = task_times[np.isfinite(task_times)]

    # TODO (@enricosutera) this should be transposed
    # TODO  controller as columns header and metrics as rows
    # Generate table
    planner_table = [['Controller',
                      'Success' +
                      '\nrate',
                      'Average linear' +
                      '\nspeed (m/s)',
                      'Average controller' +
                      '\npath len (m) ',
                      'Average time' +
                      '\ntaken(s) ',
                      'Min dist (m)' +
                      '\nfrom obstacle',
                      '--> avg (m)',
                      '--> std (m)',
                      'Avg integrated x jerk' +
                      '\n(m^2/s^6)',
                      'Avg integrated z jerk' +
                      '\n(m/s^6)']]

    for i in range(0, len(controllers)):
        planner_table.append(
            [controllers[i],
             np.sum(tasks_results[i]),
             np.average(speeds_x[i]),
             np.average(controller_paths_lenght[i]),
             np.average(task_times[i]),
             np.min(controller_obstacles_distances[i]),
             np.average(
                controller_obstacles_distances_avg[i]),
             np.average(
                controller_obstacles_distances_std[i]),
             np.average(controller_ME_linear_jerk[i]),
             np.average(controller_ME_angular_jerk[i])])
    # Visualize results
    print("Planned average len: ", np.average(path_lengths))
    print("Total number of tasks: ", len(tasks_results))
    print(tabulate(planner_table, headers="firstrow",
          showindex="always", floatfmt=".3f"))


if __name__ == '__main__':
    main()
