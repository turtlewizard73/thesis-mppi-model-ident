#! /usr/bin/env python3

# Common modules
import logging
import os
import yaml
from typing import Dict, List
from pprint import pformat
from threading import Thread
import time
from dataclasses import dataclass, field
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
import cv2
import numpy as np
import pickle
import glob

# ros imports
import rclpy
import rclpy.clock
import nav2_simple_commander.robot_navigator as nav2

# ROS msg types
from geometry_msgs.msg import PoseStamped

# Custom modules
import rclpy.time
from utils.util_functions import yaw2quat, timing_decorator, newton_diff
import utils.util_nodes as util_nodes
from utils.controller_results import ControllerResult
from utils.controller_metrics import ControllerMetric


@dataclass
class MapData:
    name: str
    path: str
    start: PoseStamped = PoseStamped()
    goal: PoseStamped = PoseStamped()
    resolution: float = 0.05
    origin: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))


class ControllerBenchmark:
    BASE_PATH = os.path.dirname(__file__)
    RESULTS_PATH = os.path.join(BASE_PATH, 'results')
    METRICS_PATH = os.path.join(BASE_PATH, 'metrics')
    COSTMAP_RESOLUTION = 0.05  # TODO: get from costmap

    def __init__(self, logger, config_path: str):
        self.logger = logger
        if os.path.isfile(config_path) is False:
            raise ValueError(f'Invalid path to config file: {config_path}')
        self.config_path = config_path
        self.params: Dict = {}
        self.load_config()

        self.map_data: Dict = {}
        self.load_maps()

        self.nodes: Dict = {}
        self.executors: Dict = {}
        self.executor_threads: Dict = {}
        self._init_nodes()

        self.nav = nav2.BasicNavigator()
        self.nodes_active = False

        # create directories
        if os.path.isdir(ControllerBenchmark.RESULTS_PATH) is False:
            os.makedirs(ControllerBenchmark.RESULTS_PATH)
        if os.path.isdir(ControllerBenchmark.METRICS_PATH) is False:
            os.makedirs(ControllerBenchmark.METRICS_PATH)

        self.results: List[ControllerResult] = []

    def __del__(self):
        self.logger.info('Destructor called.')
        if self.nodes_active:
            self.stop_nodes()

    @timing_decorator(
        lambda self: self.logger.info('Loading config...'),
        lambda self, ex_time: self.logger.info(f'Loaded config in {ex_time:.4f} seconds.'))
    def load_config(self):
        if self.logger.isEnabledFor(logging.DEBUG):
            self.logger.debug(f'Map config file: {self.config_path}')

        with open(self.config_path, 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
            self.logger.debug(f'Config: \n {pformat(data)}')

            self.params['timestamp_format'] = data['timestamp_format']
            self.params['robot_name'] = data['robot_name']
            self.params['planner'] = data['planner']
            self.params['controller'] = data['controller']
            self.params['cmd_vel_topic'] = data['cmd_vel_topic']
            self.params['odom_topic'] = data['odom_topic']
            self.params['costmap_topic'] = data['costmap_topic']
            self.params['mppi_critic_topic'] = data['mppi_critic_topic']

        if self.logger.isEnabledFor(logging.DEBUG):
            self.logger.debug(f'Config: \n {pformat(self.params)}')

    @timing_decorator(
        lambda self: self.logger.info('Loading maps from...'),
        lambda self, ex_time: self.logger.info(f'Loaded maps in {ex_time:.4f} seconds.'))
    def load_maps(self):
        # load information from main config file
        with open(self.config_path, 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
            for map_ in data.get('maps'):

                start = PoseStamped()
                start.header.frame_id = 'map'
                start.pose.position.x = data.get(map_)['start_pose']['x']
                start.pose.position.y = data.get(map_)['start_pose']['y']
                yaw = data.get(map_)['start_pose']['yaw']
                start.pose.orientation = yaw2quat(yaw)

                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.pose.position.x = data.get(map_)['goal_pose']['x']
                goal.pose.position.y = data.get(map_)['goal_pose']['y']
                yaw = data.get(map_)['goal_pose']['yaw']
                goal.pose.orientation = yaw2quat(yaw)

                mapdata = MapData(
                    name=map_,
                    path=os.path.join(ControllerBenchmark.BASE_PATH, data.get(map_)['path']),
                    start=start,
                    goal=goal)

                self.map_data[map_] = mapdata

        # load information from map config files
        # for map_ in self.map_data.keys():
        #     map_config_path = os.path.join(
        #         ControllerBenchmark.BASE_PATH, self.map_data[map_].path)
        #     with open(map_config_path, 'r') as file:
        #         map_config = yaml.safe_load(file)
        #         self.map_data[map_].resolution = map_config['resolution']
        #         self.map_data[map_].origin = np.array(
        #             [[map_config['origin'][0], map_config['origin'][1]]])

        for map_name, map_data in self.map_data.items():
            map_config_path = os.path.join(
                ControllerBenchmark.BASE_PATH, map_data.path)
            with open(map_config_path, 'r', encoding='utf-8') as file:
                map_config = yaml.safe_load(file)
                self.map_data[map_name].resolution = map_config['resolution']
                self.map_data[map_name].origin = np.array(
                    [[map_config['origin'][0], map_config['origin'][1]]])

        if self.logger.isEnabledFor(logging.DEBUG):
            self.logger.debug(f'Maps: \n {pformat(self.map_data)}')

    @timing_decorator(
        lambda self: self.logger.info('Initializing nodes...'),
        lambda self, ex_time: self.logger.info(f'Initialized nodes in {ex_time:.4f} seconds.'))
    def _init_nodes(self):
        rclpy.init()

        # init marker server
        self.marker_server = util_nodes.MarkerServer()
        self.nodes['marker_server'] = self.marker_server
        # init gazebo interface
        self.gazebo_interface = util_nodes.GazeboInterface()
        self.nodes['gazebo_interface'] = self.gazebo_interface
        # init odom subscriber
        self.odom_sub = util_nodes.OdomSubscriber(self.params['odom_topic'])
        self.nodes['odom_sub'] = self.odom_sub
        # init cmd_vel subscriber
        self.cmd_vel_sub = util_nodes.CmdVelSubscriber(self.params['cmd_vel_topic'])
        self.nodes['cmd_vel_sub'] = self.cmd_vel_sub
        # init costmap subscriber
        # self.costmap_sub = util_nodes.CostmapSubscriber(
        #     topic=self.params['costmap_topic'])
        # self.nodes['costmap_sub'] = self.costmap_sub
        # init mppi critic subscriber
        self.mppi_critic_sub = util_nodes.MPPICriticSubscriber(self.params['mppi_critic_topic'])
        self.nodes['mppi_critic_sub'] = self.mppi_critic_sub

        if self.logger.isEnabledFor(logging.DEBUG):
            self.logger.debug(f'Initialized nodes: \n {pformat(self.nodes)}')

    @timing_decorator(
        lambda self: self.logger.info('Launching nodes...'),
        lambda self, ex_time: self.logger.info(f'Launched nodes in {ex_time:.4f} seconds.'))
    def launch_nodes(self):
        for name, node in self.nodes.items():
            sub_executor = rclpy.executors.MultiThreadedExecutor()
            sub_executor.add_node(node)
            self.executors[name] = sub_executor

            sub_thread = Thread(
                target=self._spin_executor, args=(sub_executor,), daemon=True)
            self.executor_threads[name] = sub_thread
            sub_thread.start()

        if self.logger.isEnabledFor(logging.DEBUG):
            self.logger.debug(f'Executors created: \n{pformat(self.executors)}')
            self.logger.debug(f'Sub threads started: \n{pformat(self.executor_threads)}')

        self.nodes_active = True

    def _spin_executor(self, executor):
        try:
            executor.spin()
        except rclpy.executors.ExternalShutdownException:
            self.logger.warn('Executor failed to spin')

    def start_data_collection(self):
        self.logger.info('Starting data collection.')
        for name, node in self.nodes.items():
            if 'sub' in name:
                node.collect_data = True

    def stop_data_collection(self):
        self.logger.info('Stopping data collection.')
        for name, node in self.nodes.items():
            if 'sub' in name:
                node.collect_data = False

    @timing_decorator(
        lambda self: self.logger.info('Stopping nodes, sub executors, threads...'),
        lambda self, ex_time: self.logger.info(f'Stopped nodes in {ex_time:.4f} seconds.'))
    def stop_nodes(self):
        for node in self.nodes.values():
            node.destroy_node()

        for sub_executor in self.executors.values():
            sub_executor.shutdown()

        for sub_thread in self.executor_threads.values():
            sub_thread.join()

    @timing_decorator(
        lambda self: self.logger.info('Running benchmark...'),
        lambda self, ex_time: self.logger.info(f'Benchmark finished in {ex_time:.4f} seconds.'))
    def run_benchmark(self):
        for map_ in self.map_data.keys():
            self.logger.info(f'Changing map to: {map_}')
            self.nav.changeMap(os.path.join(
                ControllerBenchmark.BASE_PATH, self.map_data[map_].path))
            self.nav.clearAllCostmaps()
            time.sleep(0.5)

            # getting start and goal pose
            start: PoseStamped = self.map_data[map_].start
            self.marker_server.publish_markers([start], 'start', (1., 0., 0.))
            goal: PoseStamped = self.map_data[map_].goal
            self.marker_server.publish_markers([goal], 'goal', (0., 1., 0.))

            # resetting world and setting robot pose
            self.gazebo_interface.reset_world()
            self.gazebo_interface.set_entity_state(self.params['robot_name'], start.pose)
            self.nav.setInitialPose(start)
            self.nav.clearAllCostmaps()
            time.sleep(0.5)

            # generating global plan
            planner = self.params['planner']
            self.logger.info(f'Getting global plan for {map_} with planner: {planner}.')
            global_plan = self.nav.getPath(start, goal, planner, use_start=True)
            if global_plan is None:
                self.logger.error('Failed to get global plan.')
                continue

            # running controller
            controller = self.params['controller']
            self.logger.info(
                f'___ Starting controller: {controller}, on map: {map_}. ___')
            start_time: rclpy.clock.Time = rclpy.clock.Clock().now()  # not simulation time
            self.nav.followPath(global_plan, controller_id=controller)
            while not self.nav.isTaskComplete():
                time.sleep(1)
            end_time: rclpy.clock.Time = rclpy.clock.Clock().now()
            time_elapsed: rclpy.clock.Duration = end_time - start_time
            self.logger.info(
                f'___ Controller {controller} on map {map_} finished'
                f' in {time_elapsed.nanoseconds * 1e-9} [s]. ___')

            # collecting results
            self.logger.debug(
                f'Collecting results between {start_time.nanoseconds * 1e-9}'
                f' and {end_time.nanoseconds * 1e-9}.')

            nav_result = True if self.nav.getResult() == nav2.TaskResult.SUCCEEDED else False
            path_xy = np.array([
                (pstamped.pose.position.x, pstamped.pose.position.y) for pstamped in global_plan.poses])
            odom_xy, odom_t = self.odom_sub.get_data_between(
                start_time.nanoseconds, end_time.nanoseconds)
            cmd_vel_xy, cmd_vel_omega, cmd_vel_t = self.cmd_vel_sub.get_data_between(
                start_time.nanoseconds, end_time.nanoseconds)
            critic_scores, critic_scores_t = self.mppi_critic_sub.get_data_between(
                start_time.nanoseconds, end_time.nanoseconds)

            result = ControllerResult(
                controller_name=controller,
                map_name=map_,
                start_time=start_time.nanoseconds,
                time_elapsed=time_elapsed.nanoseconds,
                success=nav_result,
                path_xy=path_xy,
                odom_xy=odom_xy,
                odom_t=odom_t,
                cmd_vel_xy=cmd_vel_xy,
                cmd_vel_omega=cmd_vel_omega,
                cmd_vel_t=cmd_vel_t,
                critic_scores=critic_scores,
                critic_scores_t=critic_scores_t,
                avg_costs=None
            )

            self.results.append(result)

    @timing_decorator(
        lambda self: self.logger.info('Calculating metric...'),
        lambda self, ex_time: self.logger.info(f'Calculated metric in {ex_time:.4f} seconds.'))
    def calculate_metric(self, result: ControllerResult) -> ControllerMetric:
        self.logger.info(f'Calculating metric data for result: {result.map_name}.')
        dist_to_goal = np.linalg.norm(result.path_xy[-1] - result.odom_xy[-1])

        dt = np.mean(result.cmd_vel_t)
        acc_lin = newton_diff(result.cmd_vel_xy[:, 0], dt)
        jerk_lin = newton_diff(acc_lin, dt)
        ms_lin_jerk = np.sqrt(np.mean(jerk_lin**2))

        acc_ang = newton_diff(result.cmd_vel_omega, dt)
        jerk_ang = newton_diff(acc_ang, dt)
        ms_ang_jerk = np.sqrt(np.mean(jerk_ang**2))

        return ControllerMetric(
            controller_name=result.controller_name,
            map_name=result.map_name,
            time_elapsed=result.time_elapsed,
            success=result.success,
            max_linear_velocity=np.max(result.cmd_vel_xy[:, 0]),
            avg_linear_velocity=np.mean(result.cmd_vel_xy[:, 0]),
            max_linear_acceleration=np.max(acc_lin),
            avg_linear_acceleration=np.mean(acc_lin),
            max_angular_acceleration=np.max(acc_ang),
            avg_angular_acceleration=np.mean(acc_ang),
            distance_to_goal=dist_to_goal,
            linear_jerks=jerk_lin,
            ms_linear_jerk=ms_lin_jerk,
            angular_jerks=jerk_ang,
            ms_angular_jerk=ms_ang_jerk)

    def save_result(self, result: ControllerResult) -> None:
        self.logger.info(f'Saving result: {result.map_name}.')

        stamp = time.strftime(self.params['timestamp_format'])
        filename = f'controller_benchmark_result_{stamp}.pickle'
        with open(os.path.join(self.RESULTS_PATH, filename), 'wb+') as f:
            pickle.dump(result, f, pickle.HIGHEST_PROTOCOL)
        self.logger.info(f'Written results to: {filename}')

    def load_result(self, path: str) -> ControllerResult:
        self.logger.info(f'Loading result: {path}.')
        if os.path.isfile(path) is False:
            raise ValueError(f'Invalid path to results file: {path}')

        with open(path, 'rb') as file:
            result: ControllerResult = pickle.load(file)
        self.logger.info(f'Read result: {result.map_name}.')
        return result

    def load_last_result(self) -> ControllerResult:
        results_files = glob.glob(os.path.join(
            self.RESULTS_PATH, 'controller_benchmark_result_*.pickle'))
        latest_file_path = max(results_files, key=os.path.getctime)
        return self.load_result(latest_file_path)

    def plot_result(self, result: ControllerResult) -> Figure:
        self.logger.info('Generating result plot.')

        if self.logger.isEnabledFor(logging.DEBUG):
            pass
            # self.logger.debug(f'Path ({result.path_xy.shape}): \n{result.path_xy}')
            # self.logger.debug(f'Odom ({result.odom_xy.shape}): \n{result.odom_xy}')
            # self.logger.debug(f'Cmd vel ({result.cmd_vel_xy.shape}): \n{result.cmd_vel_xy}')

        fig, (ax_plan, ax_vel, ax_critics) = plt.subplots(
            ncols=1, nrows=3, sharex=False, sharey=False, num=1)

        # map and route vs plan
        # ax_plan.set_title('Plan vs Route')
        ax_plan.set_xlabel('x [m]')
        ax_plan.set_ylabel('y [m]')

        map_path = self.map_data[result.map_name].path.replace('.yaml', '.png')
        map_resolution = self.map_data[result.map_name].resolution
        map_img = cv2.imread(os.path.join(
            ControllerBenchmark.BASE_PATH, map_path), cv2.IMREAD_GRAYSCALE)

        height, width = map_img.shape
        origin_x = self.map_data[result.map_name].origin[0][0]
        rect_x_min = origin_x  # The minimum x coordinate of the rectangle
        rect_x_max = width * map_resolution + origin_x  # The maximum x coordinate of the rectangle
        rect_y_min = - height * map_resolution / 2  # The minimum y coordinate of the rectangle
        rect_y_max = height * map_resolution / 2  # The maximum y coordinate of the rectangle
        # map_img = cv2.resize(map_img, (0, 0), fx=map_resolution, fy=map_resolution, interpolation=cv2.INTER_LINEAR)
        ax_plan.imshow(
            map_img, cmap='gray', aspect='auto',
            interpolation='none', extent=[rect_x_min, rect_x_max, rect_y_min, rect_y_max])

        ax_plan.plot(result.path_xy[:, 0], result.path_xy[:, 1], label='Plan', color='g')
        ax_plan.plot(result.odom_xy[:, 0], result.odom_xy[:, 1], label='Route', color='r')
        ax_plan.grid(visible=True, which='both', axis='both')
        ax_plan.legend()

        # velocity graph
        # ax_vel.set_title('Velocity')
        ax_vel.set_xlabel('Time [s]')
        ax_vel.set_ylabel('Velocity [m/s], [rad/s]')
        ax_vel.plot(result.cmd_vel_t, result.cmd_vel_xy[:, 0], label='Velocity x', color='b')
        ax_vel.plot(result.cmd_vel_t, result.cmd_vel_omega, label='Velocity omega', color='r')
        ax_vel.grid(visible=True, which='both', axis='both')
        ax_vel.legend()

        # costs graph
        # TODO: implement

        # critic scores graph
        # ax_critics.set_title('MPPI Critic scores')
        ax_critics.set_xlabel('Time [s]')
        ax_critics.set_ylabel('MPPI Critic scores')
        for critic, scores in result.critic_scores.items():
            ax_critics.plot(result.critic_scores_t, scores, label=critic)
        ax_critics.grid(visible=True, which='both', axis='both')
        ax_critics.legend()

        return fig

    def plot_metric(
            self, result: ControllerResult, metric: ControllerMetric) -> Figure:
        self.logger.info('Generating metric plot.')

        # plot1: same as result plot
        # plot2: jerks vs time
        fig, (ax_plan, ax_jerk) = plt.subplots(
            ncols=1, nrows=2, sharex=False, sharey=False, num=1)

        # map and route vs plan
        # ax_plan.set_title('Plan vs Route')
        ax_plan.set_xlabel('x [m]')
        ax_plan.set_ylabel('y [m]')

        map_path = self.map_data[result.map_name].path.replace('.yaml', '.png')
        map_resolution = self.map_data[result.map_name].resolution
        map_img = cv2.imread(os.path.join(
            ControllerBenchmark.BASE_PATH, map_path), cv2.IMREAD_GRAYSCALE)

        height, width = map_img.shape
        origin_x = self.map_data[result.map_name].origin[0][0]
        rect_x_min = origin_x  # The minimum x coordinate of the rectangle
        rect_x_max = width * map_resolution + origin_x  # The maximum x coordinate of the rectangle
        rect_y_min = - height * map_resolution / 2  # The minimum y coordinate of the rectangle
        rect_y_max = height * map_resolution / 2  # The maximum y coordinate of the rectangle
        # map_img = cv2.resize(map_img, (0, 0), fx=map_resolution, fy=map_resolution, interpolation=cv2.INTER_LINEAR)
        ax_plan.imshow(
            map_img, cmap='gray', aspect='auto',
            interpolation='none', extent=[rect_x_min, rect_x_max, rect_y_min, rect_y_max])

        plan_label = f'Plan - dist: {metric.distance_to_goal:.2f} [m]'
        ax_plan.plot(result.path_xy[:, 0], result.path_xy[:, 1], label=plan_label, color='g')
        route_label = f'Route - time: {metric.time_elapsed:.2f} [s]'
        ax_plan.plot(result.odom_xy[:, 0], result.odom_xy[:, 1], label=route_label, color='r')
        ax_plan.grid(visible=True, which='both', axis='both')
        ax_plan.legend()

        # jerk graph
        # ax_jerk.set_title('Jerk')
        ax_jerk.set_xlabel('Time [s]')
        ax_jerk.set_ylabel('Jerk [m/s^3], [rad/s^3]')
        ax_jerk.plot(result.cmd_vel_t[1:], metric.linear_jerks, label='Linear jerk', color='b')
        ax_jerk.plot(result.cmd_vel_t[1:], metric.angular_jerks, label='Angular jerk', color='r')

        return fig
