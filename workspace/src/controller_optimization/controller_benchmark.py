#! /usr/bin/env python3

# Common modules
import logging
import os
import yaml
from typing import Dict, List, Tuple
from pprint import pformat
from threading import Thread
import subprocess
import time
from dataclasses import dataclass, field
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
import cv2
import numpy as np
import pickle

# ros imports
import rclpy
import rclpy.clock
import nav2_simple_commander.robot_navigator as nav2

# ROS msg types
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import LoadMap

# Custom modules
import rclpy.time
from utils.util_functions import yaw2quat, timing_decorator, newton_diff
import utils.util_nodes as util_nodes

import constants
from utils.controller_results import ControllerResult
from utils.controller_metrics import ControllerMetric
from utils.controller_parameters import ControllerParameters
from utils.parameter_manager import ParameterManager

BASE_PATH = constants.BASE_PATH
REPO_PATH = constants.REPO_PATH


@dataclass
class MapData:
    """MapData dataclass."""
    name: str
    yaml_path: str
    yaml_path_local: str = ''
    start: PoseStamped = PoseStamped()
    goal: PoseStamped = PoseStamped()
    resolution: float = 0.05  # same as local map
    origin: np.ndarray = field(
        default_factory=lambda: np.empty((0, 2)))  # same as local map


class ControllerBenchmark:
    def __init__(
            self, logger,
            config_path: str,
            result_save_path: str = constants.RESULTS_PATH,
            metric_save_path: str = constants.METRICS_PATH) -> None:
        self.logger = logger
        self.nodes_active = False

        if os.path.isdir(result_save_path) is False:
            os.makedirs(result_save_path)
        self.result_save_path = result_save_path

        if os.path.isdir(metric_save_path) is False:
            os.makedirs(metric_save_path)
        self.metric_save_path = metric_save_path

        if os.path.isfile(config_path) is False:
            raise ValueError(f'Invalid path to config file: {config_path}')

        self.config_path = config_path
        self.params: Dict = {}
        self.mapdata: MapData = None
        self.load_config()

        self.nodes: Dict = {}
        self.executors: Dict = {}
        self.executor_threads: Dict = {}
        self._init_nodes()

        self.nav = nav2.BasicNavigator()
        self.override_navigator()

        self.update_map(self.mapdata)

        self.results: List[ControllerResult] = []

    def __del__(self) -> None:
        self.logger.info('Destructor called.')
        if self.nodes_active is True:
            self.stop_nodes()

    @timing_decorator(
        lambda self: self.logger.info('Loading config...'),
        lambda self, ex_time: self.logger.info(f'Loaded config in {ex_time:.4f} seconds.'))
    def load_config(self) -> None:
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

            map_name = data['map']
            start = PoseStamped()
            start.header.frame_id = 'map'
            start.pose.position.x = data.get(map_name)['start_pose']['x']
            start.pose.position.y = data.get(map_name)['start_pose']['y']
            yaw = data.get(map_name)['start_pose']['yaw']
            start.pose.orientation = yaw2quat(yaw)

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = data.get(map_name)['goal_pose']['x']
            goal.pose.position.y = data.get(map_name)['goal_pose']['y']
            yaw = data.get(map_name)['goal_pose']['yaw']
            goal.pose.orientation = yaw2quat(yaw)

            self.mapdata = MapData(
                name=map_name,
                yaml_path=os.path.join(
                    BASE_PATH, data.get(map_name)['path']),
                start=start,
                goal=goal)

            # local is optional
            if data.get(map_name).get('path_local') is not None:
                self.mapdata.yaml_path_local = os.path.join(
                    BASE_PATH, data.get(map_name)['path_local'])
            else:
                self.mapdata.yaml_path_local = self.mapdata.yaml_path

        map_config_path = os.path.join(
            BASE_PATH, self.mapdata.yaml_path)
        with open(map_config_path, 'r', encoding='utf-8') as file:
            map_config = yaml.safe_load(file)
            self.mapdata.resolution = map_config['resolution']
            self.mapdata.origin = np.array(
                [[map_config['origin'][0], map_config['origin'][1]]])

        if self.logger.isEnabledFor(logging.DEBUG):
            self.logger.debug(f'Config: \n {pformat(self.params)}')
            self.logger.debug(f'Map: \n {pformat(self.mapdata)}')

    @timing_decorator(
        lambda self: self.logger.info('Initializing nodes...'),
        lambda self, ex_time: self.logger.info(f'Initialized nodes in {ex_time:.4f} seconds.'))
    def _init_nodes(self) -> None:
        rclpy.init()
        # init parameter manager
        self.param_manager = ParameterManager('controller_server')
        self.nodes['param_manager'] = self.param_manager
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
        # init mppi critic subscriber
        self.mppi_critic_sub = util_nodes.MPPICriticSubscriber(self.params['mppi_critic_topic'])
        self.nodes['mppi_critic_sub'] = self.mppi_critic_sub

        if self.logger.isEnabledFor(logging.DEBUG):
            self.logger.debug(f'Initialized nodes: \n {pformat(self.nodes)}')

    def override_navigator(self) -> None:
        if self.nav is None:
            raise ValueError('Navigator not initialized.')

        self.nav.change_local_map_srv = self.nav.create_client(
            LoadMap, '/local_map_server/load_map')

        # def new_nav_error(self, msg):
        #     self.get_logger().error(msg)
        #     return False

        # funcType = type(self.nav.error)
        # self.nav.error = funcType(new_nav_error, self.nav, nav2.BasicNavigator)

    def changeGlobalMap(self, map_filepath) -> bool:
        """Change the current global static map in the map server."""
        while not self.nav.change_maps_srv.wait_for_service(timeout_sec=1.0):
            self.nav.info('change global map service not available, waiting...')
        req = LoadMap.Request()
        req.map_url = map_filepath
        future = self.nav.change_maps_srv.call_async(req)
        rclpy.spin_until_future_complete(self.nav, future)
        status = future.result().result
        if status != LoadMap.Response().RESULT_SUCCESS:
            self.nav.error('Change global map request failed!')
            return False
        self.nav.info('Change global map request was successful!')
        return True

    def changeLocalMap(self, map_filepath) -> bool:
        """Change the current local static map in the map server."""
        while not self.nav.change_local_map_srv.wait_for_service(timeout_sec=1.0):
            self.nav.info('change local map service not available, waiting...')
        req = LoadMap.Request()
        req.map_url = map_filepath
        future = self.nav.change_local_map_srv.call_async(req)
        rclpy.spin_until_future_complete(self.nav, future)
        status = future.result().result
        if status != LoadMap.Response().RESULT_SUCCESS:
            self.nav.error('Change local map request failed!')
            return False
        self.nav.info('Change local map request was successful!')
        return True

    def update_map(self, mapdata: MapData = None) -> bool:
        mapdata = mapdata if mapdata is not None else self.mapdata

        self.logger.info(f'Changing global map to: {mapdata.yaml_path}')
        res = self.changeGlobalMap(mapdata.yaml_path)
        if res is False:
            self.logger.error('Failed to change global map')
            return False

        local_map_path = mapdata.yaml_path_local if mapdata.yaml_path_local != '' else mapdata.yaml_path
        self.logger.info(f'Changing local map to: {local_map_path}')
        res = self.changeLocalMap(local_map_path)
        if res is False:
            self.logger.error('Failed to change local map')
            return False

        self.nav.clearAllCostmaps()
        time.sleep(0.5)
        return True

    def update_parameters(self, parameters: ControllerParameters) -> bool:
        self.logger.info(f'Setting parameters: {parameters}')
        controller_name = parameters.controller_name
        critics_dict = {}
        for critic in parameters.critics:
            critics_dict.update({
                f'{controller_name}.{critic.name}.cost_weight': critic.cost_weight,
                f'{controller_name}.{critic.name}.cost_power': critic.cost_power
            })

        params_set = self.param_manager.set_parameters(parameters.to_dict())
        if params_set is False:
            self.logger.error('Failed to set parameters')
            return False
        return True

    @timing_decorator(
        lambda self: self.logger.info('Launching nodes...'),
        lambda self, ex_time: self.logger.info(f'Launched nodes in {ex_time:.4f} seconds.'))
    def launch_nodes(self):
        try:
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

        except Exception as e:
            raise RuntimeError(f'Failed to launch nodes: {e}')

    def _spin_executor(self, executor):
        try:
            executor.spin()
        except rclpy.executors.ExternalShutdownException:
            self.logger.warn('Executor failed to spin')

    def _start_data_collection(self):
        for name, node in self.nodes.items():
            if 'sub' in name:
                node.collect_data = True

    def _stop_data_collection(self):
        for name, node in self.nodes.items():
            if 'sub' in name:
                node.collect_data = False

    @timing_decorator(
        lambda self: self.logger.info('Stopping nodes, sub executors, threads...'),
        lambda self, ex_time: self.logger.info(f'Stopped nodes in {ex_time:.4f} seconds.'))
    def stop_nodes(self):
        try:
            for node in self.nodes.values():
                node.destroy_node()

            for sub_executor in self.executors.values():
                sub_executor.shutdown()

            for sub_thread in self.executor_threads.values():
                sub_thread.join()

            self.nodes_active = False

        except Exception as e:
            raise RuntimeError(f'Failed to stop nodes: {e}')

    @timing_decorator(
        lambda self: self.logger.info('Checking if nodes are active...'),
        lambda self, ex_time: self.logger.info(f'Checked nodes in {ex_time:.4f} seconds.')
    )
    def check_launch_nodes_active(self) -> Tuple[bool, str]:
        running_nodes = []
        try:
            # Run `ros2 node list` and decode output
            result = subprocess.run(
                ["ros2", "node", "list"], capture_output=True, text=True, check=True)
            # Split the output by lines to get each node name
            running_nodes = result.stdout.strip().splitlines()
            if self.logger.isEnabledFor(logging.DEBUG):
                self.logger.debug(f'Running nodes: {running_nodes}')
        except subprocess.CalledProcessError as e:
            return False, e

        expected = [
            "/controller_server",
            "/gazebo",
            "/gazebo_ros_state",
            "/global_costmap/global_costmap",
            "/lifecycle_manager",
            "/local_costmap/local_costmap",
            "/local_map_server",
            "/map_odom_broadcaster",
            "/map_server",
            "/planner_server",
            "/robot_state_publisher",
            # "/rviz",
            # "/rviz_navigation_dialog_action_client",
            # "/transform_listener_impl_5dd763758dc0",
            # "/transform_listener_impl_61933a70ffb0",
            # "/transform_listener_impl_79ac64003470",
            "/turtlebot3_diff_drive",
            "/turtlebot3_imu",
            "/turtlebot3_joint_state",
            "/turtlebot3_laserscan"
        ]

        missing_nodes = [node for node in expected if node not in running_nodes]

        if len(missing_nodes) > 0:
            return False, f'Missing nodes: {missing_nodes}'

        return True, 'All nodes are active'

    @timing_decorator(
        lambda self: self.logger.info('Running benchmark...'),
        lambda self, ex_time: self.logger.info(f'Benchmark finished in {ex_time:.4f} seconds.'))
    def run_benchmark(
            self, run_uid: str = '', timeout: float = None, store_result: bool = False) -> ControllerResult:
        # _____________________ BENCHMARK SETUP _____________________
        mapdata = self.mapdata
        result = ControllerResult(
            controller_name=self.params['controller'],
            map_name=self.mapdata.name,
            uid=run_uid)

        # check if nodes are active
        if self.nodes_active is False:
            msg = 'Nodes are not active'
            result.success = False
            result.status_msg = msg
            self.logger.error('Failed to run benchmark: %s', msg)
            return result

        # getting start and goal pose
        self.logger.info('Generating robot start and goal pose.')
        start: PoseStamped = mapdata.start
        self.marker_server.publish_markers([start], 'start', (1., 0., 0.))
        goal: PoseStamped = mapdata.goal
        self.marker_server.publish_markers([goal], 'goal', (0., 1., 0.))

        # start data collection
        self.logger.info('Starting data collection.')
        self._start_data_collection()

        # resetting world and setting robot pose
        # TODO: check if successful (not really priority)
        self.logger.info('Resetting world and setting robot pose.')
        self.gazebo_interface.reset_world()
        self.gazebo_interface.set_entity_state(self.params['robot_name'], start.pose)
        self.nav.setInitialPose(start)
        self.nav.clearAllCostmaps()
        time.sleep(0.5)

        # generating global plan
        planner = self.params['planner']
        self.logger.info(
            f'Getting global plan for {mapdata.name} with planner: {planner}.')
        global_plan = self.nav.getPath(start, goal, planner, use_start=True)
        if global_plan is None:
            msg = 'Failed to get global plan'
            result.success = False
            result.status_msg = msg
            self.logger.error('Failed to run benchmark: %s', msg)
            return result
        path_xy = np.array([
            (ps.pose.position.x, ps.pose.position.y) for ps in global_plan.poses])

        # TODO: quaternion to radian, necessary?
        path_omega = np.array([
            ps.pose.orientation.z for ps in global_plan.poses])

        # TODO: calculate timeout based on progress
        # distances = np.sqrt(np.sum(np.diff(path_xy, axis=0)**2, axis=1))
        # global_plan_length = np.sum(distances)
        # # calculate max time with min velocity
        # max_time = global_plan_length / parameters.min_linear_velocity

        # _____________________ BENCHMARK RUN _____________________
        controller = self.params['controller']
        self.logger.info(
            f'___ Starting controller: {controller}, on map: {mapdata.name}. ___')
        start_time: rclpy.clock.Time = rclpy.clock.Clock().now()  # not simulation time
        action_response = self.nav.followPath(global_plan, controller_id=controller)
        if action_response is False:
            msg = 'Failed to send action goal'
            result.success = False
            result.status_msg = msg
            self.logger.error('Failed to run benchmark: %s', msg)
            return result

        while not self.nav.isTaskComplete():
            if timeout is not None:
                run_time = (rclpy.clock.Clock().now() - start_time).nanoseconds * 1e-9
                if run_time > timeout:
                    self.nav.cancelTask()
                    msg = f'Timeout reached: {run_time} > {timeout}'
                    result.success = False
                    result.status_msg = msg
                    self.logger.error('Failed to run benchmark: %s', msg)
                    return result
            time.sleep(1)

        end_time: rclpy.clock.Time = rclpy.clock.Clock().now()
        time_elapsed: rclpy.clock.Duration = end_time - start_time
        time_elapsed = time_elapsed.nanoseconds * 1e-9
        self.logger.info(
            f'___ Controller {controller} on map {mapdata.name} finished'
            f' in {time_elapsed} [s]. ___')

        # _____________________ EVALUATION & CLEANUP _____________________
        self._stop_data_collection()

        # collecting results
        self.logger.debug(
            f'Collecting results between {start_time.nanoseconds * 1e-9}'
            f' and {end_time.nanoseconds * 1e-9}.')

        if self.nav.getResult() != nav2.TaskResult.SUCCEEDED:
            msg = 'Failed to reach goal, nav fault'
            result.success = False
            result.status_msg = msg
            self.logger.error('Failed to run benchmark: %s', msg)
            return result

        odom_xy, odom_omega, odom_t = self.odom_sub.get_data_between(
            start_time.nanoseconds, end_time.nanoseconds)
        cmd_vel_xy, cmd_vel_omega, cmd_vel_t = self.cmd_vel_sub.get_data_between(
            start_time.nanoseconds, end_time.nanoseconds)
        critic_scores, critic_scores_t = self.mppi_critic_sub.get_data_between(
            start_time.nanoseconds, end_time.nanoseconds)

        # getting local costmap as array
        costmap_msg: Costmap = self.nav.getLocalCostmap()
        size_x = costmap_msg.metadata.size_x
        size_y = costmap_msg.metadata.size_y
        costmap_array = np.array(costmap_msg.data, dtype=np.uint8).reshape((size_y, size_x))
        costmap_array = np.flip(costmap_array, 0)

        map_resolution = costmap_msg.metadata.resolution
        origin_x = costmap_msg.metadata.origin.position.x
        origin_y = costmap_msg.metadata.origin.position.y
        path_costs = []
        for x, y in path_xy:
            x_idx = int((x - origin_x) / map_resolution)
            y_idx = int((y - origin_y) / map_resolution)
            path_costs.append(costmap_msg.data[y_idx * size_x + x_idx])

        result.start_time = start_time.nanoseconds * 1e-9
        result.time_elapsed = time_elapsed
        result.success = True
        result.status_msg = 'Success'
        result.path_xy = path_xy
        result.path_omega = path_omega
        result.odom_xy = odom_xy
        result.odom_omega = odom_omega
        result.odom_t = odom_t
        result.cmd_vel_xy = cmd_vel_xy
        result.cmd_vel_omega = cmd_vel_omega
        result.cmd_vel_t = cmd_vel_t
        result.critic_scores = critic_scores
        result.critic_scores_t = critic_scores_t
        result.costmap = costmap_array
        result.path_costs = np.array(path_costs)

        if store_result is True:
            self.results.append(result)

        return result

    @timing_decorator(
        lambda self: self.logger.info('Running batch...'),
        lambda self, ex_time: self.logger.info(f'Batch finished in {ex_time:.4f} seconds.'))
    def run_batch(self, parameters: list[dict]) -> list[dict[int, bool]]:
        # make a list to store if the benchmark was successful
        results: list[dict[int, bool]] = []

        for i, params in enumerate(parameters):
            success = self.run_benchmark(parameters=params, store_results=True)
            results.append({i: success})

        return results

    @timing_decorator(
        lambda self: self.logger.info('Calculating metric...'),
        lambda self, ex_time: self.logger.info(f'Calculated metric in {ex_time:.4f} seconds.'))
    def calculate_metric(self, result: ControllerResult) -> ControllerMetric:
        distance_to_goal = np.linalg.norm(result.path_xy[-1] - result.odom_xy[-1])
        angle_to_goal = result.path_omega[-1] - result.odom_omega[-1]

        dt = np.mean(np.diff(result.cmd_vel_t))
        acc_lin = newton_diff(result.cmd_vel_xy[:, 0], dt)
        jerk_lin = newton_diff(acc_lin, dt)
        rms_lin_jerk = np.sqrt(np.mean(jerk_lin**2))

        acc_ang = newton_diff(result.cmd_vel_omega, dt)
        jerk_ang = newton_diff(acc_ang, dt)
        rms_ang_jerk = np.sqrt(np.mean(jerk_ang**2))

        return ControllerMetric(
            controller_name=result.controller_name,
            map_name=result.map_name,
            uid=result.uid,

            time_elapsed=result.time_elapsed,
            success=result.success,
            distance_to_goal=distance_to_goal,
            angle_to_goal=angle_to_goal / np.pi * 180,  # rad to deg

            path_xy=result.path_xy,

            linear_velocity=result.cmd_vel_xy[:, 0],
            avg_linear_velocity=np.mean(result.cmd_vel_xy[:, 0]),
            max_linear_velocity=np.max(result.cmd_vel_xy[:, 0]),
            rms_linear_velocity=np.sqrt(np.mean(result.cmd_vel_xy[:, 0]**2)),

            linear_acceleration=acc_lin,
            avg_linear_acceleration=np.mean(acc_lin),
            max_linear_acceleration=np.max(acc_lin),
            rms_linear_acceleration=np.sqrt(np.mean(acc_lin**2)),

            max_angular_acceleration=np.max(acc_ang),
            avg_angular_acceleration=np.mean(acc_ang),

            linear_jerks=jerk_lin,
            rms_linear_jerk=rms_lin_jerk,
            angular_jerks=jerk_ang,
            rms_angular_jerk=rms_ang_jerk,

            path_costs=result.path_costs,
            sum_of_costs=np.sum(result.path_costs),
            avg_cost=np.mean(result.path_costs),
            rms_cost=np.sqrt(np.mean(result.path_costs**2))
        )

    def save_result(
            self, result: ControllerResult,
            output_dir: str = '', uid: str = '') -> str:
        self.logger.info(f'Saving result: {result.map_name}.')
        output_dir = output_dir if output_dir != '' else \
            self.result_save_path

        if os.path.isdir(output_dir) is False:
            os.makedirs(output_dir)

        uid = uid if uid != '' else time.strftime(self.params['timestamp_format'])
        filename = f'result_{uid}.pickle'

        save_path = os.path.join(output_dir, filename)
        with open(save_path, 'wb+') as f:
            pickle.dump(result, f, pickle.HIGHEST_PROTOCOL)

        # save in the output_dir the path to the result
        with open(os.path.join(output_dir, 'last_result.txt'), 'w') as f:
            f.write(save_path)

        self.logger.info(f'Written results to: {save_path}')
        return save_path

    def load_result(self, path: str) -> ControllerResult:
        self.logger.info(f'Loading result: {path}.')
        if not path.endswith('.pickle'):
            raise ValueError('Path must end with .pickle')

        if os.path.isfile(path) is False:
            raise FileNotFoundError(f'Invalid path to results file: {path}')

        with open(path, 'rb') as file:
            result: ControllerResult = pickle.load(file)
        self.logger.info(f'Read result: {result.map_name}.')
        return result

    def load_last_result(self) -> ControllerResult:
        latest_result_txt = os.path.join(
            self.result_save_path, 'last_result.txt')

        if os.path.isfile(latest_result_txt) is False:
            raise FileNotFoundError(f'File not found: {latest_result_txt}')

        with open(os.path.join(ControllerBenchmark.RESULTS_PATH, 'last_result.txt'), 'r') as f:
            last_result_path = f.read().strip()

        if os.path.isfile(last_result_path) is False:
            raise FileNotFoundError(f'Invalid path to results file: {last_result_path}')

        return self.load_result(last_result_path)

    def save_metric(
            self, metric: ControllerMetric,
            output_dir: str = '', uid: str = '') -> str:
        self.logger.info(f'Saving result: {metric.map_name}.')
        output_dir = output_dir if output_dir != '' else self.metric_save_path

        if os.path.isdir(output_dir) is False:
            os.makedirs(output_dir)

        uid = uid if uid != '' else time.strftime(self.params['timestamp_format'])
        filename = f'result_{uid}.pickle'

        save_path = os.path.join(output_dir, filename)
        with open(save_path, 'wb+') as f:
            pickle.dump(metric, f, pickle.HIGHEST_PROTOCOL)

        # save in the output_dir the path to the result
        with open(os.path.join(output_dir, 'last_metric.txt'), 'w') as f:
            f.write(save_path)

        self.logger.info(f'Written results to: {save_path}')
        return save_path

    def load_metric(self, path: str) -> ControllerMetric:
        self.logger.info(f'Loading metric: {path}.')
        if not path.endswith('.pickle'):
            raise ValueError('Path must end with .pickle')

        if os.path.isfile(path) is False:
            raise FileNotFoundError(f'Invalid path to results file: {path}')

        with open(path, 'rb') as file:
            metric: ControllerMetric = pickle.load(file)
        self.logger.info(f'Read metric: {metric.map_name}.')
        return metric

    def load_last_metric(self) -> ControllerMetric:
        latest_metric_txt = os.path.join(self.metric_save_path, 'last_metric.txt')

        if os.path.isfile(latest_metric_txt) is False:
            raise FileNotFoundError(f'File not found: {latest_metric_txt}')

        with open(latest_metric_txt, 'r') as f:
            last_metric_path = f.read().strip()

        if os.path.isfile(last_metric_path) is False:
            raise FileNotFoundError(f'Invalid path to metric file: {last_metric_path}')

        return self.load_metric(last_metric_path)

    def plot_result(self, result: ControllerResult) -> Figure:
        self.logger.info('Generating result plot.')

        if self.logger.isEnabledFor(logging.DEBUG):
            self.logger.debug(f'Path ({result.path_xy.shape}): \n{result.path_xy}')
            self.logger.debug(f'Odom ({result.odom_xy.shape}): \n{result.odom_xy}')
            self.logger.debug(f'Cmd vel ({result.cmd_vel_xy.shape}): \n{result.cmd_vel_xy}')

        fig, (ax_plan, ax_vel, ax_critics) = plt.subplots(
            ncols=1, nrows=3, sharex=False, sharey=False, num=1)

        # map and route vs plan
        # ax_plan.set_title('Plan vs Route')
        ax_plan.set_xlabel('x [m]')
        ax_plan.set_ylabel('y [m]')

        map_path = self.mapdata.yaml_path.replace('.yaml', '.png')
        map_resolution = self.mapdata.resolution
        map_img = cv2.imread(os.path.join(
            BASE_PATH, map_path), cv2.IMREAD_GRAYSCALE)

        height, width = map_img.shape
        origin_x = self.mapdata.origin[0][0]
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
            ncols=1, nrows=2, sharex=False, sharey=False, num=2)

        # map and route vs plan
        # ax_plan.set_title('Plan vs Route')
        ax_plan.set_xlabel('x [m]')
        ax_plan.set_ylabel('y [m]')

        map_path = self.mapdata.yaml_path.replace('.yaml', '.png')
        map_resolution = self.mapdata.resolution
        map_img = cv2.imread(os.path.join(
            BASE_PATH, map_path), cv2.IMREAD_GRAYSCALE)

        height, width = map_img.shape
        origin_x = self.mapdata.origin[0][0]
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
        time_s = metric.time_elapsed
        route_label = f'Route - time: {time_s:.2f} [s]'
        ax_plan.plot(result.odom_xy[:, 0], result.odom_xy[:, 1], label=route_label, color='r')
        ax_plan.grid(visible=True, which='both', axis='both')
        ax_plan.legend()

        # jerk graph
        # ax_jerk.set_title('Jerk')
        cmd_vel_t_s = result.cmd_vel_t
        ax_jerk.set_xlabel('Time [s]')
        ax_jerk.set_ylabel('Jerk [m/s^3], [rad/s^3]')
        label = f'Linear jerk - RMS: {metric.rms_linear_jerk:.2f} [m/s^3]'
        ax_jerk.plot(cmd_vel_t_s, metric.linear_jerks, label=label, color='b')
        label = f'Angular jerk - RMS: {metric.rms_angular_jerk:.2f} [rad/s^3]'
        ax_jerk.plot(cmd_vel_t_s, metric.angular_jerks, label=label, color='r')
        ax_jerk.grid(visible=True, which='both', axis='both')
        ax_jerk.legend()

        # TODO: make nicer data vis
        self.logger.info(metric.to_table_string())

        return fig
