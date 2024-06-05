#! /usr/bin/env python3
import sys
sys.path.append("/home/turtlewizard/thesis-mppi-model-ident/workspace/install/controller_benchmark/lib/controller_benchmark")
# TODO: import discrete module somehow otherwise

# Common modules
import os
from threading import Thread
import logging
import time
import yaml
import pickle
import cv2
import numpy as np
from concurrent.futures import ProcessPoolExecutor, as_completed
from typing import List, Dict

# ROS modules
import rclpy
from rclpy.clock import Clock
from rclpy.time import Time as RosTime
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ROS msg definitions
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from nav2_mppi_controller.msg import CriticScores

# Custom modules
from discrete import FastDiscreteFrechetMatrix, euclidean as euclidean_dist
from utils import (
    ControllerResult, ControllerMetric,
    MarkerServer, ListenerBase, GazeboInterface)

logging.basicConfig(level=logging.INFO)


class ControllerBenchmark:
    BASE_PATH = os.path.dirname(os.path.dirname(__file__))
    RESULTS_DIR = os.path.join(BASE_PATH, 'results')
    CONFIG_DIR = os.path.join(BASE_PATH, 'config')
    MAP_DIR = os.path.join(BASE_PATH, 'maps')

    def __init__(self, config: str = None) -> None:
        rclpy.init()

        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger('controller_benchmark')

        self.nav = BasicNavigator()
        self.fdfdm = FastDiscreteFrechetMatrix(euclidean_dist)

        # Setting up parameters
        config = config if config is not None else 'controller_benchmark.yaml'
        self.config_file = os.path.join(self.CONFIG_DIR, config)
        self.params = yaml.safe_load(open(self.config_file))
        self.params['map'] = yaml.safe_load(open(
            os.path.join(self.MAP_DIR, self.params['map_yaml'])))
        self.map_as_img = cv2.imread(
            os.path.join(self.MAP_DIR, self.params['map']['image']), cv2.IMREAD_GRAYSCALE)

        self.params['robot_name'] = 'turtlebot3_waffle'

        self.results: List[ControllerResult] = []

        # Launching nodes
        self.marker_server_node = MarkerServer()
        self.gazebo_interface_node = GazeboInterface()
        self.cmd_vel_sub_node = ListenerBase(
            'cmd_vel_subscriber', self.params['cmd_vel_topic'], Twist)
        self.odom_sub_node = ListenerBase(
            'odom_subscriber', self.params['odom_topic'], Odometry)
        self.costmap_sub_node = ListenerBase(
            'costmap_sub', self.params['costmap_topic'], OccupancyGrid)
        self.mppi_critics_sub_node = ListenerBase(
            'mppi_critics_sub', self.params['mppi_critic_topic'], CriticScores)

        self.sub_executor = MultiThreadedExecutor()
        self.sub_executor.add_node(self.marker_server_node)
        self.sub_executor.add_node(self.gazebo_interface_node)
        self.sub_executor.add_node(self.cmd_vel_sub_node)
        self.sub_executor.add_node(self.odom_sub_node)
        self.sub_executor.add_node(self.costmap_sub_node)
        self.sub_executor.add_node(self.mppi_critics_sub_node)

        self.sub_executor_thread = Thread(
            target=self.spin_executor, args=(self.sub_executor, ), daemon=True)
        self.sub_executor_thread.start()
        time.sleep(1)  # wait for nodes to start

    def update_map(self, map_yaml: str) -> None:
        self.params['map_yaml'] = map_yaml
        self.params['map'] = yaml.safe_load(open(
            os.path.join(self.MAP_DIR, self.params['map_yaml'])))
        self.map_as_img = cv2.imread(
            os.path.join(self.MAP_DIR, self.params['map']['image']), cv2.IMREAD_GRAYSCALE)

    def __del__(self):
        self.marker_server_node.destroy_node()
        self.gazebo_interface_node.destroy_node()
        self.cmd_vel_sub_node.destroy_node()
        self.odom_sub_node.destroy_node()
        self.costmap_sub_node.destroy_node()
        self.mppi_critics_sub_node.destroy_node()

        self.sub_executor.shutdown()
        time.sleep(1)
        self.sub_executor_thread.join()
        rclpy.shutdown()

    def spin_executor(self, executor):
        try:
            executor.spin()
        except rclpy.executors.ExternalShutdownException:
            self.logger.info('Executor failed to spin')
            pass

    def setup_test(self, map_yaml: str) -> None:
        os.makedirs(self.RESULTS_DIR, exist_ok=True)

        # check if monitoring nodes are active
        names = []
        names.append(self.marker_server_node.get_name())
        names.append(self.gazebo_interface_node.get_name())
        names.append(self.cmd_vel_sub_node.get_name())
        names.append(self.odom_sub_node.get_name())
        names.append(self.costmap_sub_node.get_name())
        names.append(self.mppi_critics_sub_node.get_name())

        if len(names) < 6:
            self.logger.error(f'Not all monitoring nodes are active, {names}')
            return
        else:
            self.logger.info(f'All monitoring nodes are active: {names}')

        # Change map
        if map_yaml is None or map_yaml == '' or map_yaml == self.params['map_yaml']:
            map_yaml = self.params['map_yaml']
        else:
            if os.path.isfile(os.path.join(self.MAP_DIR, map_yaml)):
                self.update_map(map_yaml)
            else:
                self.logger.error(f'Invalid map: {map_yaml}')
                return

        self.logger.info(f'Changing map to: {map_yaml}')
        self.nav.changeMap(os.path.join(self.MAP_DIR, self.params['map_yaml']))
        self.nav.clearAllCostmaps()
        time.sleep(2)

    def run_test(self, plan: Path, controller: str,
                 start: PoseStamped, goal: PoseStamped) -> ControllerResult:
        # for a single controller with single goal
        self.gazebo_interface_node.reset_world()
        self.gazebo_interface_node.set_entity_state(
            self.params['robot_name'], start.pose, None)
        self.marker_server_node.publish_markers([start], 'start', (1., 0., 0.))
        self.marker_server_node.publish_markers([goal], 'goal', (0., 1., 0.))
        self.nav.setInitialPose(start)
        self.nav.clearAllCostmaps()
        time.sleep(0.5)

        start_time = Clock().now()
        self.nav.followPath(plan, controller_id=controller)
        self.logger.info(
            f'Starting controller: {controller}, '
            f'at: {start_time.seconds_nanoseconds}')
        while not self.nav.isTaskComplete():
            time.sleep(1)
        end_time = Clock().now()
        nav_result = True if self.nav.getResult() == TaskResult.SUCCEEDED else False
        self.logger.info(
            f'{controller} finished with result: {nav_result}, '
            f'at: {end_time.seconds_nanoseconds},')

        result = ControllerResult(
            controller_name=controller,
            # plan_idx=i,
            plan=plan,
            # run=j,
            start_time=start_time.nanoseconds,
            end_time=end_time.nanoseconds,
            result=nav_result,
            odom=self.odom_sub_node.get_msgs(start_time, end_time),
            cmd_vel=self.cmd_vel_sub_node.get_msgs(start_time, end_time),
            costmaps=self.costmap_sub_node.get_msgs(start_time, end_time),
            critic_scores=self.mppi_critics_sub_node.get_msgs(start_time, end_time)
        )

        self.gazebo_interface_node.reset_world()

        self.results.append(result)
        return result

    def save_results(
            self, results: List[ControllerResult] = None, delete: bool = False) -> None:
        self.logger.info("Write Results...")
        results = results if results is not None else self.results

        stamp = time.strftime(self.params['timestamp_format'])
        filename = f'/controller_benchmark_results_{stamp}.pickle'
        # filename = '/controller_benchmark_results.pickle'
        with open(self.RESULTS_DIR + filename, 'wb+') as f:
            pickle.dump(results, f, pickle.HIGHEST_PROTOCOL)
        self.logger.info(f'Written results to: {self.RESULTS_DIR + filename}')

        if delete is True:
            self.logger.info('Deleting results...')
            self.results = []

    def calculate_metrics(
            self, results: List[ControllerResult] = None) -> List[ControllerMetric]:
        results = results if results is not None else self.results

        with ProcessPoolExecutor() as executor:
            futures = {executor.submit(self.calculate_metric_, r): r for r in results}
            for future in as_completed(futures):
                result = futures[future]
                try:
                    metric = future.result()
                    if metric:
                        self.metrics.append(metric)
                        self.logger.info(f'{metric.controller_name} finished')
                except Exception as e:
                    self.logger.error(f'Error processing {result.controller_name}: {e}')

    def calculate_metric_(self, result: ControllerResult) -> ControllerMetric:
        distance_to_goal = np.linalg.norm(
            self.get_xy(result.plan.poses[-1]) - self.get_xy(result.odom[-1]))
        time = self.time_diff(result.cmd_vel[0].header.stamp, result.cmd_vel[-1].header.stamp)
        plan_length = self.dist_fullplan(result.plan.poses)
        traversed_length = self.dist_fullplan(result.odom)
        completion_ratio = (traversed_length - plan_length) / plan_length

        dt = np.array(
            [RosTime.from_msg(t.header.stamp).nanoseconds * 1e-9 for t in result.cmd_vel])
        dt_array = dt - dt[0]
        dt = np.mean(np.diff(dt_array))  # TODO: make dt stay an array differences are big
        #  0.05612588 0.0438931  0.0548203  0.04536557 0.05174136 0.05203557
        vel_lin = np.array([v.twist.linear.x for v in result.cmd_vel])
        vel_ang = np.array([v.twist.angular.z for v in result.cmd_vel])

        acc_lin = self.newton_diff(vel_lin, dt)
        jerk_lin = self.newton_diff(acc_lin, dt)

        acc_ang = self.newton_diff(vel_ang, dt)
        jerk_ang = self.newton_diff(acc_ang, dt)

        # fdfdm = FastDiscreteFrechetMatrix(euclidean_dist)
        plan = np.array([self.get_xy(p) for p in result.plan.poses])
        route = np.array([self.get_xy(p) for p in result.odom])
        print(f"calculating frechet dist for {result.controller_name}")
        frechet_dist = self.fdfdm.distance(plan, route)
        print(f"frechet dist for {result.controller_name} is {frechet_dist}")

        if result.critic_scores is not None:
            critic_scores_: Dict[str, List[float]] = {}
            for cs in result.critic_scores:
                for name, score in zip(cs.critic_names, cs.critic_scores):
                    if name.data not in critic_scores_:
                        critic_scores_[name.data] = []
                    critic_scores_[name.data].append(score.data)
        else:
            critic_scores_ = None

        metric = ControllerMetric(
            controller_name=result.controller_name,
            plan_idx=result.plan_idx,
            result=result.result,
            distance_to_goal=distance_to_goal,
            time=time,
            plan_length=plan_length,
            traversed_length=traversed_length,
            completion_ratio=completion_ratio,
            frechet_dist=frechet_dist,
            avg_linear_vel=np.mean(vel_lin),
            avg_linear_acc=np.mean(acc_lin),
            ms_linear_jerk=np.sqrt(np.mean(jerk_lin**2)),
            avg_angular_vel=np.mean(vel_ang),
            avg_angular_acc=np.mean(acc_ang),
            ms_angular_jerk=np.sqrt(np.mean(jerk_ang**2)),
            plan_poses=plan,
            route_poses=route,
            time_steps=dt_array,
            linear_acc=acc_lin,
            linear_jerks=jerk_lin,
            angular_acc=acc_ang,
            angular_jerks=jerk_ang,
            critic_scores=critic_scores_,
        )
        return metric

    def get_xy(self, p) -> np.ndarray:
        if isinstance(p, PoseStamped):
            return np.array([p.pose.position.x, p.pose.position.y])
        elif isinstance(p, PoseWithCovarianceStamped):
            return np.array([p.pose.pose.position.x, p.pose.pose.position.y])
        elif isinstance(p, Odometry):
            return np.array([p.pose.pose.position.x, p.pose.pose.position.y])
        else:
            raise ValueError('Unknown type')

    def time_diff(self, t1: RosTime, t2: RosTime) -> float:
        return np.abs(
            RosTime.from_msg(t1).nanoseconds - RosTime.from_msg(t2).nanoseconds) / 1e9

    def dist_fullplan(self, plan: list) -> float:
        if len(plan) < 2:
            return 0.0
        xy_positions = np.array([self.get_xy(p) for p in plan])
        deltas = np.diff(xy_positions, axis=0)
        return np.sum(np.linalg.norm(deltas, axis=1))

    def newton_diff(self, y: np.ndarray, dt: float):
        derivative = np.zeros_like(y)
        # Central differences
        derivative[2:-2] = (-y[4:] + 8*y[3:-1] - 8*y[1:-3] + y[0:-4]) / (12 * dt)
        # Use lower-order methods at the boundaries
        derivative[0] = (y[1] - y[0]) / dt  # Forward difference for the first point
        derivative[1] = (y[2] - y[0]) / (2 * dt)  # Central difference for the second point
        derivative[-2] = (y[-1] - y[-3]) / (2 * dt)  # Central difference for the second last point
        derivative[-1] = (y[-1] - y[-2]) / dt  # Backward difference for the last point
        return derivative


def main():
    maps = [
        'corridor_empty.yaml',
        'corridor_mid_obstacles.yaml',
        'corridor_side_obstacles.yaml',
    ]

    start_pose = PoseStamped()
    start_pose.header.frame_id = 'map'
    start_pose.pose.position.x = 0.0
    start_pose.pose.position.y = 0.0

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.position.x = 8.0
    goal_pose.pose.position.y = 0.0

    # TODO: get conf from enjoy config
    planner = 'GridBased'
    controller = 'Maneuver'

    controller_benchmark = ControllerBenchmark()
    for map_ in maps:
        controller_benchmark.setup_test(map_)

        plan = controller_benchmark.nav.getPath(
            start_pose, goal_pose, planner, use_start=True)

        _ = controller_benchmark.run_test(plan, controller, start_pose, goal_pose)

    # Save results
    controller_benchmark.save_results()

    # Plot results


if __name__ == '__main__':
    main()
