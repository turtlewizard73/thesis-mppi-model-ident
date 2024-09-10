#! /usr/bin/env python3

# Common modules
import argparse
import logging
import os
import yaml
from typing import Dict
from pprint import pformat
from time import strftime

# Custom modules
import utils.util_nodes as util_nodes

BASE_PATH = os.path.dirname(__file__)


class ControllerBenchmark:
    def __init__(self, logger, config_path: str):
        self.logger = logger
        if os.path.isfile(config_path) is False:
            raise ValueError(f'Invalid path to maps config file: {config_path}')
        self.config_path = config_path
        self.params: Dict = {}
        self.load_config()

        self.map_data: Dict = {}
        self.load_maps()

        self.nodes: Dict = {}
        self.init_nodes()

    def load_config(self):
        with open(self.config_path, 'r') as file:
            data = yaml.safe_load(file)
            self.params['timestamp_format'] = data.get('timestamp_format')
            self.params['cmd_vel_topic'] = data.get('cmd_vel_topic')
            self.params['odom_topic'] = data.get('odom_topic')
            self.params['costmap_topic'] = data.get('costmap_topic')
            self.params['mppi_critic_topic'] = data.get('mppi_critic_topic')

        self.logger.info('Loaded config.')
        if self.logger.isEnabledFor(logging.DEBUG):
            self.logger.debug(f'Config: \n {pformat(self.params)}')

    def load_maps(self):
        with open(self.config_path, 'r') as file:
            data = yaml.safe_load(file)
            for map_ in data.get('maps'):
                self.map_data[map_] = data.get(map_)
        self.logger.info(f'Loaded {len(self.map_data)} maps.')
        if self.logger.isEnabledFor(logging.DEBUG):
            self.logger.debug(f'Maps: \n {pformat(self.map_data)}')

    def init_nodes(self):
        self.logger.info('Initializing nodes...')
        # init marker server
        self.marker_server = util_nodes.MarkerServer()
        # init gazebo interface
        self.gazebo_interface = util_nodes.GazeboInterface()
        # init odom subscriber
        self.odom_sub = util_nodes.OdomSubscriber()
        # init cmd_vel subscriber
        self.cmd_vel_sub = util_nodes.CmdVelSubscriber()
        # init costmap subscriber
        self.costmap_sub = util_nodes.CostmapSubscriber()
        # init mppi critic subscriber
        self.mppi_critic_sub = util_nodes.MPPICriticSubscriber()

    def launch_nodes(self):
        self.logger.info('Launching nodes...')

    def stop_nodes(self):
        self.logger.info('Stopping nodes...')

    def _init_test_run(self):
        self.logger.info('Initializing test runs...')

    def _run_test(self):
        self.logger.info('Running tests...')

    def _process_test_result(self):
        self.logger.info('Processing results...')


def main():
    parser = argparse.ArgumentParser(description='Run controller benchmark.')
    parser.add_argument(
        '-d', '--debug', action='store_true', default=False,
        help='Run in debug mode.')
    parser.add_argument(
        '-p', '--plot_only', action='store_true', default=False,
        help='Plot results only, no test run')
    parser.add_argument(
        '-r', '--process_results', action='store_true', default=False,
        help='Only process results, no test run.')
    args = parser.parse_args()

    # LOGGING
    logging_level = logging.DEBUG if args.debug is True else logging.INFO
    logger = logging.getLogger('ControllerBenchmark')
    logger.setLevel(logging_level)

    # create console handler
    ch = logging.StreamHandler()
    ch.setLevel(logging_level)
    # create file handler
    stamp = strftime('%Y-%m-%d-%H-%M-%S')
    log_file = os.path.join(BASE_PATH, 'logs', f'controller_benchmark_{stamp}.log')
    fh = logging.FileHandler(
        filename=log_file,
        mode='w',
        encoding='utf-8')
    fh.setLevel(logging_level)

    # create formatter
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    ch.setFormatter(formatter)
    fh.setFormatter(formatter)

    # add ch and fh to logger
    logger.addHandler(ch)
    logger.addHandler(fh)

    if args.plot_only is True:
        print('Plotting only.')
        # TODO: plot the latest results
    elif args.process_results is True:
        print('Processing results only.')
        # TODO: processing the latest results
    # if args.process_results is True and args.plot_only is True:
    #     raise ValueError('Unable to use both flags at the same time.')

    controller_benchmark = ControllerBenchmark(
        logger=logger,
        config_path=os.path.join(
            BASE_PATH, 'config/controller_benchmark_config.yaml')
    )

    controller_benchmark.launch_nodes()


if __name__ == '__main__':
    main()
