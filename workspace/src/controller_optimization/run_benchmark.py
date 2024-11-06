#! /usr/bin/env python3

# Common modules
import argparse
import logging
import os
from time import strftime
import matplotlib.pyplot as plt
from controller_benchmark import ControllerBenchmark
from utils.controller_results import ControllerResult
from utils.controller_parameters import MPPIControllerParameters

BASE_PATH = os.path.dirname(__file__)
LAUNCH_PATH = '/home/turtlewizard/thesis-mppi-model-ident/workspace/src/controller_launch'
global logger, controller_benchmark


def setup(logger_name: str = 'ControllerBenchmark') -> logging.Logger:
    global logger

    parser = argparse.ArgumentParser(description='Run controller benchmark.')
    parser.add_argument(
        '-d', '--debug', action='store_true', default=False,
        help='Run in debug mode.')
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
    formatter = logging.Formatter('[%(levelname)s] [%(asctime)s] [%(name)s] %(message)s')
    ch.setFormatter(formatter)
    fh.setFormatter(formatter)

    # add ch and fh to logger
    logger.addHandler(ch)
    logger.addHandler(fh)


def test_benchmark():
    global controller_benchmark, logger
    controller_benchmark.launch_nodes()
    result: ControllerResult = controller_benchmark.run_benchmark()
    logger.info("____________________")
    logger.info(f"Benchmark result: {result.success}, {result.status_msg}")
    controller_benchmark.save_result(result)
    controller_benchmark.stop_nodes()

    fig_result = controller_benchmark.plot_result(result)
    metric = controller_benchmark.calculate_metric(result)
    controller_benchmark.save_metric(metric)

    fig_metric = controller_benchmark.plot_metric(result, metric)

    plt.show()


def plot(metric, result):
    # read last metric and result
    pass


def main():
    global logger, controller_benchmark
    setup()

    default_mppi_params = MPPIControllerParameters()
    default_mppi_params.load_from_yaml(
        os.path.join(LAUNCH_PATH, 'config/nav2_params_benchmark.yaml'))

    logger.info("Default mppi parameters initialized: \n%s", default_mppi_params)

    controller_benchmark = ControllerBenchmark(
        logger=logger,
        config_path=os.path.join(BASE_PATH, 'config/controller_benchmark_config.yaml'),
        mppi_params=default_mppi_params)

    test_benchmark()
    exit(0)


if __name__ == '__main__':
    main()
