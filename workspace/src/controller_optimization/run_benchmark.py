#! /usr/bin/env python3

import os
import numpy as np
import matplotlib.pyplot as plt

import constants
from controller_benchmark import ControllerBenchmark
from utils.util_functions import setup_run
from utils.controller_results import ControllerResult
from utils.controller_parameters import ControllerParameters

BASE_PATH = constants.BASE_PATH
LAUNCH_PATH = constants.LAUNCH_PATH
global logger, controller_benchmark


def test_benchmark():
    global controller_benchmark, logger
    controller_benchmark.launch_nodes()
    res, msg = controller_benchmark.check_launch_nodes_active()
    if not res:
        logger.error(f"Failed to launch nodes: {msg}")
        return

    result: ControllerResult = controller_benchmark.run_benchmark()
    logger.info("____________________")
    logger.info(f"Benchmark result: {result.success}, msg: {result.status_msg}")
    controller_benchmark.save_result(result)
    controller_benchmark.stop_nodes()

    fig_result = controller_benchmark.plot_result(result)
    metric = controller_benchmark.calculate_metric(result)
    controller_benchmark.save_metric(metric)

    fig_metric = controller_benchmark.plot_metric(result, metric)

    plt.show()


def plot_last():
    # read last metric and result
    global controller_benchmark, logger
    result = controller_benchmark.load_last_result()
    metric = controller_benchmark.load_last_metric()

    fig_result = controller_benchmark.plot_result(result)
    fig_metric = controller_benchmark.plot_metric(result, metric)

    plt.show()


def main():
    global logger, controller_benchmark
    logger = setup_run(
        logger_name='ControllerBenchmark',
        log_file_path=os.path.join(BASE_PATH, 'logs')
    )

    controller_benchmark = ControllerBenchmark(
        logger=logger,
        config_path=os.path.join(BASE_PATH, 'config/controller_benchmark_config.yaml'))

    default_mppi_params = ControllerParameters()
    default_mppi_params.load_from_yaml(
        os.path.join(LAUNCH_PATH, 'config/nav2_params_benchmark.yaml'))

    controller_benchmark.update_parameters(default_mppi_params)
    try:
        test_benchmark()
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt, stopping nodes.")

    exit(0)


if __name__ == '__main__':
    main()
