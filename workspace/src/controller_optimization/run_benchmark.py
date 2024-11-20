#! /usr/bin/env python3

import os
import argparse
import matplotlib.pyplot as plt

import constants
from utils.util_functions import setup_run
from utils.controller_results import ControllerResult
from utils.controller_metrics import ControllerMetric
from utils.controller_parameters import ControllerParameters
from controller_benchmark import ControllerBenchmark
from controller_optimizer import ControllerOptimizer


BENCHMARK_CONFIG_PATH = constants.BENCHMARK_CONFIG_PATH
OPTIMIZER_CONFIG_PATH = constants.OPTIMIZER_CONFIG_PATH
DEFAULT_MPPI_PARAMS_PATH = constants.DEFAULT_MPPI_PARAMS_PATH

global args, logger


def run_benchmark():
    global args, logger

    # init benchmark
    benchmark = ControllerBenchmark(
        logger=logger,
        config_path=BENCHMARK_CONFIG_PATH)

    # init parameters
    default_mppi_params = ControllerParameters()
    default_mppi_params.load_from_yaml(DEFAULT_MPPI_PARAMS_PATH)
    if args.random is True:
        default_mppi_params.randomize_parameters()
    benchmark.update_parameters(default_mppi_params)

    # init map
    if args.map == '':
        logger.warning("No map selected, using default.")
        args.map = 'complex_test'
    benchmark.update_map(args.map)

    # run benchmark
    benchmark.launch_nodes()
    result: ControllerResult = benchmark.run_benchmark()
    benchmark.stop_nodes()

    result_path = benchmark.save_result(result)
    logger.info(f"Saved result to {result_path}")

    # calculate metric
    metric: ControllerMetric = benchmark.calculate_metric(result)
    metric_path = benchmark.save_metric(metric)
    logger.info(f"Saved metric to {metric_path}")

    # plot results
    fig_result = benchmark.plot_result(result)
    fig_metric = benchmark.plot_metric(metric)
    plt.show(block=False)

    while True:
        key = input("Press 'q' to quit: ").strip().lower()
        if key == 'q':
            break
    plt.close('all')

    del benchmark


def plot_last():
    global args, logger

    # init benchmark
    benchmark = ControllerBenchmark(
        logger=logger,
        config_path=BENCHMARK_CONFIG_PATH)

    # read last result and metric
    result = benchmark.load_last_result()
    metric = benchmark.load_last_metric()

    # plot results
    fig_result = benchmark.plot_result(result)
    fig_metric = benchmark.plot_metric(metric)
    plt.show(block=False)

    while True:
        key = input("Press 'q' to quit: ").strip().lower()
        if key == 'q':
            break
    plt.close('all')

    del benchmark


def run_optimizer():
    global args, logger

    # init optimizer
    optimizer = ControllerOptimizer(
        logger=logger,
        config_path=OPTIMIZER_CONFIG_PATH)

    # setup optimizer
    if args.trial == '':
        logger.warning("No trial selected, using default.")
        args.trial = 'test_trial'
    optimizer.setup_run(trial_name=args.trial)

    # run optimizer
    optimizer.run_reference()
    optimizer.run()

    del optimizer


def main():
    global args, logger

    parser = argparse.ArgumentParser(description='Run controller benchmark.')
    logger = setup_run(parser, constants.TIMESTAMP_FORMAT, constants.LOG_PATH)
    args = parser.parse_args()

    try:
        if args.benchmark is True:
            run_benchmark()
        elif args.plot_last is True:
            plot_last()
        elif args.optimizer is True:
            run_optimizer()
        else:
            raise ValueError("Invalid run mode.")
    except Exception as e:
        logger.error(e)
        return 1
    else:
        return 0


if __name__ == '__main__':
    exit(main())
