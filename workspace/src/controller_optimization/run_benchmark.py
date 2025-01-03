#! /usr/bin/env python3

import os
import yaml
import pandas as pd
import pickle
import traceback
import argparse
import matplotlib.pyplot as plt
import faulthandler

import constants
from utils.util_functions import setup_run
from utils.controller_metrics import ControllerMetric
from utils.controller_parameters import ControllerParameters
from controller_benchmark import ControllerBenchmark
from controller_optimizer import ControllerOptimizer


BENCHMARK_CONFIG_PATH = constants.BENCHMARK_CONFIG_PATH
OPTIMIZER_CONFIG_PATH = constants.OPTIMIZER_CONFIG_PATH
DEFAULT_MPPI_PARAMS_PATH = constants.DEFAULT_MPPI_PARAMS_PATH

global args, logger
faulthandler.enable()


def run_benchmark():
    global args, logger

    try:
        # init benchmark
        benchmark = ControllerBenchmark(
            logger=logger,
            config_path=BENCHMARK_CONFIG_PATH)

        # init parameters
        default_mppi_params = ControllerParameters()
        default_mppi_params.load_from_yaml(DEFAULT_MPPI_PARAMS_PATH)
        if args.random is True:
            default_mppi_params.randomize_weights()
        benchmark.update_parameters(default_mppi_params)

        # init map
        if args.map == 'default':
            logger.warning("No map selected, using default.")
            benchmark.update_map()
        else:
            benchmark.update_map(args.map)

        # run benchmark
        benchmark.launch_nodes()
        metric: ControllerMetric = benchmark.run_benchmark()
        benchmark.stop_nodes()

        metric_path = benchmark.save_metric(metric)
        logger.info(f"Saved metric to {metric_path}")

        # print data
        logger.info('Printing Mapdata.')
        logger.info('\n' + benchmark.print_mapdata())
        logger.info('Printing Parameters.')
        logger.info('\n' + benchmark.print_parameters())
        logger.info('Printing Metric.')
        logger.info('\n' + benchmark.print_metric_data(metric))

        # plot results
        fig_metric = benchmark.plot_metric(metric)
        plt.show(block=False)

        while True:
            key = input("Press 'q' to quit: ").strip().lower()
            if key == 'q':
                break
    except Exception as e:
        logger.error(e)
        traceback.print_exc()
    finally:
        plt.close('all')
        del benchmark


def plot_last():
    global args, logger

    # init benchmark
    benchmark = ControllerBenchmark(
        logger=logger,
        config_path=BENCHMARK_CONFIG_PATH)

    # read last result and metric
    metric = benchmark.load_last_metric()

    # plot results
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

    try:
        # init optimizer
        optimizer = ControllerOptimizer(
            logger=logger,
            config_path=OPTIMIZER_CONFIG_PATH)

        # setup optimizer
        if args.trial == 'default':
            logger.warning("No trial selected, using default.")
            args.trial = 'test_trial'
        optimizer.setup_run(trial_name=args.trial)

        # run optimizer
        optimizer.run_reference()
        optimizer.run()
    except Exception as e:
        logger.error(e)
        traceback.print_exc()

    del optimizer


def run_evaluation():
    global args, logger
    WORK_DIR = args.evaluation
    WORK_DIR = os.path.abspath(WORK_DIR)
    logger.info(f"Running evaluation on {WORK_DIR}.")


def run_test():
    global args, logger
    logger.info("___ TEST ___")
    try:
        benchmark = ControllerBenchmark(logger, BENCHMARK_CONFIG_PATH)
        benchmark.update_map()
        # metric: ControllerMetric = benchmark.run_benchmark()

        # init parameters
        default_mppi_params = ControllerParameters()
        default_mppi_params.load_from_yaml(DEFAULT_MPPI_PARAMS_PATH)
        default_mppi_params.randomize_weights()

        benchmark.update_parameters(default_mppi_params)

    except Exception as e:
        logger.error(e)
        traceback.print_exc()
    finally:
        del benchmark


def main():
    global args, logger

    parser = argparse.ArgumentParser(description='Run controller benchmark.')
    logger = setup_run(parser, constants.TIMESTAMP_FORMAT, constants.LOG_PATH)
    args = parser.parse_args()

    try:
        if args.benchmark is True:
            run_benchmark()
            return 0
        elif args.plot_last is True:
            plot_last()
            return 0
        elif args.optimizer is True:
            run_optimizer()
            return 0
        elif args.evaluation != '':
            run_evaluation()
            return 0
        else:
            run_test()
    except Exception as e:
        logger.error(e)
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    exit(main())
