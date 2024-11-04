#! /usr/bin/env python3
import argparse
import logging
import os
from time import strftime

from controller_benchmark import ControllerBenchmark
from utils.controller_parameters import MPPIControllerParameters
from utils.controller_metrics import ControllerMetric

BASE_PATH = os.path.dirname(__file__)


def setup(logger_name: str) -> logging.Logger:
    # Parse arguments
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

    return logger

def score_random_search(metric: ControllerMetric):


def main():
    logger = setup('ControllerOptimizer')

    # Initialize default controller parameters
    default_mppi_params = MPPIControllerParameters()
    default_mppi_params.load_from_yaml(
        os.path.join(BASE_PATH, 'config', 'default_mppi_params.yaml'))

    logger.info("Default mppi parameters initialized: \n%s", default_mppi_params)

    # Initialize controller benchmark
    controller_benchmark = ControllerBenchmark(
        logger=logger,
        config_path=os.path.join(BASE_PATH, 'config/controller_benchmark_config.yaml'))

    num_trials = 100

    test_mppi_params = MPPIControllerParameters()
    for i in range(num_trials):
        # generate parameters
        test_mppi_params.randomize_weights(
            distribution='uniform', lower_bound=0.01, upper_bound=100.0)
        # run simulation
        success, results = controller_benchmark.run_benchmark(
            parameters=test_mppi_params.to_dict(),
            store_results=False)
        res = results[0]
        metric = controller_benchmark.calculate_metric(res)

        score = score(metric)


        # evaluate performance
        # update best parameters if needed
        print(f"Trial: {parameters}, Score: {score}")



if __name__ == '__main__':
    main()
