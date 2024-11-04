#! /usr/bin/env python3

# Common modules
import argparse
import logging
import os
from time import strftime
import matplotlib.pyplot as plt
from controller_benchmark import ControllerBenchmark

BASE_PATH = os.path.dirname(__file__)


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
    parser.add_argument(
        '-b', '--batch', action='store_true', default=False,
        help='Run in batch mode.')
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

    if args.batch is True:
        from itertools import product
        import numpy as np

        controller_benchmark.launch_nodes()
        # Generate a list of weights from 1-100
        n = 10
        weights = np.linspace(1.0, 100.0, n)
        critics = ['ConstraintCritic', 'GoalCritic', 'PreferForwardCritic',
                   'CostCritic', 'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic']

        # weights = [1.0, 10.0, 200.0]
        critics = ['CostCritic', 'PathFollowCritic']

        parameter_space = list(product(weights, repeat=len(critics)))
        print(f'Possible parameter combinations: {len(parameter_space)}')

        i = 0
        results: list[dict[int, bool]] = []
        critic_weights = []
        for weights in parameter_space:
            params = {critic: weight for critic, weight in zip(critics, weights)}
            critic_weights.append(params)

            success, res = controller_benchmark.run_benchmark(
                parameters=params, store_results=False)
            msg = 'successful' if success else 'unsuccessful'
            print(f'Iteration {i} was {msg}. with weights {weights}')

            controller_benchmark.save_result(res[0], sub_folder='paramspace0')

            results.append({i: success})
            i += 1
        exit(0)

    if args.plot_only is False:
        controller_benchmark.launch_nodes()

        success, results = controller_benchmark.run_benchmark(store_results=False)
        res = results[0]
        controller_benchmark.save_result(res)
        controller_benchmark.stop_nodes()

        fig_result = controller_benchmark.plot_result(res)
        metric = controller_benchmark.calculate_metric(res)
        fig_metric = controller_benchmark.plot_metric(res, metric)
        plt.show()
    else:
        res = controller_benchmark.load_last_result()

        controller_benchmark.save_result(res, sub_folder='test')

        fig_result = controller_benchmark.plot_result(res)

        metric = controller_benchmark.calculate_metric(res)
        fig_metrics = controller_benchmark.plot_metric(res, metric)
        plt.show()

    exit(0)


if __name__ == '__main__':
    main()
