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

    if args.plot_only is False:
        controller_benchmark.launch_nodes()
        controller_benchmark.start_data_collection()
        controller_benchmark.run_benchmark()
        controller_benchmark.stop_data_collection()
        res = controller_benchmark.results[0]
        controller_benchmark.save_result(res)
        controller_benchmark.stop_nodes()

        fig_result = controller_benchmark.plot_result(res)
        # metric = controller_benchmark.calculate_metric(res)
        # fig_result = controller_benchmark.plot_result(res)
        plt.show()
    else:
        res = controller_benchmark.load_last_result()
        fig_result = controller_benchmark.plot_result(res)

        metric = controller_benchmark.calculate_metric(res)
        fig_metrics = controller_benchmark.plot_metric(res, metric)
        plt.show()

    exit(0)


if __name__ == '__main__':
    main()
