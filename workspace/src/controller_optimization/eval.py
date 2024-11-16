import os
from typing import Dict, List

import matplotlib.pyplot as plt

import constants
from utils.util_functions import setup_run
from controller_benchmark import ControllerBenchmark
from utils.controller_metrics import ControllerMetric

WORK_DIR = os.path.join(constants.REPO_PATH, 'evaluation')


def main():
    logger = setup_run('Eval')
    plt.style.context('bmh')

    cb = ControllerBenchmark(
        logger=logger.getChild('Benchmark'),
        config_path=os.path.join(constants.BASE_PATH, 'config/controller_benchmark_config.yaml'),
        save_path=WORK_DIR
    )

    metrics_dict: Dict[str, ControllerMetric] = {}
    for benchmark in os.listdir(WORK_DIR):
        benchmark_path = os.path.join(WORK_DIR, benchmark)
        if not os.path.isdir(benchmark_path):
            continue

        metrics_list: List[ControllerMetric] = []

        logger.info('Loading metrics for benchmark %s', benchmark)
        for metric in os.listdir(os.path.join(benchmark_path, 'metrics')):
            if not metric.endswith('.pickle'):
                continue

            metric_path = os.path.join(benchmark_path, 'metrics', metric)
            metrics_list.append(cb.load_metric(metric_path))

        # create a plot from each metric-s odomxy
        plot_save_path = os.path.join(benchmark_path, 'path_comparison.png')
        logger.info('Creating path comparison plot: %s', plot_save_path)
        plt.figure()
        plt.plot(
            metrics_list[0].path_xy[:, 0],
            metrics_list[0].path_xy[:, 1], label='path', color='green')
        map_name = metrics_list[0].map_name
        mapdata = cb.mapdata_dict[map_name]
        for metric in metrics_list:
            plt.plot(metric.odom_xy[:, 0], metric.odom_xy[:, 1], label=metric.uid)

        # plt.title('Path comparison')
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.grid(visible=True, which='both', axis='both')
        # plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.savefig(plot_save_path, bbox_inches='tight', dpi=500)
        plt.close()

        break


if __name__ == '__main__':
    main()
