import os
from typing import Dict

import matplotlib.pyplot as plt

import constants
from utils.util_functions import setup_run
from controller_benchmark import ControllerBenchmark
from utils.controller_metrics import ControllerMetric

WORK_DIR = os.path.join(constants.REPO_PATH, 'evaluation')


def main():
    logger = setup_run('Eval')

    cb = ControllerBenchmark(
        logger=logger.getChild('Benchmark'),
        config_path=os.path.join(constants.REPO_PATH, 'config/controller_benchmark_config.yaml'),
        save_path=WORK_DIR
    )

    metrics: Dict[str, ControllerMetric] = {}
    for benchmark in os.listdir(WORK_DIR):
        benchmark_path = os.path.join(WORK_DIR, benchmark)
        if not os.path.isdir(benchmark_path):
            continue

        metrics[benchmark] = []

        logger.info(f'Loading metrics for benchmark {benchmark}')
        for metric in os.listdir(os.path.join(benchmark_path, 'metrics')):
            if not metric.endswith('.pickle'):
                continue

            metric_path = os.path.join(benchmark_path, 'metrics', metric)
            metrics[benchmark].append(cb.load_metric(metric_path))

        # create a plot from each metric-s odomxy
        path_plot = plt.figure()
        plt.plot(
            metric[benchmark][0].path_xy[:, 0],
            metric[benchmark][0].path_xy[:, 1], label='path', color='green')
        mapdata = cb.mapdata_dict[metric[benchmark][0].map_name]
        plt.imshow()
        for metric in metrics[benchmark]:
            plt.plot(metric.odomxy[:, 0], metric.odomxy[:, 1], label=metric.run_uid)


if __name__ == '__main__':
    main()
