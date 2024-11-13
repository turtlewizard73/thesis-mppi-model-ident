import os
import time
import matplotlib.pyplot as plt

import rclpy

import constants
from utils.util_functions import setup_run
from controller_benchmark import ControllerBenchmark
from utils.controller_results import ControllerResult


def main():
    logger = setup_run('Measurement')
    benchmark = ControllerBenchmark(
        logger=logger.getChild('Benchmark'),
        config_path=os.path.join(
            constants.CONFIG_PATH, '/controller_benchmark_config.yaml'))

    result = ControllerResult(
            controller_name='mppi_controller',
            map_name='real_world',
            uid='service_robot')

    # TODO: change topics
    benchmark._start_data_collection()  # pylint: disable=W0212
    logger.info("___ Data collection started ___")

    start_time: rclpy.clock.Time = rclpy.clock.Clock().now()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        benchmark._stop_data_collection()  # pylint: disable=W0212
        benchmark.stop_nodes()
        logger.info("___ Data collection stopped ___")
    end_time: rclpy.clock.Time = rclpy.clock.Clock().now()

    time_elapsed: rclpy.clock.Duration = end_time - start_time
    time_elapsed = time_elapsed.nanoseconds * 1e-9
    logger.info('___ Measurement finished in %s [s]. ___', time_elapsed)

    result.time_elapsed = time_elapsed
    result.success = True
    result.status_msg = 'Success'
    # result.path_xy = path_xy  TODO: globalplan subscriber
    # result.path_omega = path_omega
    benchmark.get_result(result, start_time, end_time)
    res_path = benchmark.save_result(result)
    benchmark.save_to_yaml(result, res_path.replace('.pickle', '.yaml')) # TODO: test
    logger.info("Saved result to %s", res_path)


    metric = benchmark.calculate_metric(result)
    met_path = benchmark.save_metric(metric)
    benchmark.save_to_yaml(metric, met_path.replace('.pickle', '.yaml'))
    logger.info("Saved metric to %s", met_path)

    fig_result = benchmark.plot_result(result)
    fig_metric = benchmark.plot_metric(result, metric)

    plt.show()


if __name__ == '__main__':
    main()
