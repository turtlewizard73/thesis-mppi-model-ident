#! /usr/bin/env python3
import os
import time

import constants
from controller_optimizer import ControllerOptimizer
from utils.util_functions import setup_run

BASE_PATH = constants.BASE_PATH
LAUNCH_PATH = constants.LAUNCH_PATH
OPTIMIZATION_OUTPUT_PATH = constants.OPTIMIZATION_OUTPUT_PATH


def main():
    NAME = 'test0'

    logger = setup_run('optimizer', os.path.join(BASE_PATH, 'logs'))
    stamp = time.strftime(constants.TIMESTAMP_FORMAT)
    WORK_DIR = os.path.join(OPTIMIZATION_OUTPUT_PATH, f'{NAME}_{stamp}')

    optimizer = ControllerOptimizer(
        logger=logger,
        config_path=os.path.join(BASE_PATH, 'config/controller_optimizer_config.yaml'),
        work_dir=WORK_DIR)

    optimizer.setup_run(trial_name='test_trial')

    optimizer.run_reference()
    optimizer.run()

    time.sleep(5)


if __name__ == '__main__':
    main()
