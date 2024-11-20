#! /usr/bin/env python3
import os
import argparse

import constants
from controller_optimizer import ControllerOptimizer
from utils.util_functions import setup_run

BASE_PATH = constants.BASE_PATH
LAUNCH_PATH = constants.LAUNCH_PATH
OPTIMIZATION_OUTPUT_PATH = constants.OPTIMIZATION_OUTPUT_PATH


def main():
    parser = argparse.ArgumentParser(description='Run controller optimizer.')
    logger = setup_run(parser, 'Optimizer', os.path.join(BASE_PATH, 'logs'))
    trial_name = parser.parse_args().trial

    optimizer = ControllerOptimizer(
        logger=logger,
        config_path=os.path.join(BASE_PATH, 'config/controller_optimizer_config.yaml'))

    optimizer.setup_run(trial_name=trial_name)

    optimizer.run_reference()
    optimizer.run()

    del optimizer
    return 0


if __name__ == '__main__':
    exit(main())
