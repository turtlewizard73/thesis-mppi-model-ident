#! /usr/bin/env python3

import os
import numpy as np
import time
import yaml
import pickle
from copy import deepcopy

import constants
from controller_benchmark import ControllerBenchmark
from utils.util_functions import setup_run
from utils.controller_parameters import MPPIControllerParameters
from utils.controller_metrics import ControllerMetric

BASE_PATH = constants.BASE_PATH
LAUNCH_PATH = constants.LAUNCH_PATH

global logger


def analyze_random_search():
    global logger


def main():
    global logger
    logger = setup_run(
        logger_name='Analyzer',
        log_file_path=os.path.join(BASE_PATH, 'logs')
    )

    try:
        analyze_random_search()
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt, stopping.")

    exit(0)


if __name__ == '__main__':
    main()
