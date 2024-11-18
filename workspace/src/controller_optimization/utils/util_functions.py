#! /usr/bin/env python3

# Common modules
import csv
import os
import argparse
import logging
from time import strftime
import time
# import yaml
import yaml
import numpy as np
from functools import wraps
from scipy.spatial.transform import Rotation

# ROS msg types
from geometry_msgs.msg import Quaternion


def setup_run(
        parser: argparse.ArgumentParser = None,
        logger_name: str = 'ControllerBenchmark',
        log_file_path: str = '/home/turtlewizard/thesis-mppi-model-ident/workspace/src/controller_optimization/logs') -> logging.Logger:
    parser = parser if parser is not None else argparse.ArgumentParser('Setup run script.')
    parser.add_argument(
        '-d', '--debug', action='store_true', default=False,
        help='Run in debug mode.')
    parser.add_argument(
        '-t', '--trial', type=str, default='test_trial',)
    args = parser.parse_args()

    # LOGGING
    if not os.path.exists(log_file_path):
        os.makedirs(log_file_path)
    # log_file_path = os.path.join(log_file_path, logger_name.lower())
    # if not os.path.exists(log_file_path):
    #     os.makedirs(log_file_path)

    logging_level = logging.DEBUG if args.debug is True else logging.INFO
    logger = logging.getLogger(logger_name)
    logger.setLevel(logging_level)

    # create console handler
    ch = logging.StreamHandler()
    ch.setLevel(logging_level)
    # create file handler
    stamp = strftime('%Y-%m-%d-%H-%M-%S')
    log_file = os.path.join(log_file_path, f'{logger_name.lower()}_{stamp}.log')
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


def timing_decorator(on_start, on_end):
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            context = args[0]

            # Call the start function with `context`
            if callable(on_start):
                on_start(context)

            start_time = time.time()
            result = func(*args, **kwargs)
            end_time = time.time()
            execution_time = end_time - start_time

            # Call the end function with `context` and the execution time
            if callable(on_end):
                on_end(context, execution_time)

            return result
        return wrapper
    return decorator


def flatten_dict(d: dict, parent_key: str = '', sep: str = '.'):
    flat_data = {}
    for key, value in d.items():
        if 'qos' in key:
            continue
        if 'qos' in key:
            continue
        new_key = f'{parent_key}{sep}{key}' if parent_key else key
        if isinstance(value, dict):
            flat_data.update(flatten_dict(d=value, parent_key=new_key, sep=sep))
        else:
            flat_data[new_key] = value
    return flat_data


def reformat_yaml(input_path: str, output_path: str):
    with open(input_path, 'r', encoding='utf-8') as file:
        data = yaml.safe_load(file)

    flat_data = {}
    for node_name, node_params in data.items():
        ros_parameters = node_params['ros__parameters']
        node_name = node_name if node_name.startswith("/") else f"/{node_name}"
        flat_data.update({
            node_name: {
                'ros__parameters': flatten_dict(ros_parameters)
            }
        })

    with open(output_path, 'w', encoding='utf-8') as file:
        file.write(yaml.dump(flat_data, default_flow_style=None))


def numpy_dict_to_list(d: dict) -> dict:
    for key, value in d.items():
        if isinstance(value, dict):
            d[key] = numpy_dict_to_list(value)
        elif isinstance(value, np.ndarray):
            d[key] = value.tolist()
        elif isinstance(value, np.float64):
            d[key] = float(value)
    return d


def yaw2quat(yaw: float) -> Quaternion:
    q = Rotation.from_euler('XYZ', [0., 0., yaw], degrees=False).as_quat()
    quat = Quaternion()
    quat.x = q[0]
    quat.y = q[1]
    quat.z = q[2]
    quat.w = q[3]
    return quat


def plan_length(plan: np.ndarray) -> float:
    return np.sum(np.linalg.norm(plan[1:] - plan[:-1], axis=1))


def newton_diff(y: np.ndarray, dt: float) -> np.ndarray:
    derivative = np.zeros_like(y)
    # Central differences
    derivative[2:-2] = (-y[4:] + 8 * y[3:-1] - 8 * y[1:-3] + y[0:-4]) / (12 * dt)
    # Use lower-order methods at the boundaries
    derivative[0] = (y[1] - y[0]) / dt  # Forward difference for the first point
    derivative[1] = (y[2] - y[0]) / (2 * dt)  # Central difference for the second point
    derivative[-2] = (y[-1] - y[-3]) / (2 * dt)  # Central difference for the second last point
    derivative[-1] = (y[-1] - y[-2]) / dt  # Backward difference for the last point
    return derivative


def newton_diff_nonuniform(y: np.ndarray, t: np.ndarray) -> np.ndarray:
    derivative = np.zeros_like(y)
    dt_forward = t[1:] - t[:-1]  # Time differences between each consecutive time step

    # Use central differences for interior points
    derivative[1:-1] = (y[2:] - y[:-2]) / (t[2:] - t[:-2])

    # Handle boundaries with forward and backward differences
    derivative[0] = (y[1] - y[0]) / dt_forward[0]  # Forward difference for the first point
    derivative[-1] = (y[-1] - y[-2]) / dt_forward[-1]  # Backward difference for the last point

    return derivative
