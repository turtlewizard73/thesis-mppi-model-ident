#! /usr/bin/env python3

# Common modules
import time
from functools import wraps
from scipy.spatial.transform import Rotation

# ROS msg types
from geometry_msgs.msg import Quaternion


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


def yaw2quat(yaw: float) -> Quaternion:
    q = Rotation.from_euler('XYZ', [0., 0., yaw], degrees=False).as_quat()
    quat = Quaternion()
    quat.x = q[0]
    quat.y = q[1]
    quat.z = q[2]
    quat.w = q[3]
    return quat
