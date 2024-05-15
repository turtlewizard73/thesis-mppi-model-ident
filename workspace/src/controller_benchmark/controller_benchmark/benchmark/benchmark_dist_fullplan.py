import timeit
import numpy as np
from typing import List

# Mock classes to simulate PoseStamped, PoseWithCovarianceStamped, and Odometry
class Position:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Pose:
    def __init__(self, position):
        self.position = position

class PoseStamped:
    def __init__(self, pose):
        self.pose = pose

class PoseWithCovariance:
    def __init__(self, pose):
        self.pose = pose

class PoseWithCovarianceStamped:
    def __init__(self, pose_with_covariance):
        self.pose = pose_with_covariance

class Odometry:
    def __init__(self, pose_with_covariance):
        self.pose = pose_with_covariance

# Original get_xy function
def get_xy_original(p) -> np.ndarray:
    if isinstance(p, PoseStamped):
        return np.array([p.pose.position.x, p.pose.position.y])
    elif isinstance(p, PoseWithCovarianceStamped):
        return np.array([p.pose.pose.position.x, p.pose.pose.position.y])
    elif isinstance(p, Odometry):
        return np.array([p.pose.pose.position.x, p.pose.pose.position.y])

# Original dist_fullplan function
def dist_fullplan_original(plan: List) -> float:
    length = 0.0
    p_last = plan[0]
    for i in range(1, len(plan)):
        p = plan[i]
        length += np.linalg.norm(get_xy_original(p_last) - get_xy_original(p))
        p_last = p
    return length

# Optimized get_xy function
def get_xy(p) -> np.ndarray:
    if hasattr(p, 'pose'):
        pose = p.pose.pose if hasattr(p.pose, 'pose') else p.pose
        return np.array([pose.position.x, pose.position.y])
    return np.zeros(2)

# Optimized dist_fullplan function
def dist_fullplan(plan: list) -> float:
    if len(plan) < 2:
        return 0.0

    xy_positions = np.array([get_xy_original(p) for p in plan])
    deltas = np.diff(xy_positions, axis=0)
    distances = np.linalg.norm(deltas, axis=1)
    return np.sum(distances)

# Create test data
pose = Pose(Position(1.0, 2.0))
pose_with_covariance = PoseWithCovariance(pose)
pose_stamped = PoseStamped(pose)
pose_with_covariance_stamped = PoseWithCovarianceStamped(pose_with_covariance)
odometry = Odometry(pose_with_covariance)

# Create a sample plan with a mix of objects
plan = [pose_stamped, pose_with_covariance_stamped, odometry, pose_stamped, pose_with_covariance_stamped]

# Benchmarking functions
def benchmark_dist_fullplan_original():
    dist_fullplan_original(plan)

def benchmark_dist_fullplan():
    dist_fullplan(plan)

# Run the benchmarks
if __name__ == "__main__":
    number_of_runs = 1000

    time_original = timeit.timeit(benchmark_dist_fullplan_original, number=number_of_runs)
    print(f"Original dist_fullplan: Time taken for {number_of_runs} runs: {time_original:.6f} seconds")

    time_optimized = timeit.timeit(benchmark_dist_fullplan, number=number_of_runs)
    print(f"Optimized dist_fullplan: Time taken for {number_of_runs} runs: {time_optimized:.6f} seconds")
