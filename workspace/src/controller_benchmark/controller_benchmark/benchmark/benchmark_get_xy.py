import timeit
import numpy as np

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

# The get_xy function to be benchmarked
def get_xy(p) -> np.ndarray:
    if isinstance(p, PoseStamped):
        return np.array([p.pose.position.x, p.pose.position.y])
    elif isinstance(p, PoseWithCovarianceStamped):
        return np.array([p.pose.pose.position.x, p.pose.pose.position.y])
    elif isinstance(p, Odometry):
        return np.array([p.pose.pose.position.x, p.pose.pose.position.y])

def get_xy_v1(p) -> np.ndarray:
    if hasattr(p, 'pose'):
        pose = p.pose.pose if hasattr(p.pose, 'pose') else p.pose
        return np.array([pose.position.x, pose.position.y])
    return np.zeros(2)  # Fallback in case p doesn't match expected types

# Create test data
pose = Pose(Position(1.0, 2.0))
pose_with_covariance = PoseWithCovariance(pose)
pose_stamped = PoseStamped(pose)
pose_with_covariance_stamped = PoseWithCovarianceStamped(pose_with_covariance)
odometry = Odometry(pose_with_covariance)

# Benchmarking script
def benchmark_get_xy0(version=0):
    if version == 0:
        get_xy(pose_stamped)
    elif version == 1:
        get_xy_v1(pose_stamped)

def benchmark_get_xy1(version=0):
    if version == 0:
        get_xy(pose_with_covariance_stamped)
    elif version == 1:
        get_xy_v1(pose_with_covariance_stamped)

def benchmark_get_xy2(version=0):
    if version == 0:
        get_xy(odometry)
    elif version == 1:
        get_xy_v1(odometry)

# Run the benchmark
if __name__ == "__main__":
    number_of_runs = 100000
    time_taken = timeit.timeit(lambda: benchmark_get_xy0(0), number=number_of_runs)
    print(f"Time taken for {number_of_runs} runs: {time_taken:.6f} seconds (pose_stamped)")
    time_taken = timeit.timeit(lambda: benchmark_get_xy1(0), number=number_of_runs)
    print(f"Time taken for {number_of_runs} runs: {time_taken:.6f} seconds (pose_with_covariance_stamped)")
    time_taken = timeit.timeit(lambda: benchmark_get_xy2(0), number=number_of_runs)
    print(f"Time taken for {number_of_runs} runs: {time_taken:.6f} seconds (odometry)")

    time_taken = timeit.timeit(lambda: benchmark_get_xy0(1), number=number_of_runs)
    print(f"Time taken for {number_of_runs} runs: {time_taken:.6f} seconds (pose_stamped)")
    time_taken = timeit.timeit(lambda: benchmark_get_xy1(1), number=number_of_runs)
    print(f"Time taken for {number_of_runs} runs: {time_taken:.6f} seconds (pose_with_covariance_stamped)")
    time_taken = timeit.timeit(lambda: benchmark_get_xy2(1), number=number_of_runs)
    print(f"Time taken for {number_of_runs} runs: {time_taken:.6f} seconds (odometry)")