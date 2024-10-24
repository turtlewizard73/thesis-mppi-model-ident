import time
import numpy as np
import tracemalloc
from collections import deque
from typing import Tuple

# Mock the original and optimized classes for comparison

class OdomSubscriberOriginal:
    def __init__(self) -> None:
        self.odom_xy = deque()
        self.odom_t_ns = deque()

    def add_data(self, x, y, t_ns):
        self.odom_xy.append([x, y])
        self.odom_t_ns.append(t_ns)

    def get_data_between(self, start_time_ns, end_time_ns) -> Tuple[np.ndarray, np.ndarray]:
        xy_values = []
        t_values = []
        for _ in range(len(self.odom_t_ns)):
            xy = self.odom_xy.popleft()
            t = self.odom_t_ns.popleft()
            if start_time_ns <= t <= end_time_ns:
                xy_values.append(xy)
                t_values.append(t - start_time_ns)
        return np.array(xy_values), np.array(t_values)

class OdomSubscriberOptimized:
    def __init__(self) -> None:
        self.odom_xy = deque()
        self.odom_t_ns = deque()

    def add_data(self, x, y, t_ns):
        self.odom_xy.append([x, y])
        self.odom_t_ns.append(t_ns)

    def get_data_between(self, start_time_ns, end_time_ns) -> Tuple[np.ndarray, np.ndarray]:
        odom_xy = np.array(self.odom_xy)
        odom_t_ns = np.array(self.odom_t_ns)
        mask = (odom_t_ns >= start_time_ns) & (odom_t_ns <= end_time_ns)
        xy_values = odom_xy[mask]
        t_values = odom_t_ns[mask] - start_time_ns
        return xy_values, t_values

def benchmark_method(subscriber_class, num_samples=10000):
    # Initialize the class
    subscriber = subscriber_class()

    # Generate synthetic data
    start_time_ns = 1_000_000_000
    end_time_ns = 2_000_000_000
    timestamps = np.linspace(start_time_ns, end_time_ns, num=num_samples)

    # Populate the subscriber with mock data
    for t in timestamps:
        subscriber.add_data(x=np.random.random(), y=np.random.random(), t_ns=int(t))

    # Measure time and memory usage for get_data_between
    tracemalloc.start()
    start_time = time.time()
    _ = subscriber.get_data_between(start_time_ns, end_time_ns)
    end_time = time.time()
    current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    return end_time - start_time, current, peak

if __name__ == "__main__":
    num_samples = 10000

    # Benchmark original method
    original_time, original_current_mem, original_peak_mem = benchmark_method(OdomSubscriberOriginal, num_samples)
    print(f"Original method - Time: {original_time:.4f} seconds, Current memory: {original_current_mem / 1024:.2f} KB, Peak memory: {original_peak_mem / 1024:.2f} KB")

    # Benchmark optimized method
    optimized_time, optimized_current_mem, optimized_peak_mem = benchmark_method(OdomSubscriberOptimized, num_samples)
    print(f"Optimized method - Time: {optimized_time:.4f} seconds, Current memory: {optimized_current_mem / 1024:.2f} KB, Peak memory: {optimized_peak_mem / 1024:.2f} KB")

    # Calculate improvements
    time_improvement = (original_time - optimized_time) / original_time * 100
    memory_improvement = (original_peak_mem - optimized_peak_mem) / original_peak_mem * 100

    print(f"Time improvement: {time_improvement:.2f}%")
    print(f"Memory improvement: {memory_improvement:.2f}%")
