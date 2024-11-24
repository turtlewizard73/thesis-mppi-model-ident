from time import time
import numpy as np
from discrete_frechet import FastDiscreteFrechetMatrix, euclidean
import similaritymeasures
from frechet_display import compute_frechet_distance

# Function to measure execution time over multiple runs


def benchmark_function(func, *args, num_runs=10):
    times = []
    for _ in range(num_runs):
        start = time()
        func(*args)  # Call the function with arguments
        end = time()
        times.append(end - start)
    return {
        "min_time": np.min(times),
        "max_time": np.max(times),
        "avg_time": np.mean(times),
        "std_dev": np.std(times),
    }


# Generate random experimental data
x = np.linspace(0, 2 * np.pi, 100)
y = np.sin(x)
exp_data = np.zeros((100, 2))
exp_data[:, 0] = x
exp_data[:, 1] = y

# Generate random numerical data
x = np.linspace(0, 2 * np.pi, 100)
y = np.sin(x) + np.random.rand(100) * 0.1
num_data = np.zeros((100, 2))
num_data[:, 0] = x
num_data[:, 1] = y

# Number of runs for benchmarking
num_runs = 100

# Benchmark the three functions
similaritymeasures_results = benchmark_function(
    similaritymeasures.frechet_dist, exp_data, num_data, num_runs=num_runs)
print("similaritymeasures.frechet_dist:")
print(similaritymeasures_results)

compute_frechet_results = benchmark_function(
    compute_frechet_distance, exp_data, num_data, num_runs=num_runs)
print("\ncompute_frechet_distance:")
print(compute_frechet_results)

frechet_calc = FastDiscreteFrechetMatrix(euclidean)
fast_discrete_results = benchmark_function(
    frechet_calc.distance, exp_data, num_data, num_runs=num_runs)
print("\nFastDiscreteFrechetMatrix:")
print(fast_discrete_results)
