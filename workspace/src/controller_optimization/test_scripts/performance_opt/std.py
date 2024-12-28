import timeit
import numpy as np

# Sample float list for benchmarking
float_list = [1.1, 2.3, 3.3, 4.5, 5.8, 6.7, 7.1, 8.2, 9.0, 10.4]

# Custom method to calculate standard deviation


def custom_std(float_list):
    n = len(float_list)
    mean = sum(float_list) / n
    return (sum([(x - mean) ** 2 for x in float_list]) / n) ** 0.5

# NumPy method to calculate standard deviation with recasting back to a Python list


def numpy_std(float_list):
    return np.std(np.array(float_list, dtype=np.float64))  # Convert back to Python float


# Benchmarking both methods
custom_time = timeit.timeit(lambda: custom_std(float_list), number=100000)
numpy_time = timeit.timeit(lambda: numpy_std(float_list), number=100000)

print(f'Custom method: {custom_time} seconds')
print(f'NumPy method: {numpy_time} seconds')
