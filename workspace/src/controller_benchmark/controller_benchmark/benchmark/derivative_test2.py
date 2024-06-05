import numpy as np
import matplotlib.pyplot as plt

# benchmark
import os
import psutil
import memory_profiler
from time import time as python_time


def measure_resource_usage(func):
    def wrapper(*args, **kwargs):
        process = psutil.Process(os.getpid())

        # Measure initial memory usage
        initial_memory = memory_profiler.memory_usage()[0]

        # Measure initial CPU time
        initial_cpu_times = process.cpu_times()
        initial_cpu_user_time = initial_cpu_times.user

        start_time = python_time()
        result = func(*args, **kwargs)
        end_time = python_time()

        # Measure final memory usage
        final_memory = memory_profiler.memory_usage()[0]

        # Measure final CPU time
        final_cpu_times = process.cpu_times()
        final_cpu_user_time = final_cpu_times.user

        # Calculate memory and CPU usage
        memory_usage = final_memory - initial_memory
        cpu_usage = final_cpu_user_time - initial_cpu_user_time
        execution_time = end_time - start_time

        print(f"{func.__name__} Memory usage: {memory_usage} MiB")
        print(f"{func.__name__} CPU usage: {cpu_usage} seconds")
        print(f"{func.__name__} Execution time: {execution_time} seconds")

        return result

    return wrapper

# Parameters
dt = 0.25  # Time step
x_values = np.arange(0, np.pi*2, dt)  # x values from 0 to π with step dt

# Pre-generated array of sine values
sine_values = np.sin(x_values)

# Define the exact derivative of sine function (cosine function)
def exact_derivative(x):
    return np.cos(x)

# Function to calculate numerical derivative using central difference
def central_difference_derivative(y, dt):
    derivative = np.zeros_like(y)
    derivative[1:-1] = (y[2:] - y[:-2]) / (2 * dt)
    derivative[0] = (y[1] - y[0]) / dt  # Forward difference for the first point
    derivative[-1] = (y[-1] - y[-2]) / dt  # Backward difference for the last point
    return derivative

# Function to calculate numerical derivative using forward difference
def forward_difference_derivative(y, dt):
    derivative = np.zeros_like(y)
    derivative[:-1] = (y[1:] - y[:-1]) / dt
    derivative[-1] = (y[-1] - y[-2]) / dt  # Backward difference for the last point
    return derivative

# Function to calculate numerical derivative using backward difference
def backward_difference_derivative(y, dt):
    derivative = np.zeros_like(y)
    derivative[1:] = (y[1:] - y[:-1]) / dt
    derivative[0] = (y[1] - y[0]) / dt  # Forward difference for the first point
    return derivative

# numpy diff
def np_diff(y, dt):
    return np.diff(y) / dt

# numpy gradient
def np_gradient(y, dt):
    return np.gradient(y, dt)

# newton
@measure_resource_usage
def higher_order_derivative_original(y, dt):
    derivative = np.zeros_like(y)
    n = len(y)
    for i in range(2, n-2):
        derivative[i] = (-y[i+2] + 8*y[i+1] - 8*y[i-1] + y[i-2]) / (12 * dt)
    # Use lower-order methods at the boundaries
    derivative[0] = (y[1] - y[0]) / dt  # Forward difference for the first point
    derivative[1] = (y[2] - y[0]) / (2 * dt)  # Central difference for the second point
    derivative[-2] = (y[-1] - y[-3]) / (2 * dt)  # Central difference for the second last point
    derivative[-1] = (y[-1] - y[-2]) / dt  # Backward difference for the last point
    return derivative

@measure_resource_usage
def higher_order_derivative_optimized(y, dt):
    derivative = np.zeros_like(y)
    # Central differences
    derivative[2:-2] = (-y[4:] + 8*y[3:-1] - 8*y[1:-3] + y[0:-4]) / (12 * dt)
    # Use lower-order methods at the boundaries
    derivative[0] = (y[1] - y[0]) / dt  # Forward difference for the first point
    derivative[1] = (y[2] - y[0]) / (2 * dt)  # Central difference for the second point
    derivative[-2] = (y[-1] - y[-3]) / (2 * dt)  # Central difference for the second last point
    derivative[-1] = (y[-1] - y[-2]) / dt  # Backward difference for the last point
    return derivative

# Function to calculate mean squared error
def calculate_error(numerical_derivative, exact_derivative):
    error = np.mean((numerical_derivative - exact_derivative) ** 2)
    # print(f"Mean Squared Error: {error}")
    return error

# Compute the numerical derivatives
central_difference_values = central_difference_derivative(sine_values, dt)
forward_difference_values = forward_difference_derivative(sine_values, dt)
backward_difference_values = backward_difference_derivative(sine_values, dt)
numpy_diff_values = np_diff(sine_values, dt)
numpy_grad_values = np_gradient(sine_values, dt)
newton_diff_values = higher_order_derivative(sine_values, dt)
newton_diff_values2 = higher_order_derivative_optimized(sine_values, dt)

# Compute the exact derivative values
exact_derivative_values = exact_derivative(x_values)

# Calculate errors
central_difference_errors = np.abs(central_difference_values - exact_derivative_values)
forward_difference_errors = np.abs(forward_difference_values - exact_derivative_values)
backward_difference_errors = np.abs(backward_difference_values - exact_derivative_values)
numpy_diff_errors = np.abs(numpy_diff_values - exact_derivative_values[:-1])
numpy_grad_errors = np.abs(numpy_grad_values - exact_derivative_values)
newton_diff_errors = np.abs(newton_diff_values - exact_derivative_values)

# Calculate and print errors
central_difference_error = calculate_error(central_difference_values, exact_derivative_values)
forward_difference_error = calculate_error(forward_difference_values, exact_derivative_values)
backward_difference_error = calculate_error(backward_difference_values, exact_derivative_values)
numpy_diff_error = calculate_error(numpy_diff_values, exact_derivative_values[:-1])
numpy_grad_error = calculate_error(numpy_grad_values, exact_derivative_values)
newton_diff_error = calculate_error(newton_diff_values, exact_derivative_values)

# Plot the sine function and its derivatives
plt.figure(figsize=(12, 8))

plt.subplot(2, 1, 1)
plt.plot(x_values, sine_values, label='sin(x)')
plt.plot(x_values, exact_derivative_values, label='cos(x)')
plt.plot(x_values, central_difference_values, label='Central Difference Derivative', linestyle='-')
plt.plot(x_values, forward_difference_values, label='Forward Difference Derivative', linestyle='-.')
plt.plot(x_values, backward_difference_values, label='Backward Difference Derivative', linestyle='--')
plt.plot(x_values[:-1], numpy_diff_values, label='numpy diff Derivative', linestyle='--')
plt.plot(x_values, numpy_grad_values, label='numpy grad Derivative', linestyle='dotted')
plt.plot(x_values, newton_diff_values, label='newton Derivative', linestyle='dotted')
plt.title('Sine Function and Its Derivatives')
plt.xlabel('x')
plt.ylabel('y')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(x_values, central_difference_errors, label=f'Central Difference Error {central_difference_error}', linestyle='-')
plt.plot(x_values, forward_difference_errors, label=f'Forward Difference Error {forward_difference_error}', linestyle='-.')
plt.plot(x_values, backward_difference_errors, label=f'Backward Difference Error {backward_difference_error}', linestyle='--')
plt.plot(x_values[:-1], numpy_diff_errors, label=f'numpy diff Error {numpy_diff_error}', linestyle='--')
plt.plot(x_values, numpy_grad_errors, label=f'numpy grad Error {numpy_grad_error}', linestyle='dotted')
plt.plot(x_values, newton_diff_errors, label=f'newton Error {newton_diff_error}', linestyle='dotted')
plt.title('Errors of Numerical Derivatives Compared to Exact Derivative')
plt.xlabel('x')
plt.ylabel('Error')
plt.legend()

plt.tight_layout()
plt.show()


# numpy diff == forward
# numpy gradient == central
# https://stackoverflow.com/questions/16841729/how-do-i-compute-the-derivative-of-an-array-in-python

# higher_order_derivative Memory usage: 0.0 MiB
# higher_order_derivative CPU usage: 0.0 seconds
# higher_order_derivative Execution time: 7.486343383789062e-05 seconds
# higher_order_derivative_optimized Memory usage: 0.0 MiB
# higher_order_derivative_optimized CPU usage: 0.0 seconds
# higher_order_derivative_optimized Execution time: 6.198883056640625e-05 seconds
