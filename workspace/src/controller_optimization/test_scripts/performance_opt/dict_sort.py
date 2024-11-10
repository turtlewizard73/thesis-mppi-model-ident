import timeit
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

# Sorting function
def sort_dict(my_dict):
    return dict(sorted(my_dict.items(), key=lambda item: (item[0][0].islower(), item[0])))

# Generate a dictionary of a given size
def generate_dict(size):
    return {f"key{str(i).zfill(4)}": i for i in range(size)}

# Test dictionary sizes
dict_sizes = [10, 100, 1000, 5000, 10000, 20000, 50000, 100000, 200000, 500000]
times = []

# Measure the time taken to sort each dictionary size
for size in dict_sizes:
    test_dict = generate_dict(size)
    time_taken = timeit.timeit(lambda: sort_dict(test_dict), number=10) / 10  # Average over 10 runs
    times.append(time_taken)

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(dict_sizes, times, 'o-', label='Measured Time')

# Define potential time complexities
def linear(n, a, b): return a * n + b
def n_log_n(n, a, b): return a * n * np.log(n) + b
def quadratic(n, a, b): return a * n**2 + b

# Fit each function to the measured data
popt_linear, _ = curve_fit(linear, dict_sizes, times)
popt_nlogn, _ = curve_fit(n_log_n, dict_sizes, times)
popt_quadratic, _ = curve_fit(quadratic, dict_sizes, times)

# Plot fitted curves
sizes = np.linspace(min(dict_sizes), max(dict_sizes), 100)
plt.plot(sizes, linear(sizes, *popt_linear), '--', label='O(n)')
plt.plot(sizes, n_log_n(sizes, *popt_nlogn), '--', label='O(n log n)')
plt.plot(sizes, quadratic(sizes, *popt_quadratic), '--', label='O(n^2)')

# Customize plot
plt.xlabel('Dictionary Size (n)')
plt.ylabel('Time (seconds)')
plt.title('Time Complexity of Sorting a Dictionary by Lowercase-First Key Order')
plt.legend()
plt.grid(True)
plt.show()

# Determine best-fit complexity by comparing residuals
residuals = {
    "O(n)": np.sum((linear(np.array(dict_sizes), *popt_linear) - times) ** 2),
    "O(n log n)": np.sum((n_log_n(np.array(dict_sizes), *popt_nlogn) - times) ** 2),
    "O(n^2)": np.sum((quadratic(np.array(dict_sizes), *popt_quadratic) - times) ** 2),
}
best_fit = min(residuals, key=residuals.get)
print(f"Best fit complexity: {best_fit}")
