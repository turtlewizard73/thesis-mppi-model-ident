import numpy as np
import matplotlib.pyplot as plt

def newton_diff0(y: np.ndarray, dt: float) -> np.ndarray:
    derivative = np.zeros_like(y)
    # Central differences
    derivative[2:-2] = (-y[4:] + 8*y[3:-1] - 8*y[1:-3] + y[0:-4]) / (12 * dt)
    # Use lower-order methods at the boundaries
    derivative[0] = (y[1] - y[0]) / dt  # Forward difference for the first point
    derivative[1] = (y[2] - y[0]) / (2 * dt)  # Central difference for the second point
    derivative[-2] = (y[-1] - y[-3]) / (2 * dt)  # Central difference for the second last point
    derivative[-1] = (y[-1] - y[-2]) / dt  # Backward difference for the last point
    return derivative

def newton_diff(y: np.ndarray, t: np.ndarray) -> np.ndarray:
    derivative = np.zeros_like(y)
    dt_forward = t[1:] - t[:-1]  # Forward time differences

    # Central differences for interior points
    derivative[2:-2] = (
        -y[4:] / ((t[4:] - t[2:-2]) * (t[4:] - t[3:-1])) +
        8 * y[3:-1] / ((t[3:-1] - t[2:-2]) * (t[3:-1] - t[1:-3])) -
        8 * y[1:-3] / ((t[2:-2] - t[1:-3]) * (t[3:-1] - t[1:-3])) +
        y[0:-4] / ((t[2:-2] - t[0:-4]) * (t[1:-3] - t[0:-4]))
    )

    # Forward difference for the first point
    derivative[0] = (y[1] - y[0]) / dt_forward[0]

    # Central difference for the second point
    derivative[1] = (y[2] - y[0]) / (t[2] - t[0])

    # Central difference for the second-to-last point
    derivative[-2] = (y[-1] - y[-3]) / (t[-1] - t[-3])

    # Backward difference for the last point
    derivative[-1] = (y[-1] - y[-2]) / dt_forward[-1]

    return derivative

# Example data: velocity and time
velocity = np.array([0, 2, 4, 6, 8, 10])  # Example velocity data
time = np.array([0, 1, 1.5, 3, 4, 5])  # Corresponding time points with non-uniform intervals

# Calculate the derivative using the modified Newton method
acceleration_newton = newton_diff(velocity, time)

# Calculate the derivative using NumPy's diff method (adjusted for non-uniform time steps)
# NumPy's diff gives differences between consecutive elements, so we divide by dt.
dt = np.diff(time)
acceleration_numpy = np.diff(velocity) / dt

# To match the length of the original array, prepend the first derivative value
# For comparison purposes.
acceleration_numpy = np.concatenate(([acceleration_numpy[0]], acceleration_numpy))

# Plot the results for comparison
plt.figure(figsize=(10, 6))
plt.plot(time, acceleration_newton, label='Newton Method Derivative', marker='o')
plt.plot(time, acceleration_numpy, label='NumPy Diff Derivative', marker='x', linestyle='--')
plt.plot(time, newton_diff0(velocity, np.mean(time)), label='Newton Method Derivative 0', marker='o', linestyle='--')
plt.xlabel('Time')
plt.ylabel('Acceleration')
plt.title('Comparison of Derivatives: Newton Method vs NumPy Diff')
plt.legend()
plt.grid(True)
plt.show()
