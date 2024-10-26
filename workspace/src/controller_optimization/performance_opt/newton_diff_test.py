import numpy as np
import matplotlib.pyplot as plt

# Provided function


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


# Generate sample data
dt = 0.01  # Time step
t = np.arange(0, 2 * np.pi, dt)
y = np.sin(t)  # Example function

# Compute the derivative
y_derivative = newton_diff(y, dt)

# Plot original data and derivative
plt.figure(figsize=(12, 6))
plt.plot(t, y, label="Original Function (sin(t))")
plt.plot(t, y_derivative, label="Numerical Derivative", linestyle='--')
plt.xlabel("Time (t)")
plt.ylabel("Amplitude")
plt.title("Function and its Numerical Derivative")
plt.legend()
plt.grid(True)
plt.show()
