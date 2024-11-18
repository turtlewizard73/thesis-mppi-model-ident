import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# Example: Original plan (path_xy) and simulated paths (odom_xy)
np.random.seed(42)

# Create original path (path_xy)
points = np.linspace(0, 100, 101)
path_xy = np.array(
    [[i, np.sin(i / 10)] for i in points])

odom_xy_1 = np.array(
    [[i, np.sin(i / 10) + np.random.normal(0, 0.1)] for i in np.linspace(0, 100, 110)])

odom_xy_2 = np.array(
    [[i, np.sin(i / 10) + np.random.normal(0, 0.15)] for i in np.linspace(0, 100, 90)])

print('Original Path Shape:', path_xy.shape)
print('Original Odom 1 Shape:', odom_xy_1.shape)
print('Original Odom 2 Shape:', odom_xy_2.shape)

plt.figure()
plt.plot(path_xy[:, 0], path_xy[:, 1], label='Original Path', color='blue', linewidth=2)
plt.plot(odom_xy_1[:, 0], odom_xy_1[:, 1], label='Original Odom 1', color='red', linestyle='--')
plt.plot(odom_xy_2[:, 0], odom_xy_2[:, 1], label='Original Odom 2', color='green', linestyle='--')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Original Path and Simulated Paths')
plt.legend()
plt.grid(True)
plt.show()


def interpolate_path(original_path, simulated_path, num_points):
    """Interpolate a simulated path to match the number of points in the original path."""
    # Cumulative distances for interpolation
    original_distances = np.cumsum(np.sqrt(np.sum(np.diff(original_path, axis=0)**2, axis=1)))
    simulated_distances = np.cumsum(np.sqrt(np.sum(np.diff(simulated_path, axis=0)**2, axis=1)))

    # Include the start point (distance 0)
    original_distances = np.insert(original_distances, 0, 0)
    simulated_distances = np.insert(simulated_distances, 0, 0)

    # Linear interpolation to match original distances
    interp_func = interp1d(simulated_distances, simulated_path, axis=0,
                           kind='linear', fill_value="extrapolate")
    interpolated_path = interp_func(np.linspace(0, original_distances[-1], num_points))

    return interpolated_path


# Interpolate simulated paths
num_points = path_xy.shape[0]
interpolated_paths = [interpolate_path(path_xy, odom, num_points) for odom in odom_paths]

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(path_xy[:, 0], path_xy[:, 1], label='Original Path', color='blue', linewidth=2)
# plot the originals
for i, odom_path in enumerate(odom_paths):
    plt.plot(odom_path[:, 0], odom_path[:, 1],
             label=f'Original Odom {i+1}', linestyle='--', alpha=0.7)


for i, interp_path in enumerate(interpolated_paths):
    plt.plot(interp_path[:, 0], interp_path[:, 1],
             label=f'Interpolated Odom {i+1}', linestyle='--', alpha=0.7)

print('Original Path Shape:', path_xy.shape)
for i, odom_path in enumerate(odom_paths):
    print(f'Original Odom {i+1} Shape:', odom_path.shape)

for i, interp_path in enumerate(interpolated_paths):
    print(f'Interpolated Odom {i+1} Shape:', interp_path.shape)

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Interpolation and Resampling Demo')
plt.legend()
plt.grid(True)
plt.show()
