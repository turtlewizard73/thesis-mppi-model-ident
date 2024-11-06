import numpy as np
import matplotlib.pyplot as plt

# Generate a sample array of (x, y) points
np.random.seed(0)  # Set seed for reproducibility
path_xy = np.cumsum(np.random.rand(10, 2) - 0.5, axis=0)  # Generate random path of 10 points
# create a linear path
path_xy = np.array([[0, 0], [1, 1], [2, 2], [3, 3], [4, 4],
                   [5, 5], [6, 6], [7, 7], [8, 8], [9, 9], [10, 10]])

# Calculate the distance between each consecutive point
distances = np.sqrt(np.sum(np.diff(path_xy, axis=0)**2, axis=1))
total_distance = np.sum(distances)

# Print the path points and total distance
print("Path points (x, y):")
print(path_xy)
print("\nTotal path distance:", total_distance)

# Plot the path
plt.figure(figsize=(8, 6))
plt.plot(path_xy[:, 0], path_xy[:, 1], marker='o', linestyle='-', color='b', label='Path')
plt.scatter(path_xy[:, 0], path_xy[:, 1], color='red')  # Mark each point
for i, (x, y) in enumerate(path_xy):
    plt.text(x, y, f"{i}", fontsize=9, ha="right")  # Label each point with its index

plt.title(f"Path Visualization\nTotal Distance: {total_distance:.2f}")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.grid(True)
plt.show()
