import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist


def compute_frechet_distance(P, Q):
    """
    Compute the discrete Fréchet distance between two 2D paths P and Q.
    """
    n, m = len(P), len(Q)
    ca = np.full((n, m), -1.0)

    def c(i, j):
        if ca[i, j] > -1:
            return ca[i, j]
        if i == 0 and j == 0:
            ca[i, j] = np.linalg.norm(P[0] - Q[0])
        elif i > 0 and j == 0:
            ca[i, j] = max(c(i - 1, 0), np.linalg.norm(P[i] - Q[0]))
        elif i == 0 and j > 0:
            ca[i, j] = max(c(0, j - 1), np.linalg.norm(P[0] - Q[j]))
        elif i > 0 and j > 0:
            ca[i, j] = max(min(c(i - 1, j), c(i - 1, j - 1), c(i, j - 1)),
                           np.linalg.norm(P[i] - Q[j]))
        else:
            ca[i, j] = float('inf')
        return ca[i, j]

    return c(n - 1, m - 1)


# Create two curves
t = np.linspace(0, 2 * np.pi, 100)
curve1 = np.column_stack((np.cos(t), np.sin(t)))  # Circle
curve2 = np.column_stack((np.cos(t), 0.5 * np.sin(t)))  # Ellipse

# Compute Fréchet distance
frechet_dist = compute_frechet_distance(curve1, curve2)

# Plot curves and their closest points
fig, ax = plt.subplots(figsize=(10, 6))
ax.plot(curve1[:, 0], curve1[:, 1], label='Curve 1 (Circle)', color='blue')
ax.plot(curve2[:, 0], curve2[:, 1], label='Curve 2 (Ellipse)', color='red')

# Highlight closest points along curves (approximation of Fréchet leash)
dist_matrix = cdist(curve1, curve2)
min_indices = np.argmin(dist_matrix, axis=1)
for i, j in enumerate(min_indices):
    ax.plot([curve1[i, 0], curve2[j, 0]], [curve1[i, 1], curve2[j, 1]], 'k--', alpha=0.5)

ax.set_title(f'Visualization of Fréchet Distance\nFréchet Distance ≈ {frechet_dist:.2f}')
ax.legend()
ax.set_aspect('equal')
plt.show()
