import numpy as np

def reject_outliers(poses, m=2.0):
    poses_array = np.array(poses)
    x = poses_array[:, 0]  # Extract x-coordinates
    y = poses_array[:, 1]  # Extract y-coordinates

    # Filter outliers for x-coordinates
    x_filtered = reject_outliers_1d(x, m)
    # Filter outliers for y-coordinates
    y_filtered = reject_outliers_1d(y, m)

    # Combine filtered x and y coordinates into a list of points
    filtered_poses = [[x, y] for x, y in zip(x_filtered, y_filtered)]

    return filtered_poses

def reject_outliers_1d(data, m=2.0):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d / mdev if mdev else np.zeros(len(d))
    return data[s < m]

poses = [[1,1],[2,1],[3,2],[100, 100],[5,5]]  # Example poses

filtered_poses = reject_outliers(poses)
print(filtered_poses)