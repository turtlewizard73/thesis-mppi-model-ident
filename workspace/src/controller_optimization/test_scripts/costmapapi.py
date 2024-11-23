import matplotlib
import copy
import rclpy
import matplotlib.pyplot as plt
import numpy as np

import nav2_simple_commander.robot_navigator as nav2
from nav2_simple_commander.costmap_2d import PyCostmap2D
from geometry_msgs.msg import PoseStamped


rclpy.init()

nav = nav2.BasicNavigator()
costmap_msg = nav.getLocalCostmap()


size_x = costmap_msg.metadata.size_x
size_y = costmap_msg.metadata.size_y
print(f'Costmap size: size_x: {size_x} | size_y: {size_y}')

# Convert data (row-major order) to a 2D NumPy array
costmap_array = np.array(costmap_msg.data, dtype=np.uint8).reshape((size_y, size_x))
costmap_array = np.flip(costmap_array, 0)

height, width = costmap_array.shape
print(f'Costmap array shape: height: {height} | width: {width}')

# ______________________________________________________________________________

start = PoseStamped()
start.header.frame_id = 'map'
start.pose.position.x = 0.0
start.pose.position.y = 0.0

goal = PoseStamped()
goal.header.frame_id = 'map'
goal.pose.position.x = 10.3
goal.pose.position.y = 0.0

global_plan = nav.getPath(start, goal, 'GridBased', use_start=True)
path_xy = np.array([
    (ps.pose.position.x, ps.pose.position.y) for ps in global_plan.poses])

# ______________________________________________________________________________
map_resolution = costmap_msg.metadata.resolution
radius_in_meter = 0.22
radius_cells = int(radius_in_meter / map_resolution) + 1


map_origin_x = costmap_msg.metadata.origin.position.x
map_origin_y = costmap_msg.metadata.origin.position.y
masks = np.zeros_like(costmap_array, dtype=bool)
size_y, size_x = costmap_array.shape
for x, y in path_xy:
    x_idx = int((x - map_origin_x) / map_resolution)
    y_idx = int((y - map_origin_y) / map_resolution) + 1
    y_idx = size_y - y_idx

    # TODO: CHANGE ME TO DISPLAY THE PATH
    # costmap_array[y_idx, x_idx] = 255

    Y, X = np.ogrid[:size_y, :size_x]
    dist_from_center = np.sqrt((X - x_idx)**2 + (Y - y_idx)**2)

    mask = dist_from_center <= radius_cells

    masks = np.logical_or(masks, mask)

# test_mask = create_circular_mask(size_y, size_x, radius=radius_cells)
# print(test_mask)
masked_img = costmap_array.copy()
masked_img[~masks] = 0


origin_x = costmap_msg.metadata.origin.position.x
origin_y = costmap_msg.metadata.origin.position.y
rect_x_min = origin_x  # The minimum x coordinate of the rectangle
rect_x_max = size_x * map_resolution + origin_x  # The maximum x coordinate of the rectangle
rect_y_min = - size_y * map_resolution / 2  # The minimum y coordinate of the rectangle
rect_y_max = size_y * map_resolution / 2
plt.imshow(
    masked_img,
    # cmap='gray',
    aspect='auto', interpolation='none',
    extent=[rect_x_min, rect_x_max, rect_y_min, rect_y_max])
plt.plot(path_xy[:, 0], path_xy[:, 1], marker='x', linestyle='-', color='r', label='Path', )
plt.colorbar(label='Cost')
plt.title('Costmap')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.show()
