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
print(f'Costmap size: {size_x}x{size_y}')

# Convert data (row-major order) to a 2D NumPy array
costmap_array = np.array(costmap_msg.data, dtype=np.uint8).reshape((size_y, size_x))
costmap_array = np.flip(costmap_array, 0)
# show costmap_array as an image
# plt.imshow(costmap_array, cmap='gray', aspect='auto', interpolation='none')

# costmap_array = 256 - costmap_array


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
# action_response = nav.followPath(global_plan, controller_id='FollowPathMPPI')

map_resolution = costmap_msg.metadata.resolution
height, width = costmap_array.shape
origin_x = costmap_msg.metadata.origin.position.x
origin_y = costmap_msg.metadata.origin.position.y
rect_x_min = origin_x  # The minimum x coordinate of the rectangle
rect_x_max = width * map_resolution + origin_x  # The maximum x coordinate of the rectangle
rect_y_min = - height * map_resolution / 2  # The minimum y coordinate of the rectangle
rect_y_max = height * map_resolution / 2
# Plot the costmap using matplotlib's imshow
print(costmap_array.shape)
# print(path_xy)


path_costs = []
draw_costmap_msg_data = costmap_msg.data
darw_costmap_array = costmap_array
for x, y in path_xy:
    x_idx = int((x - origin_x) / map_resolution)
    y_idx = int((y - origin_y) / map_resolution) + 1

    y_idx = height - y_idx

    # ________________
    radius_cells = int(0.2 / map_resolution)

    # Extract a square subarray around the (x_idx, y_idx) point
    size_y, size_x = costmap_array.shape

    # Define bounds for subarray to ensure they don't go out of bounds
    x_min = max(x_idx - radius_cells, 0)
    x_max = min(x_idx + radius_cells + 1, size_x)
    y_min = max(y_idx - radius_cells, 0)
    y_max = min(y_idx + radius_cells + 1, size_y)

    # Create the subarray for the relevant region
    # y_min = height - y_min
    # y_max = height - y_max
    darw_costmap_array[y_min:y_max, x_min:x_max] = 200

for x, y in path_xy:
    x_idx = int((x - origin_x) / map_resolution)
    y_idx = int((y - origin_y) / map_resolution) + 1
    y_idx = height - y_idx

    darw_costmap_array[y_idx, x_idx] = 255
    # ______________________
    # draw_costmap_msg_data[y_idx * size_x + x_idx] = 255

print(f'Path costs: {path_costs}')
drawed_costmap_array = np.array(draw_costmap_msg_data, dtype=np.uint8).reshape((size_y, size_x))
drawed_costmap_array = np.flip(drawed_costmap_array, 0)

plt.imshow(
    darw_costmap_array,
    # cmap='gray',
    aspect='auto', interpolation='none',
    extent=[rect_x_min, rect_x_max, rect_y_min, rect_y_max])
plt.plot(path_xy[:, 0], path_xy[:, 1], linestyle='-', color='r', label='Path')
plt.colorbar(label='Cost')
plt.title('Costmap')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()
