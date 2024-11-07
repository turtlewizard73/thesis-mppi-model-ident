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

# Convert data (row-major order) to a 2D NumPy array
costmap_array = np.array(costmap_msg.data, dtype=np.uint8).reshape((size_y, size_x))
costmap_array = np.flip(costmap_array, 0)
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
print(path_xy)

path_costs = []
draw_costmap_msg_data = costmap_msg.data
for x, y in path_xy:
    x_idx = int((x - origin_x) / map_resolution)
    y_idx = int((y - origin_y) / map_resolution)
    draw_costmap_msg_data[y_idx * size_x + x_idx] = 255


print(f'Path costs: {path_costs}')
drawed_costmap_array = np.array(draw_costmap_msg_data, dtype=np.uint8).reshape((size_y, size_x))
drawed_costmap_array = np.flip(drawed_costmap_array, 0)

plt.imshow(
    drawed_costmap_array,
    # cmap='gray',
    aspect='auto', interpolation='none',
    extent=[rect_x_min, rect_x_max, rect_y_min, rect_y_max])
plt.plot(path_xy[:, 0], path_xy[:, 1], linestyle='-', color='r', label='Path')
plt.colorbar(label='Cost')
plt.title('Costmap')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()
