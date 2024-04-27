from utils import ParamInterface
import rclpy

rclpy.init()

# Create an instance of the ParamInterface class
param_interface = ParamInterface()

# Call the functions of the ParamInterface class
p1 = param_interface.get_param(
    server_name='local_costmap/local_costmap',
    param_name='footprint_padding'
)
print(p1)


p2 = param_interface.set_param(
    param_name='footprint_padding',
    param_value=(p1 * 0.1),
    server_name='local_costmap/local_costmap'
)

rclpy.shutdown()
exit(0)
