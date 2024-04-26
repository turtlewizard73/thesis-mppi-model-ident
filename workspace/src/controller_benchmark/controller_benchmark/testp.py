from utils import ParamInterface

# Create an instance of the ParamInterface class
param_interface = ParamInterface()

# Call the functions of the ParamInterface class
p1 = param_interface.get_param(
    param_name='costmap'
    server_name='local_costmap'
)
print(p1)


footprint = param_interface.get_param(
    param_name='robot_footprint'
    server_name='local_costmap'
)
print(footprint)
p2 = param_interface.set_param(
    param_name='robot_footprint'
    param_value=footprint * 2,
    server_name='local_costmap'
)
