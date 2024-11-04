import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from typing import List, Optional, Dict, Any


class ParameterManager(Node):
    def __init__(self, target_node_name: str) -> None:
        super().__init__('parameter_manager')
        self.target_node_name: str = target_node_name
        self.get_parameters_client = self.create_client(
            GetParameters, f"{target_node_name}/get_parameters")
        self.set_parameters_client = self.create_client(
            SetParameters, f"{target_node_name}/set_parameters")

        self.get_logger().info(f'ParameterManager for {target_node_name} initialized')

    def get_parameters(self, parameter_names: List[str]) -> Optional[List[Any]]:
        request = GetParameters.Request()
        request.names = parameter_names

        while not self.get_parameters_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        future = self.get_parameters_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().values
        else:
            self.get_logger().error('Failed to get parameters')
            return None

    def set_parameters(self, parameters: Dict[str, Any]) -> Optional[List[bool]]:
        self.get_logger().info(f'Setting parameters for {self.target_node_name}.')

        request = SetParameters.Request()
        for name, value in parameters.items():
            if isinstance(value, bool):
                param_value = ParameterValue(
                    type=ParameterType.PARAMETER_BOOL, bool_value=value)
            elif isinstance(value, int):
                param_value = ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER, integer_value=value)
            elif isinstance(value, float):
                param_value = ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=value)
            elif isinstance(value, str):
                param_value = ParameterValue(
                    type=ParameterType.PARAMETER_STRING, string_value=value)
            else:
                self.get_logger().error(f'Unsupported parameter type: {type(value)}')
                return None

            request.parameters.append(Parameter(name=name, value=param_value))

        # self.get_logger().debug(f'Parameters: {request.parameters}')

        while not self.set_parameters_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        future = self.set_parameters_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return [result.successful for result in future.result().results]
        else:
            self.get_logger().error('Failed to set parameters')
            return None

# example usage
# rclpy.init()
# param_manager = ParameterManager('controller_server')

# pstr = 'FollowPathMPPI.CostCritic.cost_weight'
# p1 = param_manager.get_parameters([pstr])
# p1 = p1[0].double_value
# print(p1)

# param_manager.set_parameters({pstr: 3.81})

# p1 = param_manager.get_parameters([pstr])
# p1 = p1[0].double_value
# print(p1)


# rclpy.shutdown()
