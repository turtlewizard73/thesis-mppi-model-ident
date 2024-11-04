import os

# ROS modules
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    IncludeLaunchDescription, ExecuteProcess, GroupAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    this_package_dir = get_package_share_directory('controller_launch')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    namespace = ''
    use_sim_time = True

    urdf = os.path.join(this_package_dir, 'turtlebot3_waffle.urdf')
    with open(urdf, 'r', encoding='utf-8') as infp:
        robot_description = infp.read()

    # multi robot something from og launch file
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    robot_name = 'turtlebot3_waffle'
    robot_sdf = os.path.join(this_package_dir, 'turtlebot3_waffle.model')
    pose = {'x': '-2.0', 'y': '-0.5', 'z': '0.01', 'R': '0', 'P': '0', 'Y': '0'}

    world = os.path.join(this_package_dir, 'world_only.world')
    rviz_config_file = os.path.join(this_package_dir, 'nav2_default_view.rviz')

    start_gazebo_server_cmd = ExecuteProcess(
        # condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[this_package_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        # condition=IfCondition(PythonExpression(
        #     [use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[this_package_dir], output='screen')

    start_robot_state_publisher_cmd = Node(
        # condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
        remappings=remappings)

    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-file', robot_sdf,
            '-robot_namespace', namespace,
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])

    use_namespace = 'false'
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        # condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file}.items())

    use_composition = 'false'
    params_file = os.path.join(this_package_dir, 'nav2_params.yaml')
    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ('/', namespace)},
        condition=IfCondition(use_namespace))

    map_yaml_file = os.path.join(this_package_dir, 'turtlebot3_world.yaml')
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    autostart = 'true'
    use_respawn = 'false'
    log_level = 'info'

    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
            # condition=IfCondition(PythonExpression(['not ', slam])),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(this_package_dir, 'turtlebot_navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),
    ])

    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(nav2_bringup_dir, 'launch', 'tb3_simulation_launch.py')),
        #     launch_arguments={'headless': 'False'}.items()),

        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        start_gazebo_spawner_cmd,

        start_robot_state_publisher_cmd,
        rviz_cmd,

        # bringup_cmd_group
    ])
