import os

# ROS modules
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext

import xacro


def launch_setup(context: LaunchContext, *args, **kwargs) -> list:
    this_package_dir = get_package_share_directory('controller_launch')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    gazebo_dir = get_package_share_directory('gazebo_ros')

    gui = LaunchConfiguration('gui')
    use_composition = LaunchConfiguration('use_composition')
    namespace = ''
    use_sim_time = 'true'
    autostart = 'true'
    use_respawn = 'False'
    nav2_params_type = LaunchConfiguration('p')
    loc_params_file = os.path.join(this_package_dir, 'nav2_params_full_default.yaml')
    nav2_params_file = os.path.join(this_package_dir, 'nav2_params_full_default.yaml')
    if nav2_params_type.perform(context=context) == 'enjoy':
        nav2_params_file = os.path.join(this_package_dir, 'nav2_params_full_enjoy.yaml')

    rviz_config_file = os.path.join(this_package_dir, 'rviz_default_view.rviz')
    map_yaml_file = os.path.join(this_package_dir, 'turtlebot3_world.yaml')
    world = os.path.join(this_package_dir, 'world_only.world')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    # nav2 shenanigans
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # custom robot type
    robot_type = LaunchConfiguration('rt')
    match robot_type.perform(context=context):
        case 'enjoy':
            urdf = os.path.join(this_package_dir, 'service_robot.urdf')
            robot_name = 'service_robot'
            robot_sdf = os.path.join(this_package_dir, 'burger_model.sdf')  # TODO: change
            robot_description = xacro.process_file(
                os.path.join(this_package_dir, 'service_robot.xacro')).toprettyxml()
        case 'burger':
            urdf = os.path.join(this_package_dir, 'turtlebot3_burger.urdf')
            robot_name = 'turtlebot3_burger'
            robot_sdf = os.path.join(this_package_dir, 'burger_model.sdf')
            with open(urdf, 'r') as urdf_file:
                robot_description = urdf_file.read()
        case _:
            urdf = os.path.join(this_package_dir, 'turtlebot3_waffle.urdf')
            robot_name = 'turtlebot3_waffle'
            robot_sdf = os.path.join(this_package_dir, 'waffle_noiseless.model')
            with open(urdf, 'r') as urdf_file:
                robot_description = urdf_file.read()

    # urdf = os.path.join(nav2_bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    # with open(urdf, 'r', encoding='utf-8') as file:
    #     robot_description = file.read()

    # robot_name = 'turtlebot3_waffle'
    # robot_sdf = os.path.join(this_package_dir, 'waffle_noiseless.model')
    pose = {'x': '-2.0', 'y': '-0.5', 'z': '0.01', 'R': '0', 'P': '0', 'Y': '0'}

    # -------------------------------------------------------------------------------------
    # Nodes, processes:                         -> task                 - config file
    # -------------------------------------------------------------------------------------
    # nav2_bringup/launch/rviz_launch           -> visualization        - nav2 rviz config
    # gazebo_ros/gzserver                       -> gazebo simulation    - world
    # gazebo_ros/gzclient                       -> gazebo simulation    - visual
    # robot_state_publisher                     -> robot state          - robot_description
    # gazebo_ros/spawn_entity.py                -> spawn robot          - robot_sdf, pose
    # -------------------------------------------------------------------------------------
    # nav2 container
    # nav2/localization_launch.py               -> localization         - map, params
    # turtlebot_navigation.launch.py            -> navigation           - params

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_namespace': 'False',
                'rviz_config': rviz_config_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_dir, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world,
                              'pause': 'false',
                              'verbose': 'true'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_dir, 'launch', 'gzclient.launch.py')),
            launch_arguments={'verbose': 'true'}.items(),
            condition=IfCondition(gui)),

        # state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': True,
                 'robot_description': robot_description}],
            remappings=remappings),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity', robot_name,
                '-file', robot_sdf,
                # '-topic', 'robot_description',
                '-timeout', '60',
                '-robot_namespace', '',
                '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
                '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']],
            condition=IfCondition(str(robot_type.perform(context) != 'enjoy'))),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'service_robot',
                '-timeout', '60',
                '-topic', 'robot_description',
                '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
                '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']],
            output='screen',
            condition=IfCondition(str(robot_type.perform(context) == 'enjoy'))),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='log',
            parameters=[
                {'use_sim_time': True,
                 'rate': 10,
                 'source_list': ['/wheel_joint_states']}],
            condition=IfCondition(str(robot_type.perform(context) == 'enjoy'))),

        # nav2 stuff
        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'map': map_yaml_file,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': loc_params_file,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
                'container_name': 'nav2_container'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(this_package_dir, 'turtlebot_navigation.launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': nav2_params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),
    ]


def generate_launch_description() -> LaunchDescription:
    declared_arguments = [
        DeclareLaunchArgument(
            'gui', default_value='True',
            description='Whether to run gazebo headless.'),

        DeclareLaunchArgument(
            'use_composition', default_value='True',
            description='Whether to use composition.'),

        DeclareLaunchArgument(
            'rt', default_value='waffle',
            description='Robot type: waffle, burger, or enjoy.'
        ),

        DeclareLaunchArgument(
            'p', default_value='default',
            description='Nav2 parameters to use.'
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
