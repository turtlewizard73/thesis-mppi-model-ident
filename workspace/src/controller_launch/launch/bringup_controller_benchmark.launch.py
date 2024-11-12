# Builtin modules
import os

# ROS modules
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction)  # ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.launch_context import LaunchContext

import xacro


def launch_setup(context: LaunchContext, *args, **kwargs) -> list:
    this_package_dir = get_package_share_directory('controller_launch')
    old_launch_dir = get_package_share_directory('controller_benchmark')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    gazebo_dir = get_package_share_directory('gazebo_ros')

    gui = LaunchConfiguration('gui')
    robot_type = LaunchConfiguration('rt')

    rviz_config_file = os.path.join(this_package_dir, 'nav2_default_view.rviz')
    global_map_file = os.path.join(this_package_dir, 'empty_map.yaml')
    local_map_file = os.path.join(this_package_dir, 'empty_map.yaml')

    nav_config = os.path.join(this_package_dir, 'nav2_params_benchmark.yaml')
    # nav_config = os.path.join(this_package_dir, 'default_nav2_params.yaml')

    lifecycle_nodes = [
        'map_server', 'local_map_server', 'planner_server', 'controller_server']
    # lifecycle_nodes = [
    #     'map_server', 'planner_server', 'controller_server']
    world = os.path.join(this_package_dir, 'empty_world.world')

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

    pose = {'x': '0.0', 'y': '0.0', 'z': '0.0',
            'R': '0.0', 'P': '0.0', 'Y': '0.0'}

    # -------------------------------------------------------------------------------------
    # Nodes, processes:                         -> task                 - config file
    # -------------------------------------------------------------------------------------
    # tf2_ros/static_transform_publisher        -> odom-map tf          - -
    # nav2_map_server/map_server                -> static map           - map_file
    # nav2_planner/planner_server               -> global path planner  - nav_config
    # nav2_controller/controller_server         -> local path planner   - nav_config
    # nav2_lifecycle_manager/lifecycle_manager  -> lifecycle manager    - lifecycle_nodes
    # nav2_bringup/launch/rviz_launch           -> visualization        - (TODO)
    # gazebo_ros/gzserver                       -> gazebo simulation    - world
    # gazebo_ros/gzclient                       -> gazebo simulation    - -
    # robot_state_publisher                     -> robot state          - robot_description
    # gazebo_ros/spawn_entity.py                -> spawn robot          - robot_sdf, pose
    # -------------------------------------------------------------------------------------

    return [


        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_odom_broadcaster',
            output='screen',
            # prefix=['valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no'],
            parameters=[{'use_sim_time': True}],
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),

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

        # RVIZ
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
            launch_arguments={'namespace': '',
                              'rviz_config_file': rviz_config_file,
                              'use_namespace': 'False'}.items()),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True,
                        'robot_description': robot_description}],
            # remappings=[('/tf', 'tf'),
            #             ('/tf_static', 'tf_static')]
        ),

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
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename': global_map_file},
                        {'topic_name': 'map'},
                        {'frame_id': 'map'}]),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='local_map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename': local_map_file},
                        {'topic_name': 'map_local'},
                        {'frame_id': 'map'}]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav_config,
                        {'use_sim_time': True}]),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            # prefix=['gdb -ex=r --args'],
            output='screen',
            parameters=[nav_config,
                        {'use_sim_time': True},
                        {'odom_topic': 'odom'}]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]),
    ]


def generate_launch_description() -> LaunchDescription:
    declared_arguments = [
        DeclareLaunchArgument(
            'gui', default_value='True',
            description='Whether to run gazebo headless.'
        ),

        DeclareLaunchArgument(
            'rt', default_value='waffle',
            description='Robot type: waffle, burger, or enjoy.'
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
