# Builtin modules
import os

# ROS modules
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

# ROS message types
# ...


def generate_launch_description():
    this_package_dir = get_package_share_directory('controller_benchmark')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_basic_config = False
    use_gui = True

    map_file = os.path.join(this_package_dir, '10by10_empty.yaml')
    nav_config = os.path.join(this_package_dir, 'nav2_params_mppi.yaml') if \
        use_basic_config is False else os.path.join(this_package_dir, 'nav2_params.yaml')
    lifecycle_nodes = ['map_server', 'planner_server', 'controller_server']
    world = ''  # os.path.join(nav2_bringup_dir, 'worlds', 'world_only.model')

    urdf = os.path.join(nav2_bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as urdf_file:
        robot_description = urdf_file.read()

    gazebo_dir = get_package_share_directory('gazebo_ros')

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

    robot_name = 'turtlebot3_waffle'
    robot_sdf = os.path.join(nav2_bringup_dir, 'worlds', 'waffle.model')
    pose = {'x': '0.0', 'y': '0.0', 'z': '0.0',
            'R': '0.0', 'P': '0.0', 'Y': '0.0'}

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_odom_broadcaster',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename': map_file},
                        {'topic_name': 'map'},
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

        # RVIZ
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
            launch_arguments={'namespace': '',
                              'use_namespace': 'False'}.items()),

        # Gazebo simulation
        # ExecuteProcess(
        #     cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
        #          '-s', 'libgazebo_ros_factory.so',
        #          '-s', 'libgazebo_ros_state.so', world],
        #     cwd=[nav2_bringup_dir], output='screen'),

        # ExecuteProcess(
        #     condition=IfCondition(str(use_gui)),
        #     cmd=['gzclient'],
        #     cwd=[nav2_bringup_dir], output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_dir, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world,
                              'pause': 'false',
                              'verbose': 'true'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_dir, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(str(use_gui))),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True,
                        'robot_description': robot_description}],
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static')]),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity', robot_name,
                '-file', robot_sdf,
                '-robot_namespace', '',
                '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
                '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])
    ])
