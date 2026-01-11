import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Package Directories
    pkg_bot_bringup = get_package_share_directory('bot_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # Files
    urdf_file = os.path.join(pkg_bot_bringup, 'urdf',
                             'simple_bot.urdf')
    world_file = os.path.join(
        pkg_bot_bringup, 'worlds', 'mapping_warehouse.sdf')

    # Environment Variables to find models
    model_path = os.path.join(pkg_bot_bringup, 'models')

    # 1. Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(urdf_file).read()
        }]
    )

    # 3. Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3',
            '-file', urdf_file,
            '-x', '-1.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # 4. Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 5. Simple Odometry Node (Calculates odom from joint states)
    # This provides perfect TF synchronization by using the same source as Robot State Publisher
    simple_odometry_node = Node(
        package='bot_bringup',
        executable='simple_odometry.py',
        name='simple_odometry',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 6. SLAM Toolbox (delayed start to allow Gazebo clock to stabilize)
    slam_params_file = os.path.join(
        pkg_bot_bringup, 'config', 'mapper_params_online_async.yaml')
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': slam_params_file
        }.items()
    )

    # 7. RViz2 (delayed start to allow Gazebo clock to stabilize)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.join(pkg_bot_bringup, 'rviz', 'mapping.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Delay SLAM and RViz by 10/11 seconds to let Gazebo clock stabilize
    delayed_slam = TimerAction(
        period=10.0,
        actions=[slam_toolbox]
    )

    delayed_rviz = TimerAction(
        period=11.0,
        actions=[rviz]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gz_model_path',
            default_value='',
            description='Path to Gazebo models'
        ),

        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[
                LaunchConfiguration('gz_model_path'),
                os.path.join(pkg_bot_bringup, 'models'),
            ]
        ),
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        # Publishes odom->base_footprint TF (Robust Method)
        simple_odometry_node,
        delayed_slam,          # Start SLAM after 10 seconds
        delayed_rviz           # Start RViz after 11 seconds
    ])
