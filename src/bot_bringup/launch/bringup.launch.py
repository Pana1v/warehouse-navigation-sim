import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    GroupAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Package Directories
    pkg_bot_bringup = get_package_share_directory('bot_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Files
    urdf_file = os.path.join(pkg_bot_bringup, 'urdf',
                             'simple_bot.urdf')
    world_file = os.path.join(
        pkg_bot_bringup, 'worlds', 'simple_warehouse.sdf')
    map_file = os.path.join(pkg_bot_bringup, 'maps', 'warehouse_map.yaml')
    nav2_params_file = os.path.join(
        pkg_bot_bringup, 'config', 'nav2_mppi_params.yaml')

    # Environment Variables
    model_path = os.path.join(pkg_bot_bringup, 'models')

    # Rewrite params file to include the map path
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites={
            'yaml_filename': map_file,
        },
        convert_types=True
    )

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
            '-x', '-2.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/box_cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            # /tf removed - EKF publishes odom->base_footprint
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # EKF: Fuses wheel odometry (X/Y) + IMU (yaw orientation)
    # Wheel odom provides translation, IMU provides rotation
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_bot_bringup, 'config', 'ekf.yaml'),
            {'use_sim_time': True}
        ]
    )

    # 5. Simple Odometry Node (Calculates odom from joint states)
    # This provides better TF synchronization than the Gazebo bridge
    # simple_odometry = Node(
    #     package='bot_bringup',
    #     executable='simple_odometry.py',
    #     name='simple_odometry',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )

    # 6. Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            configured_params,
            {'use_sim_time': True}
        ]
    )

    # 7. AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            configured_params,
            {'use_sim_time': True}
        ]
    )

    # 8. Lifecycle Manager for Localization
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 10.0,  # Increased for robustness
            'attempt_respawn_reconnection': True,  # Retry on failure
            'bond_respawn_max_duration': 30.0,  # Max retry duration
            'node_names': ['map_server', 'amcl']
        }]
    )

    # 9. Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            configured_params,
            {'use_sim_time': True}
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav')
        ]
    )

    # 9. Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            configured_params,
            {'use_sim_time': True}
        ]
    )

    # 10. Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            configured_params,
            {'use_sim_time': True}
        ]
    )

    # 11. BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            configured_params,
            {'use_sim_time': True}
        ]
    )

    # 12. Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            configured_params,
            {'use_sim_time': True}
        ]
    )

    # 13. Velocity Smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[
            configured_params,
            {'use_sim_time': True}
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ]
    )

    # 14. Lifecycle Manager for Navigation (delayed)
    lifecycle_manager_navigation = TimerAction(
        period=15.0,  # Wait for navigation nodes to fully start
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'bond_timeout': 10.0,  # Increased for robustness
                    'attempt_respawn_reconnection': True,  # Retry on failure
                    'bond_respawn_max_duration': 30.0,  # Max retry duration
                    'node_names': [
                        'controller_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'velocity_smoother'
                    ]
                }]
            )
        ]
    )

    # 15. RViz2
    # Use the standard Nav2 RViz configuration
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    rviz_config_file = os.path.join(
        pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 16. Warehouse Navigation Server (delayed start)
    warehouse_nav_server = TimerAction(
        period=9.0,  # Wait for Nav2 to fully start
        actions=[
            Node(
                package='bot_bringup',
                executable='warehouse_nav_server.py',
                name='warehouse_nav_server',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # 17. Patrol Controller (Moving Box)
    patrol_controller = Node(
        package='bot_bringup',
        executable='patrol_controller.py',
        name='patrol_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        # Global sim time setting
        SetParameter(name='use_sim_time', value=True),

        DeclareLaunchArgument(
            'gz_model_path',
            default_value='',
            description='Path to Gazebo models'
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                LaunchConfiguration('gz_model_path'),
                model_path,
            ]
        ),

        # Core simulation (EKF provides odom->base_footprint TF)
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        ekf_node,

        # Localization (map_server + AMCL)
        TimerAction(
            period=1.0,  # More time for Gazebo/TF to stabilize
            actions=[map_server, amcl]
        ),

        # Localization lifecycle manager
        TimerAction(
            period=2.0,  # Give AMCL more time to initialize
            actions=[lifecycle_manager_localization]
        ),

        # Navigation stack (delayed to wait for localization)
        TimerAction(
            period=3.0,  # Wait for localization to be fully ready
            actions=[
                controller_server,
                planner_server,
                behavior_server,
                bt_navigator,
                waypoint_follower,
                velocity_smoother,
            ]
        ),

        # Navigation lifecycle manager
        lifecycle_manager_navigation,

        # Visualization
        rviz,

        # Custom navigation server
        warehouse_nav_server,

        # Patrol Controller
        patrol_controller
    ])
