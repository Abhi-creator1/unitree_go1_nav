from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

MAP_FILE = "/home/a/unitree_ros2_ws/src/go1_nav/map/unitree_map.yaml"
NAV2_PARAMS = "/home/a/unitree_ros2_ws/src/go1_nav/config/nav2_params.yaml"

def generate_launch_description():

    return LaunchDescription([

        # =========================
        # 1️⃣ Unitree UDP bridge
        # =========================
        ExecuteProcess(
            cmd=['ros2', 'run', 'unitree_legged_real', 'ros2_udp', 'highlevel'],
            output='screen'
        ),

        # =========================
        # 2️⃣ HighCmd keepalive
        # =========================
        Node(
            package='go1_nav',
            executable='unitree_highcmd_keepalive',
            name='unitree_highcmd_keepalive',
            output='screen'
        ),

        # =========================
        # 3️⃣ IMU publisher
        # =========================
        Node(
            package='unitree_legged_real',
            executable='imu_test',
            name='imu_test',
            output='screen'
        ),

        # =========================
        # 4️⃣ Lidar TCP relay
        # =========================
        Node(
            package='lidar_tcp_relay',
            executable='lidar_relay',
            name='lidar_tcp_relay',
            output='screen'
        ),

        # =========================
        # 5️⃣ Odometry
        # =========================
        Node(
            package='unitree_legged_real',
            executable='unitree_odom_node',
            name='unitree_odom_node',
            output='screen'
        ),

        # =========================
        # 6️⃣ Static TF: base_link → laser
        # =========================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0', '0', '3.14159', '3.14159', 'base_link', 'laser']
        ),

        # =========================
        # 7️⃣ Localization (slam_toolbox)
        # =========================
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox_localization',
            output='screen',
            parameters=[{
                'map_file_name': MAP_FILE,
                'use_sim_time': False
            }]
        ),

        # =========================
        # 8️⃣ Nav2 stack
        # =========================
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[NAV2_PARAMS]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[NAV2_PARAMS]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[NAV2_PARAMS]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[NAV2_PARAMS]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': MAP_FILE
            }]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'controller_server',
                    'planner_server',
                    'bt_navigator',
                    'behavior_server'
                ]
            }]
        ),

        # =========================
        # 9️⃣ cmd_vel → HighCmd adapter
        # =========================
        Node(
            package='go1_nav',
            executable='cmd_vel_to_highcmd',
            name='cmd_vel_to_highcmd',
            output='screen'
        ),
    ])
