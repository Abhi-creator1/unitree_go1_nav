from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([

        # 1️⃣ Unitree UDP bridge (HIGHLEVEL)
        ExecuteProcess(
            cmd=['ros2', 'run', 'unitree_legged_real', 'ros2_udp', 'highlevel'],
            output='screen'
        ),

        # 2️⃣ HighCmd keepalive (unlocks IMU + HighState)
        Node(
            package='go1_nav',
            executable='unitree_highcmd_keepalive',
            name='unitree_highcmd_keepalive',
            output='screen'
        ),

        # 3️⃣ IMU publisher
        Node(
            package='unitree_legged_real',
            executable='imu_test',
            name='imu_test',
            output='screen'
        ),

        # 4️⃣ Lidar TCP relay
        Node(
            package='lidar_tcp_relay',
            executable='lidar_relay',
            name='lidar_tcp_relay',
            output='screen'
        ),

        # 5️⃣ Odometry node
        Node(
            package='unitree_legged_real',
            executable='unitree_odom_node',
            name='unitree_odom_node',
            output='screen'
        ),

        # 6️⃣ Static TF: base_link → laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0', '0', '3.14159', '3.14159', 'base_link', 'laser']
        ),

        # 7️⃣ SLAM Toolbox (async mode)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'use_sim_time': False
            }]
        ),
    ])
