from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_challenge',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        Node(
            package='obstacle_challenge',
            executable='navigator_node',
            name='navigator_node',
            output='screen'
        ),
        Node(
            package='obstacle_challenge',
            executable='imu_node',
            name='imu_node',
            output='screen'
        )
    ])
