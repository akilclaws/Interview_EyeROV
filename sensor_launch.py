from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_package',
            executable='sensor_node',
            name='sensor_node',
            output='screen',
            parameters=[
                {'interval': 1000}  # Default interval in milliseconds
            ]
        )
    ])

