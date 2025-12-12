from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task_1',
            executable='talker',
            output='screen'),
        Node(
            package='task_2',
            executable='service',
            output='screen'),
    ])