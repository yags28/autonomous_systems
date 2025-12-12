# task_4/launch/gen_sync_map_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import PushRosNamespace, Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    # SLAM (TB4 nav package)
    slam_launch = os.path.join(
        get_package_share_directory('turtlebot4_navigation'),
        'launch', 'slam.launch.py'
    )

    # RViz config from TB4 desktop viz package
    rviz_config = os.path.join(
        get_package_share_directory('turtlebot4_viz'),
        'rviz', 'robot.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=TextSubstitution(text='/robot'),
            description='Robot namespace (use /robot for TB4 class setup)',
        ),

        GroupAction([
            PushRosNamespace(namespace),

            # Include SLAM (namespaced)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={'namespace': namespace}.items()
            ),

            # Launch RViz directly with the TB4 config
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                ],
            ),
        ])
    ])
