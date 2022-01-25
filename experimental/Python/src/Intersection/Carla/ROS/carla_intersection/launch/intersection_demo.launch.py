import launch
import launch.actions
import launch.substitutions
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='carla_intersection',
            executable='rsu',
            output='screen'
        )
    ])
