import launch
import launch.actions
import launch.substitutions
from launch_ros.actions import Node
from geometry_msgs.msg import Vector3


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='carla_intersection',
            executable='rsu',
            parameters=[
                {"intersection_pos": [-0.000007632,-0.001124366,2.792485]},
                {"intersection_width": 40},
                {"nominal_speed_in_intersection": 14}
            ],
            output='screen'
        ),
        Node(
            package='carla_intersection',
            executable='vehicle',
            parameters=[
                {"vehicle_id": 0},
                {"initial_speed": [0, -8.0, 0]},
                {"spawn_point": [-122.0,   # x
                                  39.6,    # y
                                  0.3,     # z
                                 -90.0]}   # yaw
            ],
            output='screen'
        ),
        Node(
            package='carla_intersection',
            executable='carla_sim',
            parameters=[
                {"vehicle_id": 0},
                {"initial_speed": [0, -8.0, 0]},
                {"spawn_point": [-122.0,   # x
                                  39.6,    # y
                                  0.3,     # z
                                 -90.0]}   # yaw
            ],
            emulate_tty=True,
            output='screen'
        )
    ])
