import launch
import launch.actions
import launch.substitutions
from launch_ros.actions import Node
from geometry_msgs.msg import Vector3


def generate_launch_description():
    positions = [{   \
        "x": -122.0,  \
        "y": 39.6,    \
        "z": 0.3,     \
        "yaw": -90.0, \
        }, {          \
        "x": -177.77, \
        "y": 6.48,    \
        "z": 0.3,     \
        "yaw": 0.0    \
        }, {          \
        "x": -132.77, \
        "y": -40,     \
        "z": 0.3,     \
        "yaw": 90.0   \
        }, {          \
        "x": -80.77,  \
        "y": -4.5,    \
        "z": 0.3,     \
        "yaw": 180.0} \
        ]

    initial_speeds = [
        [0.0, -8.0, 0.0], [8.0, 0.0, 0.0], [0.0, 8.0, 0.0], [-8.0, 0.0, 0.0]
    ]

    nodes = []
    nodes.append(
        Node(
            package='carla_intersection',
            executable='rsu',
            parameters=[
                {"intersection_pos": [-0.000007632,-0.001124366,2.792485]},
                {"intersection_width": 40},
                {"nominal_speed_in_intersection": 14}
            ],
            emulate_tty=True,
            output='screen'
        )
    )

    for i in range(4):
        nodes.append(
            Node(
                package='carla_intersection',
                executable='vehicle',
                parameters=[
                    {"vehicle_id": i},
                    {"initial_speed": initial_speeds[i]},
                    {"spawn_point": list(positions[i].values())}   
                ],
                emulate_tty=True,
                output='screen'
            )
        )
        nodes.append(
            Node(
                package='carla_intersection',
                executable='carla_sim',
                parameters=[
                    {"vehicle_id": i},
                    {"initial_speed": initial_speeds[i]},
                    {"spawn_point": list(positions[i].values())}   
                ],
                emulate_tty=True,
                output='screen'
            )
        )

    return launch.LaunchDescription(nodes)
