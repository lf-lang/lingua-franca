import launch
import launch.actions
import launch.substitutions
from launch_ros.actions import Node


def generate_launch_description():
    spawn_points = [{   \
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

    initial_positions = [
        [0.000038,-0.000674,2.794825],   # /|\ 
        [-0.000501,-0.001084,2.794891],  # -> 
        [-0.000060,-0.001510,2.794854],  # \|/  
        [0.000367,-0.001185,2.794846]    # <-
    ]

    initial_speeds = [
        [ 0.0, -8.0,  0.0], 
        [ 8.0,  0.0,  0.0], 
        [ 0.0,  8.0,  0.0], 
        [-8.0,  0.0,  0.0]
    ]

    nodes = []
    nodes.append(
        Node(
            package='carla_intersection',
            executable='rsu_node',
            parameters=[
                {"intersection_position": [-0.000007632,-0.001124366,2.792485]},
                {"intersection_width": 40},
                {"nominal_speed_in_intersection": 14.0}
            ]
        )
    )

    number_of_vehicles = 4
    assert 1 <= number_of_vehicles <= 4 

    for i in range(number_of_vehicles):
        nodes.append(
            Node(
                package='carla_intersection',
                executable='vehicle_node',
                parameters=[
                    {"vehicle_id": i},
                    {"initial_speed": initial_speeds[i]},
                    {"initial_position": initial_positions[i]}   
                ]
            )
        )
        nodes.append(
            Node(
                package='carla_intersection',
                executable='carla_sim_node',
                parameters=[
                    {"vehicle_id": i},
                    {"initial_speed": initial_speeds[i]},
                    {"spawn_point": [spawn_points[i]["x"], spawn_points[i]["y"], spawn_points[i]["z"], spawn_points[i]["yaw"]]}   
                ]
            )
        )

    return launch.LaunchDescription(nodes)
