import launch
import launch.actions
import launch.substitutions
from launch_ros.actions import Node
import sys
from os import path
sys.path.insert(0, path.dirname(path.dirname(__file__)))
from src.launch_parameters import SPAWN_POINTS, INITIAL_POSITIONS, INITIAL_VELOCITIES

def generate_launch_description():
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
                    {"initial_velocity": INITIAL_VELOCITIES[i]},
                    {"initial_position": INITIAL_POSITIONS[i]}   
                ]
            )
        )
        nodes.append(
            Node(
                package='carla_intersection',
                executable='carla_sim_node',
                parameters=[
                    {"vehicle_id": i},
                    {"initial_velocity": INITIAL_VELOCITIES[i]},
                    {"spawn_point": [SPAWN_POINTS[i]["x"], SPAWN_POINTS[i]["y"], SPAWN_POINTS[i]["z"], SPAWN_POINTS[i]["yaw"]]}   
                ]
            )
        )

    return launch.LaunchDescription(nodes)
