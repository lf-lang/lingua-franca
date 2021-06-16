import os

import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():
    ld = launch.LaunchDescription([
        ###########
        ############### Configure Vehicle 1
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='8'
        ),
        launch.actions.DeclareLaunchArgument(
            name='vehicle_filter',
            default_value='vehicle.*'
        ),launch.actions.DeclareLaunchArgument(
            name='town',
            default_value='Town05'
        ),
        launch.actions.DeclareLaunchArgument(
            name='passive',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.1'
        ),
        launch_ros.actions.Node(
            package='carla_ros_bridge',
            executable='bridge',
            name='carla_ros_bridge',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'host': launch.substitutions.LaunchConfiguration('host')
                },
                {
                    'port': launch.substitutions.LaunchConfiguration('port')
                },
                {
                    'timeout': launch.substitutions.LaunchConfiguration('timeout')
                },
                {
                    'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds')
                },
                {
                    'town': launch.substitutions.LaunchConfiguration('town')
                },
                {
                    'ego_vehicle_role_name': 'ego_vehicle_1'
                }
            ]
        ),
        launch.actions.DeclareLaunchArgument(
            name='role_name_1',
            default_value='ego_vehicle_1'
        ),
        launch.actions.DeclareLaunchArgument(
            name='objects_definition_file',
            default_value=get_package_share_directory(
                'lf_carla') + '/config/ego_vehicle.json'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_point_ego_vehicle_1',
            default_value='-132.0,39.6,0.3,0,0,-90.0'
        ),
        launch_ros.actions.Node(
            package='carla_spawn_objects',
            executable='carla_spawn_objects',
            name='carla_spawn_objects',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'objects_definition_file': launch.substitutions.LaunchConfiguration('objects_definition_file')
                },
                {
                    'spawn_point_ego_vehicle_1': launch.substitutions.LaunchConfiguration('spawn_point_ego_vehicle_1')
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_manual_control'), 'carla_manual_control.launch.py')
            ),
            launch_arguments={
                'role_name': 'ego_vehicle_1'
            }.items()
        ),
        ############
        #################### Spawn vehicle 2
        launch.actions.DeclareLaunchArgument(
            name='spawn_point_ego_vehicle_2',
            default_value='-167.77,-6.48,0.3,0,0,0.0'
        ),
        launch_ros.actions.Node(
            package='carla_ros_bridge',
            executable='bridge',
            name='carla_ros_bridge',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'host': launch.substitutions.LaunchConfiguration('host')
                },
                {
                    'port': launch.substitutions.LaunchConfiguration('port')
                },
                {
                    'timeout': launch.substitutions.LaunchConfiguration('timeout')
                },
                {
                    'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds')
                },
                {
                    'town': launch.substitutions.LaunchConfiguration('town')
                },
                {
                    'ego_vehicle_role_name': 'ego_vehicle_2'
                }
            ]
        ),
        ###########
        ############### Configure Vehicle 1
        launch.actions.DeclareLaunchArgument(
            name='role_name_2',
            default_value='ego_vehicle_2'
        ),
        launch_ros.actions.Node(
            package='carla_spawn_objects',
            executable='carla_spawn_objects',
            name='carla_spawn_objects',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'objects_definition_file': launch.substitutions.LaunchConfiguration('objects_definition_file')
                },
                {
                    'spawn_point_ego_vehicle_2': launch.substitutions.LaunchConfiguration('spawn_point_ego_vehicle_2')
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_manual_control'), 'carla_manual_control.launch.py')
            ),
            launch_arguments={
                'role_name': 'ego_vehicle_2'
            }.items()
        ),
        Node(
            package='lf_carla',
            executable='lf_carla_publisher'
        ),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
