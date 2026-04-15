from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('turtlebot4_geofence'),
            'config',
            'geofence_params.yaml'
        ]),
        description='Full path to the geofence parameter YAML file.'
    )

    geofence_node = Node(
        package='turtlebot4_geofence',
        executable='geofence_node',
        name='geofence_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        # Remap if your robot uses a namespaced topic, e.g. /robot1/cmd_vel
        # remappings=[('/cmd_vel', '/robot1/cmd_vel')],
    )

    return LaunchDescription([
        params_file_arg,
        geofence_node,
    ])
