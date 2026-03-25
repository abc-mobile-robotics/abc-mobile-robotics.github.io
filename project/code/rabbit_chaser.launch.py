from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='robot04',
            description='Robot topic prefix, e.g. robot04'
        ),
        Node(
            package='rabbit_chaser',
            executable='rabbit_chaser_node',
            name='rabbit_chaser',
            output='screen',
            parameters=[{
                'robot_namespace': LaunchConfiguration('robot_namespace')
            }]
        )
    ])