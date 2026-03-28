from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('wolf_rabbit_game')

    yolo_cfg = os.path.join(pkg_share, 'config', 'yolo.yaml')
    rabbit_geo_cfg = os.path.join(pkg_share, 'config', 'rabbit_geofence.yaml')
    wolf_geo_cfg = os.path.join(pkg_share, 'config', 'wolf_geofence.yaml')
    rabbit_cfg = os.path.join(pkg_share, 'config', 'rabbit.yaml')
    wolf_cfg = os.path.join(pkg_share, 'config', 'wolf.yaml')
    referee_cfg = os.path.join(pkg_share, 'config', 'referee.yaml')

    return LaunchDescription([
        
        
        
        Node(
            package='wolf_rabbit_game',
            executable='carrot_manager',
            name='carrot_manager',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'carrot.yaml')],
        ),
        Node(
            package='wolf_rabbit_game',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
            parameters=[yolo_cfg],
        ),
        Node(
            package='wolf_rabbit_game',
            executable='geofence_node',
            name='rabbit_geofence',
            output='screen',
            parameters=[rabbit_geo_cfg],
        ),
        Node(
            package='wolf_rabbit_game',
            executable='geofence_node',
            name='wolf_geofence',
            output='screen',
            parameters=[wolf_geo_cfg],
        ),
        Node(
            package='wolf_rabbit_game',
            executable='rabbit_fsm',
            name='rabbit_fsm',
            output='screen',
            parameters=[rabbit_cfg],
        ),
        Node(
            package='wolf_rabbit_game',
            executable='wolf_fsm',
            name='wolf_fsm',
            output='screen',
            parameters=[wolf_cfg],
        ),
        Node(
            package='wolf_rabbit_game',
            executable='referee_node',
            name='referee_node',
            output='screen',
            parameters=[referee_cfg],
        ),
    ])
