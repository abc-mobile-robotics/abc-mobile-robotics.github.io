from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    yolo_cfg = os.path.join(get_package_share_directory('yolo_perception'), 'config', 'yolo_params.yaml')
    rabbit_geo_cfg = os.path.join(get_package_share_directory('geofence_manager'), 'config', 'rabbit_geofence.yaml')
    wolf_geo_cfg = os.path.join(get_package_share_directory('geofence_manager'), 'config', 'wolf_geofence.yaml')
    rabbit_cfg = os.path.join(get_package_share_directory('rabbit_behavior'), 'config', 'rabbit_params.yaml')
    wolf_cfg = os.path.join(get_package_share_directory('wolf_behavior'), 'config', 'wolf_params.yaml')
    referee_cfg = os.path.join(get_package_share_directory('game_referee'), 'config', 'referee_params.yaml')

    return LaunchDescription([
        Node(package='yolo_perception', executable='yolo_detector', name='yolo_detector', parameters=[yolo_cfg]),
        Node(package='geofence_manager', executable='geofence_node', name='rabbit_geofence', parameters=[rabbit_geo_cfg]),
        Node(package='geofence_manager', executable='geofence_node', name='wolf_geofence', parameters=[wolf_geo_cfg]),
        Node(package='rabbit_behavior', executable='rabbit_fsm', name='rabbit_fsm', parameters=[rabbit_cfg]),
        Node(package='wolf_behavior', executable='wolf_fsm', name='wolf_fsm', parameters=[wolf_cfg]),
        Node(package='game_referee', executable='game_referee_node', name='game_referee_node', parameters=[referee_cfg]),
    ])
