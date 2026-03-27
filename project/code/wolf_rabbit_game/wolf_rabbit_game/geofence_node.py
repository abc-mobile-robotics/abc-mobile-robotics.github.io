from typing import List

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from .utils import dict_to_string_msg


class GeofenceNode(Node):
    def __init__(self) -> None:
        super().__init__('geofence_node')

        self.declare_parameter('robot_name', 'robot')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('geofence_topic', '/geofence')
        self.declare_parameter('arena_min_x', -3.0)
        self.declare_parameter('arena_max_x', 3.0)
        self.declare_parameter('arena_min_y', -3.0)
        self.declare_parameter('arena_max_y', 3.0)
        self.declare_parameter('territory_min_x', -1.5)
        self.declare_parameter('territory_max_x', 1.5)
        self.declare_parameter('territory_min_y', -1.5)
        self.declare_parameter('territory_max_y', 1.5)
        self.declare_parameter('boundary_margin', 0.3)

        self.robot_name = self.get_parameter('robot_name').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.geofence_topic = self.get_parameter('geofence_topic').value
        self.boundary_margin = float(self.get_parameter('boundary_margin').value)

        self.arena = {
            'min_x': float(self.get_parameter('arena_min_x').value),
            'max_x': float(self.get_parameter('arena_max_x').value),
            'min_y': float(self.get_parameter('arena_min_y').value),
            'max_y': float(self.get_parameter('arena_max_y').value),
        }
        self.territory = {
            'min_x': float(self.get_parameter('territory_min_x').value),
            'max_x': float(self.get_parameter('territory_max_x').value),
            'min_y': float(self.get_parameter('territory_min_y').value),
            'max_y': float(self.get_parameter('territory_max_y').value),
        }

        self.pub = self.create_publisher(type(dict_to_string_msg({})), self.geofence_topic, 10)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

    def inside_box(self, x: float, y: float, box: dict) -> bool:
        return box['min_x'] <= x <= box['max_x'] and box['min_y'] <= y <= box['max_y']

    def boundary_distance(self, x: float, y: float, box: dict) -> float:
        distances = [
            x - box['min_x'],
            box['max_x'] - x,
            y - box['min_y'],
            box['max_y'] - y,
        ]
        return min(distances)

    def odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        in_arena = self.inside_box(x, y, self.arena)
        in_territory = self.inside_box(x, y, self.territory)
        arena_margin = self.boundary_distance(x, y, self.arena) if in_arena else -1.0
        territory_margin = self.boundary_distance(x, y, self.territory) if in_territory else -1.0

        payload = {
            'robot_name': self.robot_name,
            'x': x,
            'y': y,
            'inside_global_arena': in_arena,
            'inside_wolf_territory': in_territory,
            'near_global_boundary': in_arena and arena_margin < self.boundary_margin,
            'near_territory_boundary': in_territory and territory_margin < self.boundary_margin,
            'arena_margin': arena_margin,
            'territory_margin': territory_margin,
        }
        self.pub.publish(dict_to_string_msg(payload))


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GeofenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
