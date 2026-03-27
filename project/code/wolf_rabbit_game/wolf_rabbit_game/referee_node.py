import math
from typing import List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .utils import dict_to_string_msg, string_msg_to_dict


class RefereeNode(Node):
    def __init__(self) -> None:
        super().__init__('referee_node')

        self.declare_parameter('rabbit_odom_topic', '/rabbit/odom')
        self.declare_parameter('wolf_odom_topic', '/wolf/odom')
        self.declare_parameter('rabbit_geofence_topic', '/rabbit/geofence')
        self.declare_parameter('wolf_vision_topic', '/wolf/vision')
        self.declare_parameter('game_state_topic', '/game/state')
        self.declare_parameter('capture_distance_m', 0.4)

        self.rabbit_odom_topic = str(self.get_parameter('rabbit_odom_topic').value)
        self.wolf_odom_topic = str(self.get_parameter('wolf_odom_topic').value)
        self.rabbit_geofence_topic = str(self.get_parameter('rabbit_geofence_topic').value)
        self.wolf_vision_topic = str(self.get_parameter('wolf_vision_topic').value)
        self.game_state_topic = str(self.get_parameter('game_state_topic').value)
        self.capture_distance_m = float(self.get_parameter('capture_distance_m').value)

        self.rabbit_pose: Optional[Tuple[float, float]] = None
        self.wolf_pose: Optional[Tuple[float, float]] = None
        self.rabbit_geofence = {}
        self.wolf_vision = {}
        self.phase = 'ACTIVE'

        self.pub = self.create_publisher(String, self.game_state_topic, 10)
        self.create_subscription(Odometry, self.rabbit_odom_topic, self.rabbit_odom_callback, 10)
        self.create_subscription(Odometry, self.wolf_odom_topic, self.wolf_odom_callback, 10)
        self.create_subscription(String, self.rabbit_geofence_topic, self.rabbit_geofence_callback, 10)
        self.create_subscription(String, self.wolf_vision_topic, self.wolf_vision_callback, 10)
        self.timer = self.create_timer(0.1, self.publish_state)

    def rabbit_odom_callback(self, msg: Odometry) -> None:
        self.rabbit_pose = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))

    def wolf_odom_callback(self, msg: Odometry) -> None:
        self.wolf_pose = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))

    def rabbit_geofence_callback(self, msg: String) -> None:
        self.rabbit_geofence = string_msg_to_dict(msg)

    def wolf_vision_callback(self, msg: String) -> None:
        self.wolf_vision = string_msg_to_dict(msg)

    def publish_state(self) -> None:
        distance = -1.0
        rabbit_alive = True
        rabbit_escaped = False
        wolf_chasing = False
        phase = 'ACTIVE'

        inside_wolf_territory = bool(self.rabbit_geofence.get('inside_wolf_territory', True))
        if not inside_wolf_territory:
            rabbit_escaped = True
            phase = 'ESCAPED'

        if self.rabbit_pose is not None and self.wolf_pose is not None:
            dx = self.rabbit_pose[0] - self.wolf_pose[0]
            dy = self.rabbit_pose[1] - self.wolf_pose[1]
            distance = math.hypot(dx, dy)
            if distance <= self.capture_distance_m:
                rabbit_alive = False
                rabbit_escaped = False
                phase = 'CAPTURED'

        rabbit_visible = bool(self.wolf_vision.get('rabbit_visible', False))
        if rabbit_alive and rabbit_visible and inside_wolf_territory:
            wolf_chasing = True
            phase = 'CHASING'

        self.phase = phase
        payload = {
            'phase': phase,
            'rabbit_alive': rabbit_alive,
            'rabbit_escaped': rabbit_escaped,
            'wolf_chasing': wolf_chasing,
            'distance_m': distance,
        }
        self.pub.publish(dict_to_string_msg(payload))


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RefereeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
