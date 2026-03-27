from typing import List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from .utils import string_msg_to_dict


class WolfFSM(Node):
    def __init__(self) -> None:
        super().__init__('wolf_fsm')

        self.declare_parameter('vision_topic', '/wolf/vision')
        self.declare_parameter('wolf_geofence_topic', '/wolf/geofence')
        self.declare_parameter('rabbit_geofence_topic', '/rabbit/geofence')
        self.declare_parameter('game_state_topic', '/game/state')
        self.declare_parameter('cmd_vel_topic', '/wolf/cmd_vel')
        self.declare_parameter('patrol_linear_speed', 0.10)
        self.declare_parameter('patrol_turn_speed', 0.60)
        self.declare_parameter('chase_linear_speed', 0.18)
        self.declare_parameter('chase_turn_gain', 0.002)
        self.declare_parameter('center_x_px', 320.0)

        self.vision_topic = self.get_parameter('vision_topic').value
        self.wolf_geofence_topic = self.get_parameter('wolf_geofence_topic').value
        self.rabbit_geofence_topic = self.get_parameter('rabbit_geofence_topic').value
        self.game_state_topic = self.get_parameter('game_state_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.patrol_linear_speed = float(self.get_parameter('patrol_linear_speed').value)
        self.patrol_turn_speed = float(self.get_parameter('patrol_turn_speed').value)
        self.chase_linear_speed = float(self.get_parameter('chase_linear_speed').value)
        self.chase_turn_gain = float(self.get_parameter('chase_turn_gain').value)
        self.center_x_px = float(self.get_parameter('center_x_px').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(String, self.vision_topic, self.vision_callback, 10)
        self.create_subscription(String, self.wolf_geofence_topic, self.wolf_geofence_callback, 10)
        self.create_subscription(String, self.rabbit_geofence_topic, self.rabbit_geofence_callback, 10)
        self.create_subscription(String, self.game_state_topic, self.game_state_callback, 10)
        self.timer = self.create_timer(0.1, self.step)

        self.state = 'PATROL'
        self.vision = {}
        self.wolf_geofence = {}
        self.rabbit_geofence = {}
        self.game_state = {}

    def vision_callback(self, msg: String) -> None:
        self.vision = string_msg_to_dict(msg)

    def wolf_geofence_callback(self, msg: String) -> None:
        self.wolf_geofence = string_msg_to_dict(msg)

    def rabbit_geofence_callback(self, msg: String) -> None:
        self.rabbit_geofence = string_msg_to_dict(msg)

    def game_state_callback(self, msg: String) -> None:
        self.game_state = string_msg_to_dict(msg)

    def enter_state(self, new_state: str) -> None:
        if self.state != new_state:
            self.state = new_state
            self.get_logger().info(f'Wolf state -> {new_state}')

    def publish_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def step(self) -> None:
        rabbit_alive = self.game_state.get('rabbit_alive', True)
        rabbit_visible = self.vision.get('rabbit_visible', False)
        rabbit_inside_territory = self.rabbit_geofence.get('inside_wolf_territory', True)
        near_territory_boundary = self.wolf_geofence.get('near_territory_boundary', False)

        if not rabbit_alive:
            self.enter_state('STOP')

        if self.state == 'STOP':
            self.publish_cmd(0.0, 0.0)
            return

        if rabbit_visible and rabbit_inside_territory:
            self.enter_state('CHASE')
        elif self.state == 'CHASE' and not rabbit_inside_territory:
            self.enter_state('RETURN')

        if self.state == 'PATROL':
            if near_territory_boundary:
                self.publish_cmd(0.0, self.patrol_turn_speed)
            else:
                self.publish_cmd(self.patrol_linear_speed, 0.2)

        elif self.state == 'CHASE':
            if not rabbit_inside_territory:
                self.enter_state('RETURN')
                self.publish_cmd(0.0, 0.0)
                return

            error = self.vision.get('rabbit_center_x', self.center_x_px) - self.center_x_px
            self.publish_cmd(self.chase_linear_speed, -self.chase_turn_gain * error)

        elif self.state == 'RETURN':
            if near_territory_boundary:
                self.publish_cmd(0.0, self.patrol_turn_speed)
            else:
                self.enter_state('PATROL')
                self.publish_cmd(self.patrol_linear_speed, 0.0)


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WolfFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
