from typing import List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from .utils import string_msg_to_dict


class RabbitFSM(Node):
    def __init__(self) -> None:
        super().__init__('rabbit_fsm')

        self.declare_parameter('vision_topic', '/rabbit/vision')
        self.declare_parameter('geofence_topic', '/rabbit/geofence')
        self.declare_parameter('game_state_topic', '/game/state')
        self.declare_parameter('cmd_vel_topic', '/rabbit/cmd_vel')
        self.declare_parameter('search_linear_speed', 0.12)
        self.declare_parameter('turn_angular_speed', 0.8)
        self.declare_parameter('flee_linear_speed', 0.20)
        self.declare_parameter('center_x_px', 320.0)
        self.declare_parameter('carrot_approach_gain', 0.002)
        self.declare_parameter('wolf_avoid_turn_duration_sec', 2.2)

        self.vision_topic = self.get_parameter('vision_topic').value
        self.geofence_topic = self.get_parameter('geofence_topic').value
        self.game_state_topic = self.get_parameter('game_state_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.search_linear_speed = float(self.get_parameter('search_linear_speed').value)
        self.turn_angular_speed = float(self.get_parameter('turn_angular_speed').value)
        self.flee_linear_speed = float(self.get_parameter('flee_linear_speed').value)
        self.center_x_px = float(self.get_parameter('center_x_px').value)
        self.carrot_approach_gain = float(self.get_parameter('carrot_approach_gain').value)
        self.turn_duration = float(self.get_parameter('wolf_avoid_turn_duration_sec').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(String, self.vision_topic, self.vision_callback, 10)
        self.create_subscription(String, self.geofence_topic, self.geofence_callback, 10)
        self.create_subscription(String, self.game_state_topic, self.game_state_callback, 10)
        self.timer = self.create_timer(0.1, self.step)

        self.state = 'SEARCH'
        self.state_enter_time = self.get_clock().now()
        self.vision = {}
        self.geofence = {}
        self.game_state = {}

    def vision_callback(self, msg: String) -> None:
        self.vision = string_msg_to_dict(msg)

    def geofence_callback(self, msg: String) -> None:
        self.geofence = string_msg_to_dict(msg)

    def game_state_callback(self, msg: String) -> None:
        self.game_state = string_msg_to_dict(msg)

    def enter_state(self, new_state: str) -> None:
        if self.state != new_state:
            self.state = new_state
            self.state_enter_time = self.get_clock().now()
            self.get_logger().info(f'Rabbit state -> {new_state}')

    def seconds_in_state(self) -> float:
        return (self.get_clock().now() - self.state_enter_time).nanoseconds / 1e9

    def publish_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def step(self) -> None:
        rabbit_alive = self.game_state.get('rabbit_alive', True)
        wolf_visible = self.vision.get('wolf_visible', False)
        carrot_visible = self.vision.get('carrot_visible', False)
        inside_wolf_territory = self.geofence.get('inside_wolf_territory', False)
        near_global_boundary = self.geofence.get('near_global_boundary', False)

        if not rabbit_alive:
            self.enter_state('DEAD')

        if self.state == 'DEAD':
            self.publish_cmd(0.0, 0.0)
            return

        if wolf_visible and self.state not in ('FLEE_TURN', 'FLEE_RUN'):
            self.enter_state('FLEE_TURN')

        if self.state == 'SEARCH':
            if near_global_boundary:
                self.publish_cmd(0.0, self.turn_angular_speed)
                return

            if carrot_visible:
                error = self.vision.get('carrot_center_x', self.center_x_px) - self.center_x_px
                self.publish_cmd(self.search_linear_speed, -self.carrot_approach_gain * error)
            else:
                self.publish_cmd(self.search_linear_speed, 0.25)

        elif self.state == 'FLEE_TURN':
            self.publish_cmd(0.0, self.turn_angular_speed)
            if self.seconds_in_state() >= self.turn_duration:
                self.enter_state('FLEE_RUN')

        elif self.state == 'FLEE_RUN':
            if near_global_boundary:
                self.publish_cmd(0.0, self.turn_angular_speed)
                return

            self.publish_cmd(self.flee_linear_speed, 0.0)

            if not inside_wolf_territory:
                self.enter_state('SEARCH')


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RabbitFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
