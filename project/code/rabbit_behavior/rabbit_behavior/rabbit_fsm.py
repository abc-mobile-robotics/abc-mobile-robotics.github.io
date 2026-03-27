import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from wolf_rabbit_msgs.msg import TargetDetection, GeofenceStatus, GameState


class RabbitFsmNode(Node):
    def __init__(self):
        super().__init__('rabbit_fsm')
        self.declare_parameter('cmd_vel_topic', '/rabbit/cmd_vel')
        self.declare_parameter('wolf_detection_topic', '/detections/wolf')
        self.declare_parameter('carrot_detection_topic', '/detections/carrot')
        self.declare_parameter('geofence_topic', '/rabbit/geofence')
        self.declare_parameter('game_state_topic', '/game/state')
        self.declare_parameter('search_linear_speed', 0.15)
        self.declare_parameter('flee_linear_speed', 0.22)
        self.declare_parameter('turn_angular_speed', 1.2)
        self.declare_parameter('turn_duration_sec', 2.6)

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.create_subscription(TargetDetection, self.get_parameter('wolf_detection_topic').value, self.wolf_cb, 10)
        self.create_subscription(TargetDetection, self.get_parameter('carrot_detection_topic').value, self.carrot_cb, 10)
        self.create_subscription(GeofenceStatus, self.get_parameter('geofence_topic').value, self.geofence_cb, 10)
        self.create_subscription(GameState, self.get_parameter('game_state_topic').value, self.game_cb, 10)

        self.state = 'SEARCH'
        self.turn_end_time = None
        self.latest_wolf = None
        self.latest_carrot = None
        self.latest_geofence = None
        self.rabbit_alive = True
        self.timer = self.create_timer(0.1, self.tick)

    def wolf_cb(self, msg: TargetDetection):
        self.latest_wolf = msg
        if msg.visible and self.rabbit_alive and self.state != 'FLEE_TURN':
            self.state = 'FLEE_TURN'
            self.turn_end_time = self.get_clock().now().nanoseconds / 1e9 + float(self.get_parameter('turn_duration_sec').value)

    def carrot_cb(self, msg: TargetDetection):
        self.latest_carrot = msg

    def geofence_cb(self, msg: GeofenceStatus):
        self.latest_geofence = msg

    def game_cb(self, msg: GameState):
        self.rabbit_alive = msg.rabbit_alive
        if not msg.rabbit_alive:
            self.state = 'STOPPED'

    def tick(self):
        twist = Twist()
        if not self.rabbit_alive:
            self.cmd_pub.publish(twist)
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if self.state == 'FLEE_TURN':
            twist.angular.z = float(self.get_parameter('turn_angular_speed').value)
            if now >= self.turn_end_time:
                self.state = 'FLEE_RUN'
        elif self.state == 'FLEE_RUN':
            twist.linear.x = float(self.get_parameter('flee_linear_speed').value)
            if self.latest_geofence and not self.latest_geofence.inside_wolf_territory:
                self.state = 'SEARCH'
            elif self.latest_geofence and self.latest_geofence.near_territory_boundary:
                twist.angular.z = 0.5
        elif self.latest_carrot and self.latest_carrot.visible:
            twist.linear.x = float(self.get_parameter('search_linear_speed').value)
            error_x = self.latest_carrot.center_x - 320.0
            twist.angular.z = -0.002 * error_x
        else:
            twist.linear.x = float(self.get_parameter('search_linear_speed').value)
            twist.angular.z = 0.3

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = RabbitFsmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
