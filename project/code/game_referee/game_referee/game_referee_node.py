import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from wolf_rabbit_msgs.msg import GameState, GeofenceStatus


class GameRefereeNode(Node):
    def __init__(self):
        super().__init__('game_referee_node')
        self.declare_parameter('wolf_odom_topic', '/wolf/odom')
        self.declare_parameter('rabbit_odom_topic', '/rabbit/odom')
        self.declare_parameter('rabbit_geofence_topic', '/rabbit/geofence')
        self.declare_parameter('game_state_topic', '/game/state')
        self.declare_parameter('capture_distance', 0.4)

        self.wolf_pose = None
        self.rabbit_pose = None
        self.rabbit_geofence = None
        self.capture_distance = float(self.get_parameter('capture_distance').value)

        self.create_subscription(Odometry, self.get_parameter('wolf_odom_topic').value, self.wolf_cb, 10)
        self.create_subscription(Odometry, self.get_parameter('rabbit_odom_topic').value, self.rabbit_cb, 10)
        self.create_subscription(GeofenceStatus, self.get_parameter('rabbit_geofence_topic').value, self.rabbit_geo_cb, 10)
        self.pub = self.create_publisher(GameState, self.get_parameter('game_state_topic').value, 10)
        self.timer = self.create_timer(0.1, self.tick)

    def wolf_cb(self, msg: Odometry):
        self.wolf_pose = msg.pose.pose.position

    def rabbit_cb(self, msg: Odometry):
        self.rabbit_pose = msg.pose.pose.position

    def rabbit_geo_cb(self, msg: GeofenceStatus):
        self.rabbit_geofence = msg

    def tick(self):
        state = GameState()
        state.game_active = True
        state.rabbit_alive = True
        state.rabbit_escaped = False
        state.wolf_chasing = False
        state.phase = 'PATROL_SEARCH'
        state.wolf_rabbit_distance = -1.0

        if self.wolf_pose and self.rabbit_pose:
            dist = math.hypot(self.wolf_pose.x - self.rabbit_pose.x, self.wolf_pose.y - self.rabbit_pose.y)
            state.wolf_rabbit_distance = float(dist)
            if dist <= self.capture_distance:
                state.phase = 'CAPTURED'
                state.game_active = False
                state.rabbit_alive = False

        if self.rabbit_geofence and not self.rabbit_geofence.inside_wolf_territory and state.rabbit_alive:
            state.rabbit_escaped = True
            state.phase = 'RABBIT_ESCAPED'

        state.stamp = self.get_clock().now().to_msg()
        self.pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = GameRefereeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
