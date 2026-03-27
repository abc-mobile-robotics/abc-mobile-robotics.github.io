import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from wolf_rabbit_msgs.msg import TargetDetection, GeofenceStatus, GameState


class WolfFsmNode(Node):
    def __init__(self):
        super().__init__('wolf_fsm')
        self.declare_parameter('cmd_vel_topic', '/wolf/cmd_vel')
        self.declare_parameter('rabbit_detection_topic', '/detections/rabbit')
        self.declare_parameter('geofence_topic', '/wolf/geofence')
        self.declare_parameter('game_state_topic', '/game/state')
        self.declare_parameter('patrol_linear_speed', 0.12)
        self.declare_parameter('chase_linear_speed', 0.20)

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.create_subscription(TargetDetection, self.get_parameter('rabbit_detection_topic').value, self.rabbit_cb, 10)
        self.create_subscription(GeofenceStatus, self.get_parameter('geofence_topic').value, self.geofence_cb, 10)
        self.create_subscription(GameState, self.get_parameter('game_state_topic').value, self.game_cb, 10)

        self.latest_rabbit = None
        self.latest_geofence = None
        self.game_active = True
        self.rabbit_alive = True
        self.timer = self.create_timer(0.1, self.tick)

    def rabbit_cb(self, msg: TargetDetection):
        self.latest_rabbit = msg

    def geofence_cb(self, msg: GeofenceStatus):
        self.latest_geofence = msg

    def game_cb(self, msg: GameState):
        self.game_active = msg.game_active
        self.rabbit_alive = msg.rabbit_alive

    def tick(self):
        twist = Twist()
        if not self.game_active or not self.rabbit_alive:
            self.cmd_pub.publish(twist)
            return

        rabbit_visible = self.latest_rabbit.visible if self.latest_rabbit else False
        rabbit_in_territory = self.latest_geofence.inside_wolf_territory if self.latest_geofence else True

        if rabbit_visible and rabbit_in_territory:
            twist.linear.x = float(self.get_parameter('chase_linear_speed').value)
            error_x = self.latest_rabbit.center_x - 320.0
            twist.angular.z = -0.002 * error_x
        else:
            twist.linear.x = float(self.get_parameter('patrol_linear_speed').value)
            twist.angular.z = -0.3 if self.latest_geofence and self.latest_geofence.near_territory_boundary else 0.25

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WolfFsmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
