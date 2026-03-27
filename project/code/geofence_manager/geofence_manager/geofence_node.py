import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from wolf_rabbit_msgs.msg import GeofenceStatus


class GeofenceNode(Node):
    def __init__(self):
        super().__init__('geofence_node')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('status_topic', '/geofence/status')
        self.declare_parameter('robot_role', 'rabbit')
        self.declare_parameter('global_min_x', 0.0)
        self.declare_parameter('global_max_x', 10.0)
        self.declare_parameter('global_min_y', 0.0)
        self.declare_parameter('global_max_y', 10.0)
        self.declare_parameter('wolf_min_x', 2.0)
        self.declare_parameter('wolf_max_x', 6.0)
        self.declare_parameter('wolf_min_y', 2.0)
        self.declare_parameter('wolf_max_y', 6.0)
        self.declare_parameter('boundary_margin', 0.5)

        self.role = self.get_parameter('robot_role').value
        self.boundary_margin = float(self.get_parameter('boundary_margin').value)
        self.global_box = [
            float(self.get_parameter('global_min_x').value),
            float(self.get_parameter('global_max_x').value),
            float(self.get_parameter('global_min_y').value),
            float(self.get_parameter('global_max_y').value),
        ]
        self.wolf_box = [
            float(self.get_parameter('wolf_min_x').value),
            float(self.get_parameter('wolf_max_x').value),
            float(self.get_parameter('wolf_min_y').value),
            float(self.get_parameter('wolf_max_y').value),
        ]

        odom_topic = self.get_parameter('odom_topic').value
        status_topic = self.get_parameter('status_topic').value
        self.pub = self.create_publisher(GeofenceStatus, status_topic, 10)
        self.sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

    def odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        status = GeofenceStatus()
        status.inside_global_arena, status.margin_to_global_boundary = self.check_box(x, y, self.global_box)
        inside_wolf, wolf_margin = self.check_box(x, y, self.wolf_box)
        status.inside_wolf_territory = inside_wolf if self.role == 'wolf' else inside_wolf
        status.margin_to_territory_boundary = wolf_margin
        status.near_global_boundary = status.margin_to_global_boundary < self.boundary_margin
        status.near_territory_boundary = wolf_margin < self.boundary_margin
        status.stamp = self.get_clock().now().to_msg()
        self.pub.publish(status)

    @staticmethod
    def check_box(x: float, y: float, box) -> tuple[bool, float]:
        min_x, max_x, min_y, max_y = box
        inside = (min_x <= x <= max_x) and (min_y <= y <= max_y)
        margin = min(x - min_x, max_x - x, y - min_y, max_y - y)
        return inside, float(margin)


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
