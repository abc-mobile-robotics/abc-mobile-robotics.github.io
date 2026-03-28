from typing import List

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .utils import clamp, is_stale, string_msg_to_dict, wrap_to_pi, yaw_from_quaternion


class WolfFSM(Node):
    def __init__(self) -> None:
        super().__init__('wolf_fsm')

        self.declare_parameter('vision_topic', '/wolf/vision')
        self.declare_parameter('wolf_geofence_topic', '/wolf/geofence')
        self.declare_parameter('rabbit_geofence_topic', '/rabbit/geofence')
        self.declare_parameter('game_state_topic', '/game/state')
        self.declare_parameter('odom_topic', '/wolf/odom')
        self.declare_parameter('cmd_vel_topic', '/wolf/cmd_vel')
        self.declare_parameter('patrol_linear_speed', 0.10)
        self.declare_parameter('patrol_turn_speed', 0.65)
        self.declare_parameter('chase_linear_speed', 0.20)
        self.declare_parameter('chase_turn_gain', 0.003)
        self.declare_parameter('return_turn_speed', 0.75)
        self.declare_parameter('vision_timeout_sec', 0.7)
        self.declare_parameter('center_x_px', 320.0)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('return_turn_angle_deg', 160.0)

        self.vision_topic = str(self.get_parameter('vision_topic').value)
        self.wolf_geofence_topic = str(self.get_parameter('wolf_geofence_topic').value)
        self.rabbit_geofence_topic = str(self.get_parameter('rabbit_geofence_topic').value)
        self.game_state_topic = str(self.get_parameter('game_state_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.patrol_linear_speed = float(self.get_parameter('patrol_linear_speed').value)
        self.patrol_turn_speed = float(self.get_parameter('patrol_turn_speed').value)
        self.chase_linear_speed = float(self.get_parameter('chase_linear_speed').value)
        self.chase_turn_gain = float(self.get_parameter('chase_turn_gain').value)
        self.return_turn_speed = float(self.get_parameter('return_turn_speed').value)
        self.vision_timeout_sec = float(self.get_parameter('vision_timeout_sec').value)
        self.center_x_px = float(self.get_parameter('center_x_px').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.return_turn_angle = float(self.get_parameter('return_turn_angle_deg').value) * 3.141592653589793 / 180.0

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(String, self.vision_topic, self.vision_callback, 10)
        self.create_subscription(String, self.wolf_geofence_topic, self.wolf_geofence_callback, 10)
        self.create_subscription(String, self.rabbit_geofence_topic, self.rabbit_geofence_callback, 10)
        self.create_subscription(String, self.game_state_topic, self.game_state_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.step)

        self.state = 'PATROL'
        self.state_enter_time = self.get_clock().now()
        self.vision = {}
        self.wolf_geofence = {}
        self.rabbit_geofence = {}
        self.game_state = {}
        self.current_yaw = 0.0
        self.have_odom = False
        self.return_target_yaw = None

    def vision_callback(self, msg: String) -> None:
        self.vision = string_msg_to_dict(msg)

    def wolf_geofence_callback(self, msg: String) -> None:
        self.wolf_geofence = string_msg_to_dict(msg)

    def rabbit_geofence_callback(self, msg: String) -> None:
        self.rabbit_geofence = string_msg_to_dict(msg)

    def game_state_callback(self, msg: String) -> None:
        self.game_state = string_msg_to_dict(msg)

    def odom_callback(self, msg: Odometry) -> None:
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_odom = True

    def enter_state(self, new_state: str) -> None:
        if self.state != new_state:
            self.state = new_state
            self.state_enter_time = self.get_clock().now()
            if new_state == 'RETURN_TURN' and self.have_odom:
                self.return_target_yaw = wrap_to_pi(self.current_yaw + self.return_turn_angle)
            elif new_state != 'RETURN_TURN':
                self.return_target_yaw = None
            self.get_logger().info(f'Wolf state -> {new_state}')

    def publish_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def vision_fresh(self) -> bool:
        return not is_stale(self.vision.get('stamp'), self.vision_timeout_sec)

    def rabbit_visible(self) -> bool:
        return self.vision_fresh() and bool(self.vision.get('rabbit_visible', False))

    def seconds_in_state(self) -> float:
        return (self.get_clock().now() - self.state_enter_time).nanoseconds / 1e9

    def patrol_step(self) -> None:
        near_territory_boundary = bool(self.wolf_geofence.get('near_territory_boundary', False))
        outside_territory = bool(self.wolf_geofence.get('outside_wolf_territory', False))
        if outside_territory or near_territory_boundary:
            self.publish_cmd(0.0, self.patrol_turn_speed)
        else:
            self.publish_cmd(self.patrol_linear_speed, 0.18)

    def step(self) -> None:
        rabbit_alive = bool(self.game_state.get('rabbit_alive', True))
        rabbit_escaped = bool(self.game_state.get('rabbit_escaped', False))
        rabbit_inside_territory = bool(self.rabbit_geofence.get('inside_wolf_territory', True))

        if not rabbit_alive:
            self.enter_state('STOP')

        if self.state == 'STOP':
            self.publish_cmd(0.0, 0.0)
            return

        if self.state == 'PATROL' and self.rabbit_visible() and rabbit_inside_territory and not rabbit_escaped:
            self.enter_state('CHASE')

        if self.state == 'CHASE' and (rabbit_escaped or not rabbit_inside_territory):
            self.enter_state('RETURN_TURN')

        if self.state == 'PATROL':
            self.patrol_step()
            return

        if self.state == 'CHASE':
            if not self.rabbit_visible():
                self.enter_state('PATROL')
                self.patrol_step()
                return
            error = float(self.vision.get('rabbit_center_x', self.center_x_px)) - self.center_x_px
            omega = clamp(-self.chase_turn_gain * error, -self.max_angular_speed, self.max_angular_speed)
            self.publish_cmd(self.chase_linear_speed, omega)
            return

        if self.state == 'RETURN_TURN':
            if self.have_odom and self.return_target_yaw is not None:
                yaw_error = wrap_to_pi(self.return_target_yaw - self.current_yaw)
                if abs(yaw_error) < 0.15:
                    self.enter_state('PATROL')
                    self.patrol_step()
                    return
                omega = clamp(1.6 * yaw_error, -self.return_turn_speed, self.return_turn_speed)
                self.publish_cmd(0.0, omega)
            else:
                self.publish_cmd(0.0, self.return_turn_speed)
                if self.seconds_in_state() > 1.8:
                    self.enter_state('PATROL')
            return


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WolfFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
