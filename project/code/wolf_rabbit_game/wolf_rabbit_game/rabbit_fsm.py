from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .utils import clamp, is_stale, string_msg_to_dict, wrap_to_pi, yaw_from_quaternion


class RabbitFSM(Node):
    def __init__(self) -> None:
        super().__init__('rabbit_fsm')

        self.declare_parameter('vision_topic', '/rabbit/vision')
        self.declare_parameter('carrot_state_topic', '/game/carrot_state')
        self.declare_parameter('geofence_topic', '/rabbit/geofence')
        self.declare_parameter('game_state_topic', '/game/state')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('search_linear_speed', 0.12)
        self.declare_parameter('search_spin_speed', 0.25)
        self.declare_parameter('turn_angular_speed', 0.9)
        self.declare_parameter('flee_linear_speed', 0.22)
        self.declare_parameter('boundary_turn_speed', 0.8)
        self.declare_parameter('center_x_px', 320.0)
        self.declare_parameter('carrot_reach_stop_distance', 0.20)
        self.declare_parameter('carrot_heading_gain', 1.2)
        self.declare_parameter('wolf_avoid_gain', 0.004)
        self.declare_parameter('vision_timeout_sec', 0.7)
        self.declare_parameter('escape_turn_angle_deg', 180.0)
        self.declare_parameter('escape_turn_tolerance_deg', 7.0)
        self.declare_parameter('escape_settle_time_sec', 0.4)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('control_period_sec', 0.1)

        self.vision_topic = str(self.get_parameter('vision_topic').value)
        self.carrot_state_topic = str(self.get_parameter('carrot_state_topic').value)
        self.geofence_topic = str(self.get_parameter('geofence_topic').value)
        self.game_state_topic = str(self.get_parameter('game_state_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.search_linear_speed = float(self.get_parameter('search_linear_speed').value)
        self.search_spin_speed = float(self.get_parameter('search_spin_speed').value)
        self.turn_angular_speed = float(self.get_parameter('turn_angular_speed').value)
        self.flee_linear_speed = float(self.get_parameter('flee_linear_speed').value)
        self.boundary_turn_speed = float(self.get_parameter('boundary_turn_speed').value)
        self.center_x_px = float(self.get_parameter('center_x_px').value)
        self.carrot_reach_stop_distance = float(self.get_parameter('carrot_reach_stop_distance').value)
        self.carrot_heading_gain = float(self.get_parameter('carrot_heading_gain').value)
        self.wolf_avoid_gain = float(self.get_parameter('wolf_avoid_gain').value)
        self.vision_timeout_sec = float(self.get_parameter('vision_timeout_sec').value)
        self.escape_turn_angle = float(self.get_parameter('escape_turn_angle_deg').value) * 3.141592653589793 / 180.0
        self.escape_turn_tolerance = float(self.get_parameter('escape_turn_tolerance_deg').value) * 3.141592653589793 / 180.0
        self.escape_settle_time_sec = float(self.get_parameter('escape_settle_time_sec').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.control_period_sec = float(self.get_parameter('control_period_sec').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(String, self.vision_topic, self.vision_callback, 10)
        self.create_subscription(String, self.carrot_state_topic, self.carrot_callback, 10)
        self.create_subscription(String, self.geofence_topic, self.geofence_callback, 10)
        self.create_subscription(String, self.game_state_topic, self.game_state_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.timer = self.create_timer(self.control_period_sec, self.step)

        self.state = 'SEARCH'
        self.state_enter_time = self.get_clock().now()

        self.vision = {}
        self.carrot_state = {}
        self.geofence = {}
        self.game_state = {}

        self.rabbit_x: Optional[float] = None
        self.rabbit_y: Optional[float] = None
        self.current_yaw: float = 0.0
        self.have_odom = False
        self.turn_target_yaw: Optional[float] = None

    def vision_callback(self, msg: String) -> None:
        self.vision = string_msg_to_dict(msg)

    def carrot_callback(self, msg: String) -> None:
        self.carrot_state = string_msg_to_dict(msg)

    def geofence_callback(self, msg: String) -> None:
        self.geofence = string_msg_to_dict(msg)

    def game_state_callback(self, msg: String) -> None:
        self.game_state = string_msg_to_dict(msg)

    def odom_callback(self, msg: Odometry) -> None:
        self.rabbit_x = float(msg.pose.pose.position.x)
        self.rabbit_y = float(msg.pose.pose.position.y)
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_odom = True

    def enter_state(self, new_state: str) -> None:
        if self.state == new_state:
            return

        self.state = new_state
        self.state_enter_time = self.get_clock().now()

        if new_state == 'ESCAPE_TURN' and self.have_odom:
            self.turn_target_yaw = wrap_to_pi(self.current_yaw + self.escape_turn_angle)
        else:
            self.turn_target_yaw = None

        self.get_logger().info(f'Rabbit state -> {new_state}')

    def seconds_in_state(self) -> float:
        return (self.get_clock().now() - self.state_enter_time).nanoseconds / 1e9

    def publish_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def vision_fresh(self) -> bool:
        return not is_stale(self.vision.get('stamp'), self.vision_timeout_sec)

    def wolf_visible(self) -> bool:
        return self.vision_fresh() and bool(self.vision.get('wolf_visible', False))

    def carrot_active(self) -> bool:
        return bool(self.carrot_state.get('active', False))

    def carrot_position(self) -> tuple[Optional[float], Optional[float]]:
        return self.carrot_state.get('x', None), self.carrot_state.get('y', None)

    def distance_to_carrot(self) -> Optional[float]:
        carrot_x, carrot_y = self.carrot_position()
        if self.rabbit_x is None or self.rabbit_y is None:
            return None
        if carrot_x is None or carrot_y is None:
            return None
        dx = float(carrot_x) - self.rabbit_x
        dy = float(carrot_y) - self.rabbit_y
        return (dx * dx + dy * dy) ** 0.5

    def boundary_recovery(self) -> bool:
        outside_global = bool(self.geofence.get('outside_global_arena', False))
        near_global = bool(self.geofence.get('near_global_boundary', False))
        if outside_global or near_global:
            self.publish_cmd(0.0, self.boundary_turn_speed)
            return True
        return False

    def drive_to_xy(self, target_x: float, target_y: float, speed: Optional[float] = None) -> None:
        if not self.have_odom or self.rabbit_x is None or self.rabbit_y is None:
            self.publish_cmd(0.0, 0.0)
            return

        if speed is None:
            speed = self.search_linear_speed

        dx = float(target_x) - self.rabbit_x
        dy = float(target_y) - self.rabbit_y
        target_yaw = __import__('math').atan2(dy, dx)
        yaw_error = wrap_to_pi(target_yaw - self.current_yaw)

        omega = clamp(self.carrot_heading_gain * yaw_error, -self.max_angular_speed, self.max_angular_speed)
        linear = 0.0 if abs(yaw_error) > 0.45 else float(speed)
        self.publish_cmd(linear, omega)

    def step(self) -> None:
        rabbit_alive = bool(self.game_state.get('rabbit_alive', True))
        rabbit_escaped = bool(self.game_state.get('rabbit_escaped', False))
        inside_wolf_territory = bool(self.geofence.get('inside_wolf_territory', False))
        phase = str(self.game_state.get('phase', 'ACTIVE'))

        if (not rabbit_alive) or phase == 'CAPTURED':
            self.enter_state('DEAD')

        if self.state == 'DEAD':
            self.publish_cmd(0.0, 0.0)
            return

        if self.wolf_visible() and self.state not in ('ESCAPE_TURN', 'ESCAPE_RUN'):
            self.enter_state('ESCAPE_TURN')

        if self.state == 'SEARCH':
            if self.boundary_recovery():
                return

            if self.wolf_visible():
                self.enter_state('ESCAPE_TURN')
                return

            if self.carrot_active():
                carrot_x, carrot_y = self.carrot_position()
                if carrot_x is not None and carrot_y is not None:
                    self.enter_state('APPROACH_CARROT')
                    return

            self.publish_cmd(self.search_linear_speed, self.search_spin_speed)

        elif self.state == 'APPROACH_CARROT':
            if self.boundary_recovery():
                return

            if self.wolf_visible():
                self.enter_state('ESCAPE_TURN')
                return

            if not self.carrot_active():
                self.enter_state('SEARCH')
                return

            dist = self.distance_to_carrot()
            if dist is not None and dist <= self.carrot_reach_stop_distance:
                self.publish_cmd(0.0, 0.0)
                self.enter_state('SEARCH')
                return

            carrot_x, carrot_y = self.carrot_position()
            if carrot_x is None or carrot_y is None:
                self.enter_state('SEARCH')
                return

            self.drive_to_xy(float(carrot_x), float(carrot_y), self.search_linear_speed)

        elif self.state == 'ESCAPE_TURN':
            if self.have_odom and self.turn_target_yaw is not None:
                yaw_error = wrap_to_pi(self.turn_target_yaw - self.current_yaw)
                if abs(yaw_error) < self.escape_turn_tolerance:
                    self.enter_state('ESCAPE_RUN')
                    return
                omega = clamp(1.6 * yaw_error, -self.turn_angular_speed, self.turn_angular_speed)
                self.publish_cmd(0.0, omega)
            else:
                self.publish_cmd(0.0, self.turn_angular_speed)
                if self.seconds_in_state() >= 2.1:
                    self.enter_state('ESCAPE_RUN')

        elif self.state == 'ESCAPE_RUN':
            if self.boundary_recovery():
                return

            omega = 0.0
            if self.wolf_visible():
                wolf_error = float(self.vision.get('wolf_center_x', self.center_x_px)) - self.center_x_px
                omega = clamp(self.wolf_avoid_gain * wolf_error, -self.max_angular_speed, self.max_angular_speed)

            self.publish_cmd(self.flee_linear_speed, omega)

            if rabbit_escaped and (not inside_wolf_territory) and self.seconds_in_state() >= self.escape_settle_time_sec:
                self.enter_state('SEARCH')


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RabbitFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
