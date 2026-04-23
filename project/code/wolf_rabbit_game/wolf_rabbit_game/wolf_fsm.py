from typing import List

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult

from .utils import clamp, is_stale, string_msg_to_dict, wrap_to_pi, yaw_from_quaternion


def _bool(val, default: bool = False) -> bool:
    """Safe bool parser for values coming from string_msg_to_dict.
    Direct bool() on a string like 'False' returns True (non-empty string).
    """
    if isinstance(val, bool):
        return val
    if isinstance(val, str):
        return val.strip().lower() == 'true'
    if val is None:
        return default
    return bool(val)


class PID:
    """Simple PID controller for a single axis."""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_limit: float = float('inf'),
                 integral_limit: float = float('inf')) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit   = output_limit
        self.integral_limit = integral_limit

        self._integral   = 0.0
        self._prev_error = 0.0
        self._first_tick = True

    def reset(self) -> None:
        self._integral   = 0.0
        self._prev_error = 0.0
        self._first_tick = True

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        # Derivative — skip on first tick to avoid huge spike
        if self._first_tick:
            derivative = 0.0
            self._first_tick = False
        else:
            derivative = (error - self._prev_error) / dt

        # Integral with anti-windup clamp
        self._integral += error * dt
        self._integral = clamp(self._integral, -self.integral_limit, self.integral_limit)

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        self._prev_error = error

        return clamp(output, -self.output_limit, self.output_limit)


class WolfFSM(Node):
    def __init__(self) -> None:
        super().__init__('wolf_fsm')

        self.declare_parameter('vision_topic', '/wolf/vision')
        self.declare_parameter('wolf_geofence_topic', '/wolf/geofence')
        self.declare_parameter('rabbit_geofence_topic', '/rabbit/geofence')
        self.declare_parameter('game_state_topic', '/game/state')
        self.declare_parameter('odom_topic', '/wolf/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_unstamped')
        self.declare_parameter('patrol_linear_speed', 0.10)
        self.declare_parameter('patrol_turn_speed', 0.65)
        self.declare_parameter('chase_linear_speed', 1.5)
        self.declare_parameter('return_turn_speed', 0.75)
        self.declare_parameter('vision_timeout_sec', 0.7)
        self.declare_parameter('center_x_px', 125.0)
        self.declare_parameter('max_angular_speed', 0.6)
        self.declare_parameter('return_turn_angle_deg', 160.0)
        # Chase PID gains — error is now normalized to [-1, 1],
        # so gains are in units of (rad/s) per unit-of-frame-width.
        # A kp of 2.0 means: rabbit at the edge of the frame → 2 rad/s turn.
        self.declare_parameter('chase_kp', 2.0)
        self.declare_parameter('chase_ki', 0.0)
        self.declare_parameter('chase_kd', 0.15)
        # Deadband in normalized units (0.02 ≈ 2% of frame width ≈ ~2–5 px)
        self.declare_parameter('chase_deadband', 0.02)
        # Slow down forward speed proportionally when turning hard
        self.declare_parameter('chase_speed_scale_on_turn', True)
        # Frames of missed detections before dropping out of CHASE (~0.5 s at 10 Hz)
        self.declare_parameter('chase_lost_frames', 5)
        self.add_on_set_parameters_callback(self._on_param_change)

        self.vision_topic          = str(self.get_parameter('vision_topic').value)
        self.wolf_geofence_topic   = str(self.get_parameter('wolf_geofence_topic').value)
        self.rabbit_geofence_topic = str(self.get_parameter('rabbit_geofence_topic').value)
        self.game_state_topic      = str(self.get_parameter('game_state_topic').value)
        self.odom_topic            = str(self.get_parameter('odom_topic').value)
        self.cmd_vel_topic         = str(self.get_parameter('cmd_vel_topic').value)
        self.patrol_linear_speed   = float(self.get_parameter('patrol_linear_speed').value)
        self.patrol_turn_speed     = float(self.get_parameter('patrol_turn_speed').value)
        self.chase_linear_speed    = float(self.get_parameter('chase_linear_speed').value)
        self.return_turn_speed     = float(self.get_parameter('return_turn_speed').value)
        self.vision_timeout_sec    = float(self.get_parameter('vision_timeout_sec').value)
        self.center_x_px           = float(self.get_parameter('center_x_px').value)
        self.max_angular_speed     = float(self.get_parameter('max_angular_speed').value)
        self.return_turn_angle     = float(self.get_parameter('return_turn_angle_deg').value) * 3.141592653589793 / 180.0
        self.chase_deadband        = float(self.get_parameter('chase_deadband').value)
        self.chase_speed_scale     = bool(self.get_parameter('chase_speed_scale_on_turn').value)
        self.chase_lost_frames     = int(self.get_parameter('chase_lost_frames').value)

        # PID for centering the rabbit horizontally in frame.
        # Integral limit is in (normalized-error · seconds).
        self.chase_pid = PID(
            kp=float(self.get_parameter('chase_kp').value),
            ki=float(self.get_parameter('chase_ki').value),
            kd=float(self.get_parameter('chase_kd').value),
            output_limit=self.max_angular_speed,
            integral_limit=1.0,
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(String,   self.vision_topic,          self.vision_callback,          10)
        self.create_subscription(String,   self.wolf_geofence_topic,   self.wolf_geofence_callback,   10)
        self.create_subscription(String,   self.rabbit_geofence_topic, self.rabbit_geofence_callback, 10)
        self.create_subscription(String,   self.game_state_topic,      self.game_state_callback,      10)
        self.create_subscription(Odometry, self.odom_topic,            self.odom_callback,            10)
        self.timer = self.create_timer(0.1, self.step)

        self.state             = 'PATROL'
        self.state_enter_time  = self.get_clock().now()
        self.vision            = {}
        self.wolf_geofence     = {}
        self.rabbit_geofence   = {}
        self.game_state        = {}
        self.current_yaw       = 0.0
        self.have_odom         = False
        self.return_target_yaw = None
        self._last_step_time   = self.get_clock().now()
        self._lost_frame_count = 0

        # Track which vision message we've already consumed so we only
        # run the PID on FRESH frames. Holds the last omega/linear between frames.
        self._last_vision_stamp: float | None = None
        self._last_pid_time     = self.get_clock().now()
        self._last_omega        = 0.0
        self._last_linear       = 0.0

    # ── callbacks ─────────────────────────────────────────────────────────────

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

    def _on_param_change(self, params):
        for p in params:
            name = p.name
            val = p.value
            if name == 'chase_linear_speed':
                self.chase_linear_speed = float(val)
            elif name == 'patrol_linear_speed':
                self.patrol_linear_speed = float(val)
            elif name == 'patrol_turn_speed':
                self.patrol_turn_speed = float(val)
            elif name == 'return_turn_speed':
                self.return_turn_speed = float(val)
            elif name == 'max_angular_speed':
                self.max_angular_speed = float(val)
                self.chase_pid.output_limit = float(val)
            elif name == 'chase_kp':
                self.chase_pid.kp = float(val)
            elif name == 'chase_ki':
                self.chase_pid.ki = float(val)
            elif name == 'chase_kd':
                self.chase_pid.kd = float(val)
            elif name == 'chase_deadband':
                self.chase_deadband = float(val)
            elif name == 'chase_speed_scale_on_turn':
                self.chase_speed_scale = bool(val)
            elif name == 'chase_lost_frames':
                self.chase_lost_frames = int(val)
            elif name == 'vision_timeout_sec':
                self.vision_timeout_sec = float(val)
            elif name == 'center_x_px':
                self.center_x_px = float(val)
            self.get_logger().info(f'Param updated: {name} = {val}')
        return SetParametersResult(successful=True)

    # ── helpers ───────────────────────────────────────────────────────────────

    def enter_state(self, new_state: str) -> None:
        if self.state != new_state:
            self.state = new_state
            self.state_enter_time = self.get_clock().now()
            if new_state == 'RETURN_TURN' and self.have_odom:
                self.return_target_yaw = wrap_to_pi(self.current_yaw + self.return_turn_angle)
            elif new_state != 'RETURN_TURN':
                self.return_target_yaw = None
            if new_state == 'CHASE':
                self.chase_pid.reset()
                self._lost_frame_count = 0
                self._last_vision_stamp = None
                self._last_pid_time     = self.get_clock().now()
                self._last_omega        = 0.0
                self._last_linear       = 0.0
            self.get_logger().info(f'Wolf state -> {new_state}')

    def publish_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x  = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def vision_fresh(self) -> bool:
        return not is_stale(self.vision.get('stamp'), self.vision_timeout_sec)

    def rabbit_visible(self) -> bool:
        return self.vision_fresh() and _bool(self.vision.get('rabbit_visible', False))

    def seconds_in_state(self) -> float:
        return (self.get_clock().now() - self.state_enter_time).nanoseconds / 1e9

    def patrol_step(self) -> None:
        near_territory_boundary = _bool(self.wolf_geofence.get('near_territory_boundary', False))
        outside_territory       = _bool(self.wolf_geofence.get('outside_wolf_territory',  False))
        if outside_territory or near_territory_boundary:
            self.publish_cmd(0.0, self.patrol_turn_speed)
        else:
            self.publish_cmd(self.patrol_linear_speed, 0.18)

    def _normalized_error(self) -> float:
        """Horizontal error in normalized frame coords: -1 (left edge) .. +1 (right edge).

        Uses image_width from the vision dict if available, otherwise falls back
        to 2 * center_x_px (assumes center_x_px is the image midpoint).
        """
        cx = float(self.vision.get('rabbit_center_x', self.center_x_px))
        img_w = self.vision.get('image_width')
        if img_w is not None:
            try:
                half_w = float(img_w) / 2.0
            except (TypeError, ValueError):
                half_w = self.center_x_px
        else:
            half_w = self.center_x_px
        if half_w <= 0.0:
            return 0.0
        return clamp((cx - self.center_x_px) / half_w, -1.5, 1.5)

    # ── main loop ─────────────────────────────────────────────────────────────

    def step(self) -> None:
        now = self.get_clock().now()
        dt  = (now - self._last_step_time).nanoseconds / 1e9
        self._last_step_time = now

        rabbit_alive            = _bool(self.game_state.get('rabbit_alive', True))
        rabbit_escaped          = _bool(self.game_state.get('rabbit_escaped', False))
        rabbit_inside_territory = _bool(self.rabbit_geofence.get('inside_wolf_territory', True))

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
                self._lost_frame_count += 1
                if self._lost_frame_count >= self.chase_lost_frames:
                    self.get_logger().info(
                        f'[CHASE] Rabbit lost for {self._lost_frame_count} frames — back to PATROL')
                    self.enter_state('PATROL')
                    self.patrol_step()
                else:
                    # Brief dropout — keep turning the way we were, creep forward.
                    self.publish_cmd(self._last_linear * 0.4, self._last_omega)
                return

            # Rabbit is visible. Only recompute PID on a FRESH vision frame;
            # between frames, just republish the last command so we don't
            # chew on stale error with a stretched dt.
            vision_stamp = self.vision.get('stamp')
            new_frame = (vision_stamp is not None
                         and vision_stamp != self._last_vision_stamp)

            if new_frame:
                pid_now = self.get_clock().now()
                pid_dt  = (pid_now - self._last_pid_time).nanoseconds / 1e9
                self._last_pid_time     = pid_now
                self._last_vision_stamp = vision_stamp
                self._lost_frame_count  = 0

                # Error in normalized frame coords: left = negative, right = positive.
                # We want positive error → turn right → negative omega.
                error = self._normalized_error()
                if abs(error) < self.chase_deadband:
                    error = 0.0
                omega = -self.chase_pid.compute(error, pid_dt)

                # Scale forward speed down when turning hard so the robot arcs
                # smoothly rather than swinging wide and losing the target.
                if self.chase_speed_scale and self.max_angular_speed > 0.0:
                    turn_ratio = abs(omega) / self.max_angular_speed   # 0..1
                    linear = self.chase_linear_speed * (1.0 - 0.5 * turn_ratio)
                else:
                    linear = self.chase_linear_speed

                self._last_omega  = omega
                self._last_linear = linear

                self.get_logger().debug(
                    f'[CHASE] err={error:+.3f}  omega={omega:+.3f}  '
                    f'linear={linear:.3f}  pid_dt={pid_dt:.3f}s')

            self.publish_cmd(self._last_linear, self._last_omega)
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
