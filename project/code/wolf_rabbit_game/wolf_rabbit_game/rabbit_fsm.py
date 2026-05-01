from typing import List, Optional, Tuple

import math
import random

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .utils import clamp, is_stale, string_msg_to_dict, wrap_to_pi, yaw_from_quaternion


# ── generic helpers ───────────────────────────────────────────────────────────

def _bool(val, default: bool = False) -> bool:
    """Safe bool parser for values coming from string_msg_to_dict."""
    if isinstance(val, bool):
        return val
    if val is None:
        return default
    if isinstance(val, (int, float)):
        return bool(val)
    if isinstance(val, str):
        s = val.strip().lower()
        if s in ('true', '1', 'yes', 'y', 'on'):
            return True
        if s in ('false', '0', 'no', 'n', 'off'):
            return False
    return bool(val)


def _float(val, default: float = 0.0) -> float:
    try:
        return float(val)
    except (TypeError, ValueError):
        return default


# ── LIDAR helpers ─────────────────────────────────────────────────────────────

def _deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _index_for_angle(scan: LaserScan, angle_rad: float) -> int:
    if scan.angle_increment == 0.0 or len(scan.ranges) == 0:
        return 0
    idx = int(round((angle_rad - scan.angle_min) / scan.angle_increment))
    return max(0, min(idx, len(scan.ranges) - 1))


def _sector_min(scan: LaserScan, start_deg: float, end_deg: float) -> float:
    """Minimum finite range in the given angular sector, in degrees."""
    if len(scan.ranges) == 0:
        return float('inf')
    i0 = _index_for_angle(scan, _deg_to_rad(start_deg))
    i1 = _index_for_angle(scan, _deg_to_rad(end_deg))
    if i0 > i1:
        i0, i1 = i1, i0
    valid = [r for r in scan.ranges[i0:i1 + 1] if math.isfinite(r) and r > 0.0]
    return min(valid) if valid else float('inf')


# ── wandering sub-states ──────────────────────────────────────────────────────
_W_FORWARD = 'FORWARD'
_W_SCANNING = 'SCANNING'
_W_TURNING = 'TURNING'


class RabbitFSM(Node):
    """
    Rabbit behavior node.

    Behavior difference from wolf:
      1. Rabbit starts from a safe/home zone.
      2. When started, it wanders near the safe zone instead of patrolling a wolf territory.
      3. When it sees the wolf, it returns to the safe zone instead of chasing.
      4. Energy drains over time. At zero energy, rabbit rests until recovered.
    """

    def __init__(self) -> None:
        super().__init__('rabbit_fsm')

        # ── ROS topics ────────────────────────────────────────────────────────
        self.declare_parameter('vision_topic', '/rabbit/vision')
        self.declare_parameter('geofence_topic', '/rabbit/geofence')
        self.declare_parameter('game_state_topic', '/game/state')
        self.declare_parameter('odom_topic', '/rabbit/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_unstamped')
        self.declare_parameter('scan_topic', '/scan')

        # ── safe zone / home behavior ────────────────────────────────────────
        # By default the first odometry pose is treated as the safe-zone center.
        # If you want a fixed map coordinate, set home_from_first_odom:=False
        # and pass home_x/home_y from launch.
        self.declare_parameter('home_from_first_odom', True)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('safe_radius', 0.45)
        self.declare_parameter('wander_radius', 1.30)
        self.declare_parameter('return_arrival_radius', 0.25)

        # ── motion speeds ────────────────────────────────────────────────────
        self.declare_parameter('wander_linear_speed', 0.10)
        self.declare_parameter('wander_turn_speed', 0.60)
        self.declare_parameter('return_home_speed', 0.28)
        self.declare_parameter('return_heading_gain', 1.8)
        self.declare_parameter('return_yaw_drive_threshold', 0.55)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('control_hz', 20.0)

        # ── random wandering / obstacle avoidance ────────────────────────────
        self.declare_parameter('wander_straight_min_sec', 1.5)
        self.declare_parameter('wander_straight_max_sec', 4.0)
        self.declare_parameter('wander_scan_start_deg', -30.0)
        self.declare_parameter('wander_scan_end_deg', 30.0)
        self.declare_parameter('obstacle_safety_distance', 0.45)
        self.declare_parameter('obstacle_turn_angular', 0.9)
        self.declare_parameter('obstacle_turn_min_deg', 80.0)
        self.declare_parameter('obstacle_turn_max_deg', 150.0)
        self.declare_parameter('obstacle_turn_sec_per_180', 3.2)
        self.declare_parameter('obstacle_cooldown_sec', 2.0)

        # ── vision / energy ──────────────────────────────────────────────────
        self.declare_parameter('vision_timeout_sec', 0.7)
        self.declare_parameter('center_x_px', 125.0)
        self.declare_parameter('max_energy', 100.0)
        self.declare_parameter('initial_energy', 100.0)
        self.declare_parameter('wander_energy_drain_per_sec', 0.6)
        self.declare_parameter('return_energy_drain_per_sec', 4.0)
        self.declare_parameter('rest_recover_per_sec', 15.0)
        self.declare_parameter('rest_resume_energy', 60.0)
        self.declare_parameter('energy_log_period_sec', 2.0)

        self._load_params()

        # ── publishers / subscribers ─────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(String, self.vision_topic, self.vision_callback, 10)
        self.create_subscription(String, self.geofence_topic, self.geofence_callback, 10)
        self.create_subscription(String, self.game_state_topic, self.game_state_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        # ── FSM state ────────────────────────────────────────────────────────
        now = self.get_clock().now()
        self.state = 'WANDER'
        self.state_enter_time = now
        self._last_step_time = now
        self._last_energy_log_time = now
        # rclpy Jazzy RcutilsLogger does not provide warn_throttle/info_throttle.
        # Keep our own throttle timestamps for compatible periodic logs.
        self._throttle_log_last = {}

        self.vision = {}
        self.geofence = {}
        self.game_state = {}
        self.latest_scan: Optional[LaserScan] = None

        self.rabbit_x: Optional[float] = None
        self.rabbit_y: Optional[float] = None
        self.current_yaw = 0.0
        self.have_odom = False

        self.home_x = self.home_x_param
        self.home_y = self.home_y_param
        self.home_ready = not self.home_from_first_odom

        self.energy = clamp(self.initial_energy, 0.0, self.max_energy)

        self._wander_sub = _W_FORWARD
        self._wander_sub_start = now.nanoseconds / 1e9
        self._wander_sub_dur = random.uniform(
            self.wander_straight_min_sec,
            self.wander_straight_max_sec,
        )
        self._wander_turn_dir = 1.0
        self._scan_target_yaw: Optional[float] = None
        self._last_obstacle_time = 0.0

        control_period = 1.0 / max(1.0, self.control_hz)
        self.timer = self.create_timer(control_period, self.step)

        self.get_logger().info(
            f'RabbitFSM started. vision={self.vision_topic}, odom={self.odom_topic}, '
            f'cmd_vel={self.cmd_vel_topic}, home_from_first_odom={self.home_from_first_odom}'
        )

    # ── parameter loading ─────────────────────────────────────────────────────

    def _load_params(self) -> None:
        g = self.get_parameter
        self.vision_topic = str(g('vision_topic').value)
        self.geofence_topic = str(g('geofence_topic').value)
        self.game_state_topic = str(g('game_state_topic').value)
        self.odom_topic = str(g('odom_topic').value)
        self.cmd_vel_topic = str(g('cmd_vel_topic').value)
        self.scan_topic = str(g('scan_topic').value)

        self.home_from_first_odom = bool(g('home_from_first_odom').value)
        self.home_x_param = float(g('home_x').value)
        self.home_y_param = float(g('home_y').value)
        self.safe_radius = float(g('safe_radius').value)
        self.wander_radius = float(g('wander_radius').value)
        self.return_arrival_radius = float(g('return_arrival_radius').value)

        self.wander_linear_speed = float(g('wander_linear_speed').value)
        self.wander_turn_speed = float(g('wander_turn_speed').value)
        self.return_home_speed = float(g('return_home_speed').value)
        self.return_heading_gain = float(g('return_heading_gain').value)
        self.return_yaw_drive_threshold = float(g('return_yaw_drive_threshold').value)
        self.max_angular_speed = float(g('max_angular_speed').value)
        self.control_hz = float(g('control_hz').value)

        self.wander_straight_min_sec = float(g('wander_straight_min_sec').value)
        self.wander_straight_max_sec = float(g('wander_straight_max_sec').value)
        self.wander_scan_start_deg = float(g('wander_scan_start_deg').value)
        self.wander_scan_end_deg = float(g('wander_scan_end_deg').value)
        self.obstacle_safety_distance = float(g('obstacle_safety_distance').value)
        self.obstacle_turn_angular = float(g('obstacle_turn_angular').value)
        self.obstacle_turn_min_deg = float(g('obstacle_turn_min_deg').value)
        self.obstacle_turn_max_deg = float(g('obstacle_turn_max_deg').value)
        self.obstacle_turn_sec_per_180 = float(g('obstacle_turn_sec_per_180').value)
        self.obstacle_cooldown_sec = float(g('obstacle_cooldown_sec').value)

        self.vision_timeout_sec = float(g('vision_timeout_sec').value)
        self.center_x_px = float(g('center_x_px').value)
        self.max_energy = float(g('max_energy').value)
        self.initial_energy = float(g('initial_energy').value)
        self.wander_energy_drain_per_sec = float(g('wander_energy_drain_per_sec').value)
        self.return_energy_drain_per_sec = float(g('return_energy_drain_per_sec').value)
        self.rest_recover_per_sec = float(g('rest_recover_per_sec').value)
        self.rest_resume_energy = float(g('rest_resume_energy').value)
        self.energy_log_period_sec = float(g('energy_log_period_sec').value)

    # ── callbacks ─────────────────────────────────────────────────────────────

    def vision_callback(self, msg: String) -> None:
        self.vision = string_msg_to_dict(msg)

    def geofence_callback(self, msg: String) -> None:
        # Expected style: key=value pairs. This node also tolerates plain SAFE strings.
        parsed = string_msg_to_dict(msg)
        if parsed:
            self.geofence = parsed
        else:
            self.geofence = {'state': msg.data.strip()}

    def game_state_callback(self, msg: String) -> None:
        self.game_state = string_msg_to_dict(msg)

    def odom_callback(self, msg: Odometry) -> None:
        self.rabbit_x = float(msg.pose.pose.position.x)
        self.rabbit_y = float(msg.pose.pose.position.y)
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_odom = True

        if self.home_from_first_odom and not self.home_ready:
            self.home_x = self.rabbit_x
            self.home_y = self.rabbit_y
            self.home_ready = True
            self.get_logger().info(
                f'Safe zone locked from first odom: home=({self.home_x:.2f}, {self.home_y:.2f})'
            )

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    # ── generic FSM helpers ──────────────────────────────────────────────────

    def enter_state(self, new_state: str) -> None:
        if self.state == new_state:
            return
        old_state = self.state
        self.state = new_state
        self.state_enter_time = self.get_clock().now()

        if new_state == 'WANDER':
            self._enter_wander_sub(
                _W_FORWARD,
                random.uniform(self.wander_straight_min_sec, self.wander_straight_max_sec),
            )
        elif new_state == 'REST':
            self.publish_cmd(0.0, 0.0)
        elif new_state == 'RETURN_HOME':
            self._scan_target_yaw = None

        self.get_logger().info(
            f'Rabbit state: {old_state} -> {new_state} | energy={self.energy:.1f}'
        )

    def seconds_in_state(self) -> float:
        return (self.get_clock().now() - self.state_enter_time).nanoseconds / 1e9

    def publish_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def _log_throttle(self, level: str, key: str, period_sec: float, text: str) -> None:
        """Logger throttle compatible with ROS2 Jazzy RcutilsLogger.

        Some ROS2 Python loggers do not have warn_throttle()/info_throttle(),
        so this helper prevents repeated messages without depending on those APIs.
        """
        now_s = self.get_clock().now().nanoseconds / 1e9
        last_s = self._throttle_log_last.get(key, -1.0e18)
        if now_s - last_s < max(0.0, period_sec):
            return
        self._throttle_log_last[key] = now_s
        logger = self.get_logger()
        if level == 'warn':
            logger.warn(text)
        elif level == 'error':
            logger.error(text)
        elif level == 'debug':
            logger.debug(text)
        else:
            logger.info(text)

    def _enter_wander_sub(self, sub: str, duration: float = 0.0) -> None:
        self._wander_sub = sub
        self._wander_sub_start = self.get_clock().now().nanoseconds / 1e9
        self._wander_sub_dur = max(0.0, duration)
        if sub != _W_SCANNING:
            self._scan_target_yaw = None

    # ── perception / geometry helpers ────────────────────────────────────────

    def vision_fresh(self) -> bool:
        return not is_stale(self.vision.get('stamp'), self.vision_timeout_sec)

    def wolf_visible(self) -> bool:
        return self.vision_fresh() and _bool(self.vision.get('wolf_visible', False))

    def _pose_ready(self) -> bool:
        return self.have_odom and self.rabbit_x is not None and self.rabbit_y is not None

    def distance_to_home(self) -> Optional[float]:
        if not self.home_ready or not self._pose_ready():
            return None
        dx = self.home_x - float(self.rabbit_x)
        dy = self.home_y - float(self.rabbit_y)
        return math.hypot(dx, dy)

    def in_safe_zone(self) -> bool:
        # Prefer explicit geofence boolean keys if your geofence node publishes them.
        for key in (
            'inside_safe_zone',
            'inside_home_zone',
            'inside_rabbit_safe_zone',
            'rabbit_inside_safe_zone',
        ):
            if key in self.geofence:
                return _bool(self.geofence.get(key), False)

        # Fallback: distance from home center.
        dist = self.distance_to_home()
        return dist is not None and dist <= self.safe_radius

    def _front_min_distance(self) -> float:
        if self.latest_scan is None:
            return float('inf')
        return _sector_min(self.latest_scan, self.wander_scan_start_deg, self.wander_scan_end_deg)

    def _obstacle_in_front(self) -> bool:
        return self._front_min_distance() < self.obstacle_safety_distance

    def drive_to_xy(self, target_x: float, target_y: float, speed: float) -> None:
        if not self._pose_ready():
            self.publish_cmd(0.0, 0.0)
            return

        dx = float(target_x) - float(self.rabbit_x)
        dy = float(target_y) - float(self.rabbit_y)
        target_yaw = math.atan2(dy, dx)
        yaw_error = wrap_to_pi(target_yaw - self.current_yaw)

        omega = clamp(
            self.return_heading_gain * yaw_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        # Do not drive forward hard while facing far away from the target.
        if abs(yaw_error) > self.return_yaw_drive_threshold:
            linear = 0.0
        else:
            turn_ratio = abs(omega) / max(0.001, self.max_angular_speed)
            linear = float(speed) * (1.0 - 0.45 * turn_ratio)

        self.publish_cmd(linear, omega)

    # ── energy helpers ───────────────────────────────────────────────────────

    def _drain_energy(self, rate_per_sec: float, dt: float) -> None:
        self.energy = clamp(self.energy - max(0.0, rate_per_sec) * max(0.0, dt), 0.0, self.max_energy)

    def _recover_energy(self, dt: float) -> None:
        self.energy = clamp(
            self.energy + max(0.0, self.rest_recover_per_sec) * max(0.0, dt),
            0.0,
            self.max_energy,
        )

    def _log_energy_periodically(self) -> None:
        now = self.get_clock().now()
        if (now - self._last_energy_log_time).nanoseconds / 1e9 >= self.energy_log_period_sec:
            self._last_energy_log_time = now
            self.get_logger().info(f'[ENERGY] state={self.state} energy={self.energy:.1f}')

    # ── behavior blocks ──────────────────────────────────────────────────────

    def boundary_recovery(self) -> bool:
        outside_global = _bool(self.geofence.get('outside_global_arena', False))
        near_global = _bool(self.geofence.get('near_global_boundary', False))
        if not (outside_global or near_global):
            return False

        # If the arena boundary is hit, bias motion back to home.
        if self.home_ready and self._pose_ready():
            self.drive_to_xy(self.home_x, self.home_y, self.wander_linear_speed)
        else:
            self.publish_cmd(0.0, self.wander_turn_speed)
        return True

    def wander_step(self) -> None:
        """Wander near the home/safe zone with obstacle avoidance."""
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self._wander_sub_start

        if self.boundary_recovery():
            return

        # Keep the rabbit near the safe zone. If it drifts too far, go home.
        dist = self.distance_to_home()
        if dist is not None and dist > self.wander_radius:
            self.get_logger().debug(f'[WANDER] Outside wander radius ({dist:.2f}m), returning closer to home')
            self.drive_to_xy(self.home_x, self.home_y, self.wander_linear_speed)
            return

        if self._wander_sub == _W_TURNING:
            if elapsed < self._wander_sub_dur:
                self.publish_cmd(0.0, self.obstacle_turn_angular * self._wander_turn_dir)
            else:
                dur = random.uniform(self.wander_straight_min_sec, self.wander_straight_max_sec)
                self._enter_wander_sub(_W_FORWARD, dur)
            return

        if self._wander_sub == _W_SCANNING:
            if not self._pose_ready():
                self.publish_cmd(0.0, self.wander_turn_speed)
                if elapsed > 1.0:
                    dur = random.uniform(self.wander_straight_min_sec, self.wander_straight_max_sec)
                    self._enter_wander_sub(_W_FORWARD, dur)
                return

            if self._scan_target_yaw is None:
                self._scan_target_yaw = random.uniform(-math.pi, math.pi)
                self.get_logger().debug(
                    f'[WANDER] New random heading {math.degrees(self._scan_target_yaw):.1f}deg'
                )

            yaw_err = wrap_to_pi(self._scan_target_yaw - self.current_yaw)
            if abs(yaw_err) < 0.12:
                dur = random.uniform(self.wander_straight_min_sec, self.wander_straight_max_sec)
                self._enter_wander_sub(_W_FORWARD, dur)
            else:
                omega = clamp(2.0 * yaw_err, -self.wander_turn_speed, self.wander_turn_speed)
                self.publish_cmd(0.0, omega)
            return

        # FORWARD phase
        front_min = self._front_min_distance()
        if (front_min < self.obstacle_safety_distance
                and now - self._last_obstacle_time >= self.obstacle_cooldown_sec):
            turn_deg = random.uniform(self.obstacle_turn_min_deg, self.obstacle_turn_max_deg)
            turn_dur = (self.obstacle_turn_sec_per_180 / 180.0) * turn_deg
            self._wander_turn_dir = random.choice([-1.0, 1.0])
            self._last_obstacle_time = now
            self.get_logger().info(
                f'[WANDER] Obstacle {front_min:.2f}m ahead, turning '
                f'{"left" if self._wander_turn_dir > 0 else "right"} for {turn_dur:.2f}s'
            )
            self._enter_wander_sub(_W_TURNING, turn_dur)
            self.publish_cmd(0.0, self.obstacle_turn_angular * self._wander_turn_dir)
            return

        if elapsed >= self._wander_sub_dur:
            self._enter_wander_sub(_W_SCANNING)
            self.publish_cmd(0.0, 0.0)
            return

        self.publish_cmd(self.wander_linear_speed, 0.0)

    def return_home_step(self) -> None:
        """When wolf is visible, go back to the safe zone instead of chasing."""
        if not self.home_ready or not self._pose_ready():
            self.publish_cmd(0.0, 0.0)
            self._log_throttle('warn', 'return_wait_odom_home', 2.0, '[RETURN_HOME] Waiting for odom/home position')
            return

        dist = self.distance_to_home()
        if dist is not None and dist <= self.return_arrival_radius:
            # Arrived at safe zone. If wolf is still visible, stay still and hide.
            self.publish_cmd(0.0, 0.0)
            if not self.wolf_visible():
                self.enter_state('WANDER')
            return

        # If an obstacle blocks the direct return path, rotate briefly, then try again.
        if self._obstacle_in_front() and self.seconds_in_state() > 0.2:
            self.publish_cmd(0.0, self.obstacle_turn_angular)
            return

        self.drive_to_xy(self.home_x, self.home_y, self.return_home_speed)

    def rest_step(self, dt: float) -> None:
        self.publish_cmd(0.0, 0.0)
        self._recover_energy(dt)

        if self.energy >= self.rest_resume_energy:
            # After recovery, if wolf is still visible and rabbit is not safe, go home.
            if self.wolf_visible() and not self.in_safe_zone():
                self.enter_state('RETURN_HOME')
            else:
                self.enter_state('WANDER')

    # ── main loop ─────────────────────────────────────────────────────────────

    def step(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_step_time).nanoseconds / 1e9
        self._last_step_time = now
        if dt <= 0.0:
            dt = 1.0 / max(1.0, self.control_hz)

        rabbit_alive = _bool(self.game_state.get('rabbit_alive', True), True)
        phase = str(self.game_state.get('phase', 'ACTIVE')).upper()

        if (not rabbit_alive) or phase == 'CAPTURED':
            self.enter_state('DEAD')

        if self.state == 'DEAD':
            self.publish_cmd(0.0, 0.0)
            return

        # Energy drains whenever rabbit is active. REST recovers energy.
        if self.state == 'WANDER':
            self._drain_energy(self.wander_energy_drain_per_sec, dt)
        elif self.state == 'RETURN_HOME':
            self._drain_energy(self.return_energy_drain_per_sec, dt)

        self._log_energy_periodically()

        if self.energy <= 0.0 and self.state != 'REST':
            self.get_logger().info('[ENERGY] Depleted. Rabbit must rest.')
            self.enter_state('REST')

        if self.state == 'REST':
            self.rest_step(dt)
            return

        # Highest behavior priority after death/rest: wolf visible → return home.
        if self.wolf_visible():
            if self.in_safe_zone():
                # Already safe: hide/stay still, do not wander outward.
                self.publish_cmd(0.0, 0.0)
                return
            if self.state != 'RETURN_HOME':
                self.enter_state('RETURN_HOME')

        if self.state == 'RETURN_HOME':
            self.return_home_step()
            return

        if self.state == 'WANDER':
            self.wander_step()
            return

        # Fail-safe for any unexpected state.
        self.get_logger().warn(f'Unexpected state={self.state}, stopping and returning to WANDER')
        self.publish_cmd(0.0, 0.0)
        self.enter_state('WANDER')


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = RabbitFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
