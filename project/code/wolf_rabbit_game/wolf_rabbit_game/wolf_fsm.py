from typing import List

import math
import random
import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult

from .utils import clamp, is_stale, string_msg_to_dict, wrap_to_pi, yaw_from_quaternion


def _bool(val, default: bool = False) -> bool:
    """Safe bool parser for values coming from string_msg_to_dict."""
    if isinstance(val, bool):
        return val
    if isinstance(val, str):
        return val.strip().lower() == 'true'
    if val is None:
        return default
    return bool(val)


class PID:
    """Simple PID controller for a single axis with optional derivative LPF."""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_limit: float = float('inf'),
                 integral_limit: float = float('inf'),
                 d_filter_alpha: float = 0.6) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit   = output_limit
        self.integral_limit = integral_limit
        self.d_filter_alpha = d_filter_alpha

        self._integral   = 0.0
        self._prev_error = 0.0
        self._d_filtered = 0.0
        self._first_tick = True

    def reset(self) -> None:
        self._integral   = 0.0
        self._prev_error = 0.0
        self._d_filtered = 0.0
        self._first_tick = True

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0
        if self._first_tick:
            derivative_raw = 0.0
            self._first_tick = False
        else:
            derivative_raw = (error - self._prev_error) / dt
        a = clamp(self.d_filter_alpha, 0.0, 0.99)
        self._d_filtered = a * self._d_filtered + (1.0 - a) * derivative_raw
        self._integral += error * dt
        self._integral = clamp(self._integral, -self.integral_limit, self.integral_limit)
        output = self.kp * error + self.ki * self._integral + self.kd * self._d_filtered
        self._prev_error = error
        return clamp(output, -self.output_limit, self.output_limit)


class RabbitPixelKF:
    """Constant-velocity Kalman filter for the rabbit's x-position in the image."""

    def __init__(self, pixels_per_rad, process_px_var, process_vx_var, meas_var):
        self.pixels_per_rad = pixels_per_rad
        self.process_px_var = process_px_var
        self.process_vx_var = process_vx_var
        self.meas_var       = meas_var
        self._x = [0.0, 0.0]
        self._P = [1e6, 0.0, 0.0, 1e6]
        self.initialized = False

    def reset(self):
        self._x = [0.0, 0.0]
        self._P = [1e6, 0.0, 0.0, 1e6]
        self.initialized = False

    def set_pixels_per_rad(self, ppr):
        if ppr > 0.0:
            self.pixels_per_rad = ppr

    def predict(self, dt, omega_cmd):
        if dt <= 0.0 or not self.initialized:
            return
        px, vx = self._x
        self._x = [px + vx * dt - omega_cmd * dt * self.pixels_per_rad, vx]
        p00, p01, p10, p11 = self._P
        a00 = p00 + dt * p10; a01 = p01 + dt * p11; a10 = p10; a11 = p11
        n00 = a00 + a01 * dt + self.process_px_var * dt
        n01 = a01; n10 = a10 + a11 * dt; n11 = a11 + self.process_vx_var * dt
        self._P = [n00, n01, n10, n11]

    def update(self, measurement_px, confidence=1.0):
        conf = max(0.1, min(1.0, float(confidence)))
        R = self.meas_var / (conf * conf)
        if not self.initialized:
            self._x = [measurement_px, 0.0]
            self._P = [R, 0.0, 0.0, 1e4]
            self.initialized = True
            return
        px, vx = self._x
        y = measurement_px - px
        p00, p01, p10, p11 = self._P
        S = p00 + R
        if S <= 0.0:
            return
        k0 = p00 / S; k1 = p10 / S
        self._x = [px + k0 * y, vx + k1 * y]
        self._P = [(1.0 - k0) * p00, (1.0 - k0) * p01,
                   -k1 * p00 + p10,  -k1 * p01 + p11]

    @property
    def px(self): return self._x[0]
    @property
    def vx(self): return self._x[1]
    @property
    def px_variance(self): return self._P[0]


# ── LIDAR helpers ──────────────────────────────────────────────────────────────

def _deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0

def _index_for_angle(scan: LaserScan, angle_rad: float) -> int:
    if scan.angle_increment == 0.0:
        return 0
    idx = int(round((angle_rad - scan.angle_min) / scan.angle_increment))
    return max(0, min(idx, len(scan.ranges) - 1))

def _sector_min(scan: LaserScan, start_deg: float, end_deg: float) -> float:
    """Minimum finite range in the given angular sector (degrees)."""
    i0 = _index_for_angle(scan, _deg_to_rad(start_deg))
    i1 = _index_for_angle(scan, _deg_to_rad(end_deg))
    if i0 > i1:
        i0, i1 = i1, i0
    valid = [r for r in scan.ranges[i0: i1 + 1] if math.isfinite(r) and r > 0.0]
    return min(valid) if valid else float('inf')


# ── patrol sub-states ──────────────────────────────────────────────────────────
_P_FORWARD      = 'FORWARD'       # drive straight for a random time
_P_SCANNING     = 'SCANNING'      # stopped; closed-loop spin to random heading
_P_TURNING      = 'TURNING'       # timed obstacle-avoidance turn
_P_FORCE_FWD    = 'FORCE_FORWARD' # straight blast after obstacle turn
_P_BREACH_ALIGN = 'BREACH_ALIGN'  # AMCL feedback: spin to face centroid
_P_BREACH_DRIVE = 'BREACH_DRIVE'  # AMCL feedback: drive until back inside


class WolfFSM(Node):
    def __init__(self) -> None:
        super().__init__('wolf_fsm')

        # ── parameters ────────────────────────────────────────────────────────
        self.declare_parameter('vision_topic',               '/wolf/vision')

        # Subscribe to the SIMPLE geofence topic — publishes exactly
        # "SAFE", "WARNING", or "BREACH" as a plain string.
        # Much easier to consume than parsing the key=value detail topic.
        self.declare_parameter('wolf_geofence_topic',        '/wolf/geofence')

        self.declare_parameter('rabbit_geofence_topic',      '/rabbit/geofence')
        self.declare_parameter('game_state_topic',           '/game/state')
        self.declare_parameter('odom_topic',                 '/wolf/odom')
        self.declare_parameter('cmd_vel_topic',              '/cmd_vel_unstamped')

        # Fixed: was /robot_04/scan, now matches the actual robot topic
        self.declare_parameter('scan_topic',                 '/scan')
        self.declare_parameter('amcl_pose_topic',            '/amcl_pose')
        # Also subscribe to the detail topic to read dist= for smarter recovery
        self.declare_parameter('wolf_geofence_detail_topic', '/wolf/geofence/detail')
        # Polygon vertices flat list [x0,y0,x1,y1,...] — used to compute centroid
        self.declare_parameter('zone_polygon',
            [-2.0, -2.0, 2.0, -2.0, 2.0, 2.0, -2.0, 2.0])
        # WARNING: turn inward toward centroid instead of generic curve
        self.declare_parameter('patrol_warning_turn_to_centroid', True)

        # Patrol
        self.declare_parameter('patrol_linear_speed',        0.10)
        self.declare_parameter('patrol_turn_speed',          0.65)
        self.declare_parameter('patrol_warning_speed_scale', 0.5)

        # Obstacle avoidance
        # How long to drive straight before picking a new random heading
        self.declare_parameter('patrol_straight_min_sec',    2.0)
        self.declare_parameter('patrol_straight_max_sec',    5.0)
        self.declare_parameter('patrol_safety_distance',     0.5)
        self.declare_parameter('patrol_obstacle_angular',    1.0)
        self.declare_parameter('patrol_turn_min_deg',        90.0)
        self.declare_parameter('patrol_turn_max_deg',        180.0)
        self.declare_parameter('patrol_turn_sec_per_180',    3.2)
        self.declare_parameter('patrol_force_fwd_sec',       2.0)
        self.declare_parameter('patrol_retreat_cooldown',    3.0)
        self.declare_parameter('patrol_scan_start_deg',     -30.0)  # forward cone: -30° to +30°
        self.declare_parameter('patrol_scan_end_deg',        30.0)

        # Chase
        self.declare_parameter('chase_linear_speed',         1.5)
        self.declare_parameter('chase_warning_speed_scale',  0.5)
        self.declare_parameter('return_turn_speed',          0.75)
        self.declare_parameter('vision_timeout_sec',         0.7)
        self.declare_parameter('center_x_px',                125.0)
        self.declare_parameter('max_angular_speed',          1.5)
        self.declare_parameter('return_turn_angle_deg',      160.0)
        self.declare_parameter('chase_kp',                   2.0)
        self.declare_parameter('chase_ki',                   0.0)
        self.declare_parameter('chase_kd',                   0.15)
        self.declare_parameter('chase_deadband',             0.02)
        self.declare_parameter('chase_speed_scale_on_turn',  True)
        self.declare_parameter('chase_lost_frames',          5)
        self.declare_parameter('catch_bbox_width_px',        65.0)
        self.declare_parameter('catch_confirm_frames',       2)
        self.declare_parameter('catch_pause_sec',            2.0)
        self.declare_parameter('catch_cooldown_sec',         4.0)

        # Kalman filter
        self.declare_parameter('kf_pixels_per_rad',          240.0)
        self.declare_parameter('kf_process_px_var',          50.0)
        self.declare_parameter('kf_process_vx_var',          2500.0)
        self.declare_parameter('kf_meas_var',                25.0)
        self.declare_parameter('kf_feedforward_sec',         0.15)
        self.declare_parameter('kf_sign',                    1.0)
        self.declare_parameter('control_hz',                 50.0)

        self.add_on_set_parameters_callback(self._on_param_change)
        self._load_params()

        # ── PID / KF ──────────────────────────────────────────────────────────
        self.chase_pid = PID(
            kp=float(self.get_parameter('chase_kp').value),
            ki=float(self.get_parameter('chase_ki').value),
            kd=float(self.get_parameter('chase_kd').value),
            output_limit=self.max_angular_speed,
            integral_limit=1.0,
        )
        self.kf = RabbitPixelKF(
            pixels_per_rad=float(self.get_parameter('kf_pixels_per_rad').value),
            process_px_var=float(self.get_parameter('kf_process_px_var').value),
            process_vx_var=float(self.get_parameter('kf_process_vx_var').value),
            meas_var=float(self.get_parameter('kf_meas_var').value),
        )

        # ── publishers / subscribers ──────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.create_subscription(String,    self.vision_topic,          self.vision_callback,          10)
        self.create_subscription(String,    self.wolf_geofence_topic,   self.wolf_geofence_callback,   10)
        self.create_subscription(String,    self.rabbit_geofence_topic, self.rabbit_geofence_callback, 10)
        self.create_subscription(String,    self.game_state_topic,      self.game_state_callback,      10)
        self.create_subscription(Odometry,  self.odom_topic,            self.odom_callback,            10)
        self.create_subscription(PoseWithCovarianceStamped, self.amcl_pose_topic, self.amcl_pose_callback, 10)
        self.create_subscription(String,    self.wolf_geofence_detail_topic, self.wolf_geofence_detail_callback, 10)
        self.create_subscription(LaserScan, self.scan_topic,            self.scan_callback,            10)

        control_period = 1.0 / max(1.0, self.control_hz)
        self.timer = self.create_timer(control_period, self.step)

        # ── FSM state ─────────────────────────────────────────────────────────
        self.state             = 'PATROL'
        self.state_enter_time  = self.get_clock().now()
        self.vision            = {}
        # wolf_geofence_state is now a plain string: 'SAFE' | 'WARNING' | 'BREACH'
        # Default SAFE so the robot moves before any message arrives.
        self.wolf_geofence_state = 'SAFE'
        self.rabbit_geofence   = {}
        self.game_state        = {}
        self.latest_scan: LaserScan | None = None
        self.current_yaw       = 0.0
        self.have_odom         = False
        self.robot_x           = 0.0   # from odometry
        self.robot_y           = 0.0
        self.amcl_x            = 0.0   # from AMCL — used for centroid targeting
        self.amcl_y            = 0.0
        self.amcl_yaw          = 0.0
        self.have_amcl         = False
        # breach recovery closed-loop
        self._breach_target_yaw: float | None = None
        self.geofence_dist     = float('inf')  # from detail topic
        self.return_target_yaw = None
        self._last_step_time   = self.get_clock().now()
        self._lost_frame_count = 0
        self._last_cmd_omega   = 0.0
        self._last_vision_stamp: float | None = None
        self._catch_confirm_count  = 0
        self._catch_cooldown_until = self.get_clock().now()

        # patrol sub-state
        self._patrol_sub        = _P_FORWARD
        self._patrol_sub_start  = 0.0
        self._patrol_sub_dur    = 0.0
        self._patrol_turn_dir   = 1.0
        self._last_retreat_time = 0.0

        # random-heading patrol — target yaw for the SCANNING sub-state
        self._scan_target_yaw: float | None = None

        self.get_logger().info(
            f'WolfFSM started. '
            f'Geofence: {self.wolf_geofence_topic} | Scan: {self.scan_topic}')

    # ── parameter loading ──────────────────────────────────────────────────────

    def _load_params(self) -> None:
        g = self.get_parameter
        self.vision_topic               = str(g('vision_topic').value)
        self.wolf_geofence_topic        = str(g('wolf_geofence_topic').value)
        self.rabbit_geofence_topic      = str(g('rabbit_geofence_topic').value)
        self.game_state_topic           = str(g('game_state_topic').value)
        self.odom_topic                 = str(g('odom_topic').value)
        self.cmd_vel_topic              = str(g('cmd_vel_topic').value)
        self.scan_topic                    = str(g('scan_topic').value)
        self.amcl_pose_topic               = str(g('amcl_pose_topic').value)
        self.wolf_geofence_detail_topic    = str(g('wolf_geofence_detail_topic').value)
        self.patrol_warning_turn_centroid  = bool(g('patrol_warning_turn_to_centroid').value)
        # Build polygon and centroid from parameter
        flat = list(g('zone_polygon').value)
        self._polygon = [(flat[i], flat[i+1]) for i in range(0, len(flat), 2)]
        xs = [p[0] for p in self._polygon]
        ys = [p[1] for p in self._polygon]
        self._centroid = (sum(xs) / len(xs), sum(ys) / len(ys))
        self.patrol_linear_speed        = float(g('patrol_linear_speed').value)
        self.patrol_turn_speed          = float(g('patrol_turn_speed').value)
        self.patrol_warning_speed_scale = float(g('patrol_warning_speed_scale').value)
        self.patrol_straight_min_sec    = float(g('patrol_straight_min_sec').value)
        self.patrol_straight_max_sec    = float(g('patrol_straight_max_sec').value)
        self.patrol_safety_distance     = float(g('patrol_safety_distance').value)
        self.patrol_obstacle_angular    = float(g('patrol_obstacle_angular').value)
        self.patrol_turn_min_deg        = float(g('patrol_turn_min_deg').value)
        self.patrol_turn_max_deg        = float(g('patrol_turn_max_deg').value)
        self.patrol_turn_sec_per_180    = float(g('patrol_turn_sec_per_180').value)
        self.patrol_force_fwd_sec       = float(g('patrol_force_fwd_sec').value)
        self.patrol_retreat_cooldown    = float(g('patrol_retreat_cooldown').value)
        self.patrol_scan_start_deg      = float(g('patrol_scan_start_deg').value)
        self.patrol_scan_end_deg        = float(g('patrol_scan_end_deg').value)
        self.chase_linear_speed         = float(g('chase_linear_speed').value)
        self.chase_warning_speed_scale  = float(g('chase_warning_speed_scale').value)
        self.return_turn_speed          = float(g('return_turn_speed').value)
        self.vision_timeout_sec         = float(g('vision_timeout_sec').value)
        self.center_x_px                = float(g('center_x_px').value)
        self.max_angular_speed          = float(g('max_angular_speed').value)
        self.return_turn_angle          = float(g('return_turn_angle_deg').value) * math.pi / 180.0
        self.chase_deadband             = float(g('chase_deadband').value)
        self.chase_speed_scale          = bool(g('chase_speed_scale_on_turn').value)
        self.chase_lost_frames          = int(g('chase_lost_frames').value)
        self.catch_bbox_width_px        = float(g('catch_bbox_width_px').value)
        self.catch_confirm_frames       = int(g('catch_confirm_frames').value)
        self.catch_pause_sec            = float(g('catch_pause_sec').value)
        self.catch_cooldown_sec         = float(g('catch_cooldown_sec').value)
        self.kf_feedforward_sec         = float(g('kf_feedforward_sec').value)
        self.kf_sign                    = float(g('kf_sign').value)
        self.control_hz                 = float(g('control_hz').value)

    # ── callbacks ─────────────────────────────────────────────────────────────

    def vision_callback(self, msg: String) -> None:
        self.vision = string_msg_to_dict(msg)

    def wolf_geofence_callback(self, msg: String) -> None:
        # /wolf/geofence publishes exactly "SAFE", "WARNING", or "BREACH".
        # Strip whitespace and validate; ignore anything unexpected.
        state = msg.data.strip()
        if state in ('SAFE', 'WARNING', 'BREACH'):
            if state != self.wolf_geofence_state:
                self.get_logger().info(f'Geofence: {self.wolf_geofence_state} → {state}')
            self.wolf_geofence_state = state
        else:
            self.get_logger().warn(f'Unexpected geofence message: "{state}"')

    def rabbit_geofence_callback(self, msg: String) -> None:
        self.rabbit_geofence = string_msg_to_dict(msg)

    def game_state_callback(self, msg: String) -> None:
        self.game_state = string_msg_to_dict(msg)

    def odom_callback(self, msg: Odometry) -> None:
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.robot_x     = msg.pose.pose.position.x
        self.robot_y     = msg.pose.pose.position.y
        self.have_odom   = True

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl_x   = msg.pose.pose.position.x
        self.amcl_y   = msg.pose.pose.position.y
        self.amcl_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_amcl = True

    def wolf_geofence_detail_callback(self, msg: String) -> None:
        # Parse dist= from the detail topic so we know how far outside we are.
        d = string_msg_to_dict(msg)
        try:
            self.geofence_dist = float(d.get('dist', float('inf')))
        except (TypeError, ValueError):
            pass

    def _angle_to_centroid(self) -> float:
        """Yaw angle (radians) from the robot's current position toward the
        polygon centroid.  Prefers AMCL pose (accurate map-frame position);
        falls back to odometry if AMCL hasn't arrived yet."""
        cx, cy = self._centroid
        if self.have_amcl:
            return math.atan2(cy - self.amcl_y, cx - self.amcl_x)
        return math.atan2(cy - self.robot_y, cx - self.robot_x)

    def _yaw_for_centroid(self) -> float:
        """Current best yaw estimate — AMCL preferred over odometry."""
        if self.have_amcl:
            return self.amcl_yaw
        return self.current_yaw

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _on_param_change(self, params):
        for p in params:
            n, v = p.name, p.value
            if   n == 'chase_linear_speed':          self.chase_linear_speed         = float(v)
            elif n == 'patrol_linear_speed':          self.patrol_linear_speed        = float(v)
            elif n == 'patrol_turn_speed':            self.patrol_turn_speed          = float(v)
            elif n == 'patrol_warning_speed_scale':   self.patrol_warning_speed_scale = float(v)
            elif n == 'chase_warning_speed_scale':    self.chase_warning_speed_scale  = float(v)
            elif n == 'patrol_straight_min_sec':       self.patrol_straight_min_sec    = float(v)
            elif n == 'patrol_straight_max_sec':       self.patrol_straight_max_sec    = float(v)
            elif n == 'patrol_safety_distance':       self.patrol_safety_distance     = float(v)
            elif n == 'patrol_obstacle_angular':      self.patrol_obstacle_angular    = float(v)
            elif n == 'patrol_turn_min_deg':          self.patrol_turn_min_deg        = float(v)
            elif n == 'patrol_turn_max_deg':          self.patrol_turn_max_deg        = float(v)
            elif n == 'patrol_turn_sec_per_180':      self.patrol_turn_sec_per_180    = float(v)
            elif n == 'patrol_force_fwd_sec':         self.patrol_force_fwd_sec       = float(v)
            elif n == 'patrol_retreat_cooldown':      self.patrol_retreat_cooldown    = float(v)
            elif n == 'patrol_scan_start_deg':        self.patrol_scan_start_deg      = float(v)
            elif n == 'patrol_scan_end_deg':          self.patrol_scan_end_deg        = float(v)
            elif n == 'return_turn_speed':            self.return_turn_speed          = float(v)
            elif n == 'max_angular_speed':
                self.max_angular_speed = float(v)
                self.chase_pid.output_limit = float(v)
            elif n == 'chase_kp':                     self.chase_pid.kp               = float(v)
            elif n == 'chase_ki':                     self.chase_pid.ki               = float(v)
            elif n == 'chase_kd':                     self.chase_pid.kd               = float(v)
            elif n == 'chase_deadband':               self.chase_deadband             = float(v)
            elif n == 'chase_speed_scale_on_turn':    self.chase_speed_scale          = bool(v)
            elif n == 'chase_lost_frames':            self.chase_lost_frames          = int(v)
            elif n == 'catch_bbox_width_px':          self.catch_bbox_width_px        = float(v)
            elif n == 'catch_confirm_frames':         self.catch_confirm_frames       = int(v)
            elif n == 'catch_pause_sec':              self.catch_pause_sec            = float(v)
            elif n == 'catch_cooldown_sec':           self.catch_cooldown_sec         = float(v)
            elif n == 'vision_timeout_sec':           self.vision_timeout_sec         = float(v)
            elif n == 'center_x_px':                  self.center_x_px                = float(v)
            elif n == 'kf_pixels_per_rad':            self.kf.set_pixels_per_rad(float(v))
            elif n == 'kf_process_px_var':            self.kf.process_px_var          = float(v)
            elif n == 'kf_process_vx_var':            self.kf.process_vx_var          = float(v)
            elif n == 'kf_meas_var':                  self.kf.meas_var                = float(v)
            elif n == 'kf_feedforward_sec':           self.kf_feedforward_sec         = float(v)
            elif n == 'kf_sign':                      self.kf_sign                    = float(v)
            self.get_logger().info(f'Param updated: {n} = {v}')
        return SetParametersResult(successful=True)

    # ── helpers ───────────────────────────────────────────────────────────────

    def enter_state(self, new_state: str) -> None:
        if self.state == new_state:
            return
        prev_state = self.state
        self.state = new_state
        self.state_enter_time = self.get_clock().now()

        if new_state == 'RETURN_TURN' and self.have_odom:
            self.return_target_yaw = wrap_to_pi(self.current_yaw + self.return_turn_angle)
        elif new_state != 'RETURN_TURN':
            self.return_target_yaw = None

        if new_state == 'CHASE':
            self.chase_pid.reset()
            self.kf.reset()
            self._lost_frame_count  = 0
            self._last_vision_stamp = None
            self._last_cmd_omega    = 0.0
            self._catch_confirm_count = 0

        if new_state == 'PATROL':
            dur = random.uniform(self.patrol_straight_min_sec, self.patrol_straight_max_sec)
            self._enter_patrol_sub(_P_FORWARD, dur)

        if prev_state == 'STOP_CAUGHT':
            cooldown_ns = int(max(0.0, self.catch_cooldown_sec) * 1e9)
            self._catch_cooldown_until = (
                self.get_clock().now()
                + rclpy.duration.Duration(nanoseconds=cooldown_ns)
            )
            self._catch_confirm_count = 0
            self.get_logger().info(f'Catch cooldown armed for {self.catch_cooldown_sec:.1f}s')

        self.get_logger().info(f'Wolf state -> {new_state}')

    def _enter_patrol_sub(self, sub: str, duration: float = 0.0) -> None:
        self._patrol_sub       = sub
        self._patrol_sub_start = self.get_clock().now().nanoseconds / 1e9
        self._patrol_sub_dur   = duration

    def publish_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x  = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)
        self._last_cmd_omega = float(angular_z)

    def vision_fresh(self) -> bool:
        return not is_stale(self.vision.get('stamp'), self.vision_timeout_sec)

    def rabbit_visible(self) -> bool:
        return self.vision_fresh() and _bool(self.vision.get('rabbit_visible', False))

    def seconds_in_state(self) -> float:
        return (self.get_clock().now() - self.state_enter_time).nanoseconds / 1e9

    # ── patrol_step ───────────────────────────────────────────────────────────
    #
    # Sub-state machine — priority order every tick:
    #
    #  1. BREACH (only from FORWARD/SCANNING) →
    #       REVERSING → TURNING (closed-loop to centroid) → FORCE_FORWARD → FORWARD
    #  2. WARNING (only from FORWARD/SCANNING) →
    #       slow down + closed-loop P-controller toward centroid
    #  3. REVERSING  — back away from boundary (timed)
    #  4. TURNING    — closed-loop to breach_target OR timed obstacle turn
    #  5. FORCE_FORWARD — straight blast after any turn
    #  6. SCANNING   — stopped; closed-loop spin to a random heading (AMCL)
    #  7. FORWARD    — drive straight; obstacle check; after straight_dur → SCANNING

    def patrol_step(self) -> None:
        now     = self.get_clock().now().nanoseconds / 1e9
        geo     = self.wolf_geofence_state   # 'SAFE' | 'WARNING' | 'BREACH'
        elapsed = now - self._patrol_sub_start

        # ── Priority 1: BREACH ────────────────────────────────────────────────
        # Interrupts FORWARD and SCANNING only — never a mid-manoeuvre.
        # Uses AMCL as a pure feedback loop — no fixed angles, no timers.
        #
        # BREACH_ALIGN: spin in place with omega = K * yaw_error_to_centroid
        #               until yaw_error < threshold.
        # BREACH_DRIVE: drive straight while continuously correcting heading
        #               toward centroid.  Exit when geo is no longer BREACH
        #               (i.e. we've crossed back inside with margin).
        if geo == 'BREACH' and self._patrol_sub in (_P_FORWARD, _P_SCANNING):
            self._scan_target_yaw = None
            self.get_logger().warn(
                f'[PATROL] BREACH — aligning to centroid then driving back in')
            self._enter_patrol_sub(_P_BREACH_ALIGN)
            return

        if self._patrol_sub == _P_BREACH_ALIGN:
            yaw_err = wrap_to_pi(self._angle_to_centroid() - self._yaw_for_centroid())
            if abs(yaw_err) < 0.12:   # ~7° — aligned enough, start driving
                self.get_logger().info(
                    f'[PATROL] Centroid aligned (err={math.degrees(yaw_err):.1f}°) — driving back in')
                self._enter_patrol_sub(_P_BREACH_DRIVE)
            else:
                omega = clamp(3.0 * yaw_err,
                              -self.patrol_obstacle_angular,
                               self.patrol_obstacle_angular)
                self.get_logger().debug(
                    f'[PATROL] BREACH_ALIGN yaw_err={math.degrees(yaw_err):.1f}° omega={omega:.2f}')
                self.publish_cmd(0.0, omega)
            return

        if self._patrol_sub == _P_BREACH_DRIVE:
            if geo != 'BREACH':
                # We're back inside — resume normal patrol with a fresh straight leg
                self.get_logger().info(f'[PATROL] Back inside (geo={geo}) — resuming patrol')
                self._last_retreat_time = now
                dur = random.uniform(self.patrol_straight_min_sec, self.patrol_straight_max_sec)
                self._enter_patrol_sub(_P_FORWARD, dur)
            else:
                # Still outside — drive toward centroid with continuous heading correction
                yaw_err = wrap_to_pi(self._angle_to_centroid() - self._yaw_for_centroid())
                omega   = clamp(3.0 * yaw_err,
                                -self.patrol_obstacle_angular,
                                 self.patrol_obstacle_angular)
                self.get_logger().debug(
                    f'[PATROL] BREACH_DRIVE yaw_err={math.degrees(yaw_err):.1f}° omega={omega:.2f}')
                self.publish_cmd(self.patrol_linear_speed, omega)
            return

        # ── Priority 2: WARNING ───────────────────────────────────────────────
        # Continuous centroid feedback — slows down and steers back inward.
        # Only applies in FORWARD/SCANNING; active manoeuvres run to completion.
        if geo == 'WARNING' and self._patrol_sub in (_P_FORWARD, _P_SCANNING):
            scale   = clamp(self.patrol_warning_speed_scale, 0.0, 1.0)
            yaw_err = wrap_to_pi(self._angle_to_centroid() - self._yaw_for_centroid())
            omega   = clamp(2.0 * yaw_err, -self.patrol_turn_speed, self.patrol_turn_speed)
            self.get_logger().debug(
                f'[PATROL] WARNING — centroid yaw_err={math.degrees(yaw_err):.1f}° omega={omega:.2f}')
            self.publish_cmd(self.patrol_linear_speed * scale, omega)
            return

        # ── TURNING phase (obstacle avoidance only — timed) ───────────────────
        if self._patrol_sub == _P_TURNING:
            if elapsed < self._patrol_sub_dur:
                self.publish_cmd(0.0, self.patrol_obstacle_angular * self._patrol_turn_dir)
            else:
                self.get_logger().info('[PATROL] Obstacle turn complete — force-forward')
                self._last_retreat_time = now
                self._enter_patrol_sub(_P_FORCE_FWD, self.patrol_force_fwd_sec)
            return

        # ── FORCE_FORWARD phase ────────────────────────────────────────────────
        if self._patrol_sub == _P_FORCE_FWD:
            if elapsed < self._patrol_sub_dur:
                self.publish_cmd(self.patrol_linear_speed, 0.0)
            else:
                self.get_logger().info('[PATROL] Force-forward done — straight patrol')
                dur = random.uniform(self.patrol_straight_min_sec, self.patrol_straight_max_sec)
                self._enter_patrol_sub(_P_FORWARD, dur)
            return

        # ── SCANNING phase ─────────────────────────────────────────────────────
        # Robot is stopped. Closed-loop spin to a random heading using AMCL.
        # Once aligned, pick a fresh straight duration and enter FORWARD.
        if self._patrol_sub == _P_SCANNING:
            if self._scan_target_yaw is None:
                # First tick of SCANNING — pick a random target heading
                self._scan_target_yaw = random.uniform(-math.pi, math.pi)
                self.get_logger().info(
                    f'[PATROL] SCANNING — turning to random heading '
                    f'{math.degrees(self._scan_target_yaw):.1f}°')

            yaw_err = wrap_to_pi(self._scan_target_yaw - self._yaw_for_centroid())
            if abs(yaw_err) < 0.10:
                self._scan_target_yaw = None
                dur = random.uniform(self.patrol_straight_min_sec, self.patrol_straight_max_sec)
                self.get_logger().info(
                    f'[PATROL] Heading locked — driving straight for {dur:.1f}s')
                self._enter_patrol_sub(_P_FORWARD, dur)
            else:
                omega = clamp(2.5 * yaw_err,
                              -self.patrol_obstacle_angular,
                               self.patrol_obstacle_angular)
                self.publish_cmd(0.0, omega)
            return

        # ── FORWARD phase ─────────────────────────────────────────────────────
        # Drive straight.  Two exit conditions:
        #   a) LIDAR sees an obstacle → TURNING (obstacle avoidance)
        #   b) Straight duration expires → SCANNING (pick new heading)
        # Geofence handled above (BREACH/WARNING at top of function).

        # (a) Obstacle check
        scan = self.latest_scan
        if scan is not None:
            front_min = _sector_min(scan, self.patrol_scan_start_deg, self.patrol_scan_end_deg)
            self.get_logger().debug(f'[PATROL] FORWARD front_min={front_min:.2f}m')

            if (front_min < self.patrol_safety_distance
                    and now - self._last_retreat_time > self.patrol_retreat_cooldown):
                turn_deg = random.uniform(self.patrol_turn_min_deg, self.patrol_turn_max_deg)
                turn_dur = (self.patrol_turn_sec_per_180 / 180.0) * turn_deg
                self._patrol_turn_dir = random.choice([-1.0, 1.0])
                self.get_logger().info(
                    f'[PATROL] Obstacle at {front_min:.2f}m — '
                    f'turning {"left" if self._patrol_turn_dir > 0 else "right"} '
                    f'{turn_deg:.0f}° for {turn_dur:.2f}s')
                self._enter_patrol_sub(_P_TURNING, turn_dur)
                self.publish_cmd(0.0, self.patrol_obstacle_angular * self._patrol_turn_dir)
                return

        # (b) Straight duration expired → scan and pick new heading
        if elapsed >= self._patrol_sub_dur:
            self.get_logger().info('[PATROL] Straight done — scanning for new heading')
            self._scan_target_yaw = None
            self._enter_patrol_sub(_P_SCANNING)
            self.publish_cmd(0.0, 0.0)
            return

        # Still going straight
        self.publish_cmd(self.patrol_linear_speed, 0.0)


    # ── chase helpers ─────────────────────────────────────────────────────────

    def _half_width(self) -> float:
        img_w = self.vision.get('image_width')
        if img_w is not None:
            try:
                hw = float(img_w) / 2.0
                if hw > 0.0:
                    return hw
            except (TypeError, ValueError):
                pass
        return self.center_x_px if self.center_x_px > 0.0 else 1.0

    def _rabbit_bbox_width(self) -> float | None:
        for key in ('rabbit_bbox_width', 'rabbit_width', 'rabbit_w', 'rabbit_box_width'):
            v = self.vision.get(key)
            if v is None:
                continue
            try:
                return float(v)
            except (TypeError, ValueError):
                continue
        bbox = self.vision.get('rabbit_bbox')
        if isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
            try:
                return float(bbox[2])
            except (TypeError, ValueError, IndexError):
                pass
        return None

    def _catch_in_cooldown(self) -> bool:
        return self.get_clock().now() < self._catch_cooldown_until

    def _check_catch_condition(self) -> bool:
        if self.catch_bbox_width_px <= 0.0:
            return False
        if self._catch_in_cooldown():
            self._catch_confirm_count = 0
            return False
        if not self.rabbit_visible():
            self._catch_confirm_count = 0
            return False
        bbox_w = self._rabbit_bbox_width()
        if bbox_w is None:
            self._catch_confirm_count = 0
            return False
        if bbox_w >= self.catch_bbox_width_px:
            self._catch_confirm_count += 1
        else:
            self._catch_confirm_count = 0
        return self._catch_confirm_count >= max(1, self.catch_confirm_frames)

    def _fuse_fresh_vision_if_any(self) -> bool:
        if not self.rabbit_visible():
            return False
        stamp = self.vision.get('stamp')
        if stamp is None or stamp == self._last_vision_stamp:
            return False
        try:
            cx = float(self.vision.get('rabbit_center_x', self.center_x_px))
        except (TypeError, ValueError):
            return False
        conf = 1.0
        conf_raw = self.vision.get('rabbit_confidence')
        if conf_raw is not None:
            try:
                conf = float(conf_raw)
            except (TypeError, ValueError):
                conf = 1.0
        self.kf.update(cx, confidence=conf)
        self._last_vision_stamp = stamp
        return True

    # ── main loop ─────────────────────────────────────────────────────────────

    def step(self) -> None:
        now = self.get_clock().now()
        dt  = (now - self._last_step_time).nanoseconds / 1e9
        self._last_step_time = now
        if dt <= 0.0:
            dt = 1.0 / max(1.0, self.control_hz)

        rabbit_alive            = _bool(self.game_state.get('rabbit_alive', True))
        rabbit_escaped          = _bool(self.game_state.get('rabbit_escaped', False))
        rabbit_inside_territory = _bool(self.rabbit_geofence.get('inside_wolf_territory', True))

        if not rabbit_alive:
            self.enter_state('STOP')

        if self.state == 'STOP':
            self.publish_cmd(0.0, 0.0)
            return

        if self.state == 'STOP_CAUGHT':
            self.publish_cmd(0.0, 0.0)
            if self.seconds_in_state() >= self.catch_pause_sec:
                self.get_logger().info(
                    f'[CAUGHT] Pause done ({self.catch_pause_sec:.1f}s) — back to PATROL')
                self.enter_state('PATROL')
            return

        if self.state == 'PATROL' and self.rabbit_visible() and rabbit_inside_territory and not rabbit_escaped:
            self.enter_state('CHASE')

        if self.state == 'CHASE' and (rabbit_escaped or not rabbit_inside_territory):
            self.enter_state('RETURN_TURN')

        if self.state == 'PATROL':
            self.patrol_step()
            return

        # ── CHASE ─────────────────────────────────────────────────────────────
        if self.state == 'CHASE':
            geo = self.wolf_geofence_state
            if geo == 'BREACH':
                self.get_logger().warn('[CHASE] BREACH — aborting chase, entering RETURN_TURN')
                self.enter_state('RETURN_TURN')
                return

            if self._check_catch_condition():
                bbox_w = self._rabbit_bbox_width()
                self.get_logger().info(
                    f'[CHASE] Rabbit caught! bbox_width={bbox_w:.1f}px '
                    f'>= threshold {self.catch_bbox_width_px:.1f}px — '
                    f'pausing for {self.catch_pause_sec:.1f}s')
                self.enter_state('STOP_CAUGHT')
                self.publish_cmd(0.0, 0.0)
                return

            self.kf.predict(dt, self._last_cmd_omega * self.kf_sign)
            fused = self._fuse_fresh_vision_if_any()

            if not self.vision_fresh():
                self._lost_frame_count += 1
            elif fused:
                self._lost_frame_count = 0

            if self._lost_frame_count >= self.chase_lost_frames:
                self.get_logger().info(
                    f'[CHASE] Rabbit lost ({self._lost_frame_count} stale frames) — back to PATROL')
                self.enter_state('PATROL')
                self.patrol_step()
                return

            if not self.kf.initialized:
                self.publish_cmd(0.0, 0.0)
                return

            predicted_px = self.kf.px + self.kf.vx * self.kf_feedforward_sec
            half_w = self._half_width()
            error = clamp((predicted_px - self.center_x_px) / half_w, -1.5, 1.5)
            if abs(error) < self.chase_deadband:
                error = 0.0

            omega = -self.chase_pid.compute(error, dt)

            if self.chase_speed_scale and self.max_angular_speed > 0.0:
                turn_ratio = abs(omega) / self.max_angular_speed
                linear = self.chase_linear_speed * (1.0 - 0.5 * turn_ratio)
            else:
                linear = self.chase_linear_speed

            if geo == 'WARNING':
                linear *= clamp(self.chase_warning_speed_scale, 0.0, 1.0)

            self.publish_cmd(linear, omega)
            bbox_w   = self._rabbit_bbox_width()
            bbox_str = f'{bbox_w:.1f}' if bbox_w is not None else '---'
            cd_str   = ' COOLDOWN' if self._catch_in_cooldown() else ''
            self.get_logger().debug(
                f'[CHASE] kf_px={self.kf.px:6.1f}  pred={predicted_px:6.1f}  '
                f'err={error:+.3f}  omega={omega:+.3f}  linear={linear:.3f}  '
                f'bbox_w={bbox_str}  geo={geo}{cd_str}  fused={fused}')
            return

        # ── RETURN_TURN ────────────────────────────────────────────────────────
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
