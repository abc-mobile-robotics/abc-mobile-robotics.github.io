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
        self.d_filter_alpha = d_filter_alpha   # 0 = no smoothing, ~0.99 = frozen

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

        # Exponential LPF on derivative — kills YOLO jitter
        a = clamp(self.d_filter_alpha, 0.0, 0.99)
        self._d_filtered = a * self._d_filtered + (1.0 - a) * derivative_raw

        self._integral += error * dt
        self._integral = clamp(self._integral, -self.integral_limit, self.integral_limit)

        output = self.kp * error + self.ki * self._integral + self.kd * self._d_filtered
        self._prev_error = error

        return clamp(output, -self.output_limit, self.output_limit)


class RabbitPixelKF:
    """Constant-velocity Kalman filter for the rabbit's x-position in the image.

    State:  x = [px, vx]^T       (pixels, pixels/sec)
    Control input u = omega       (rad/s) — robot's commanded angular velocity.
        Positive omega (turning left by ROS convention) shifts the rabbit RIGHT
        in the image, so we subtract omega * pixels_per_rad from px during
        prediction. If your camera is mounted reversed, flip kf_sign to -1.

    Prediction (runs every control tick, dt ~ 0.02 s at 50 Hz):
        px'  = px + vx * dt - omega * dt * pixels_per_rad
        vx'  = vx
        P'   = F P F^T + Q,   Q scaled by dt

    Update (runs when a fresh YOLO frame arrives, ~ every 0.1 s):
        z   = rabbit_center_x
        R   = measurement variance, inflated when confidence is low
        standard scalar Kalman gain / state / covariance update
    """

    def __init__(self,
                 pixels_per_rad: float,
                 process_px_var: float,
                 process_vx_var: float,
                 meas_var: float) -> None:
        self.pixels_per_rad  = pixels_per_rad
        self.process_px_var  = process_px_var   # (px)^2 per sec — position noise rate
        self.process_vx_var  = process_vx_var   # (px/s)^2 per sec — velocity noise rate
        self.meas_var        = meas_var         # (px)^2 at confidence = 1.0

        # State [px, vx], covariance P as flat [p00, p01, p10, p11]
        self._x = [0.0, 0.0]
        self._P = [1e6, 0.0, 0.0, 1e6]
        self.initialized = False

    def reset(self) -> None:
        self._x = [0.0, 0.0]
        self._P = [1e6, 0.0, 0.0, 1e6]
        self.initialized = False

    def set_pixels_per_rad(self, ppr: float) -> None:
        if ppr > 0.0:
            self.pixels_per_rad = ppr

    def predict(self, dt: float, omega_cmd: float) -> None:
        if dt <= 0.0 or not self.initialized:
            return
        px, vx = self._x
        px_new = px + vx * dt - omega_cmd * dt * self.pixels_per_rad
        vx_new = vx
        self._x = [px_new, vx_new]

        # P' = F P F^T + Q,  with F = [[1, dt],[0, 1]]
        p00, p01, p10, p11 = self._P
        # F P
        a00 = p00 + dt * p10
        a01 = p01 + dt * p11
        a10 = p10
        a11 = p11
        # (F P) F^T
        n00 = a00 + a01 * dt
        n01 = a01
        n10 = a10 + a11 * dt
        n11 = a11
        # + Q (scaled by dt so tuning is dt-independent)
        n00 += self.process_px_var * dt
        n11 += self.process_vx_var * dt
        self._P = [n00, n01, n10, n11]

    def update(self, measurement_px: float, confidence: float = 1.0) -> None:
        conf = max(0.1, min(1.0, float(confidence)))
        R = self.meas_var / (conf * conf)

        if not self.initialized:
            # First detection — seed position directly, leave velocity at 0 with
            # moderate uncertainty so the filter can learn it from subsequent frames.
            self._x = [measurement_px, 0.0]
            self._P = [R, 0.0, 0.0, 1e4]
            self.initialized = True
            return

        # H = [1, 0], innovation y = z - px
        px, vx = self._x
        y = measurement_px - px
        p00, p01, p10, p11 = self._P
        S = p00 + R
        if S <= 0.0:
            return
        k0 = p00 / S
        k1 = p10 / S
        self._x = [px + k0 * y, vx + k1 * y]
        # P = (I - K H) P, with I - KH = [[1-k0, 0], [-k1, 1]]
        n00 = (1.0 - k0) * p00
        n01 = (1.0 - k0) * p01
        n10 = -k1 * p00 + p10
        n11 = -k1 * p01 + p11
        self._P = [n00, n01, n10, n11]

    @property
    def px(self) -> float:
        return self._x[0]

    @property
    def vx(self) -> float:
        return self._x[1]

    @property
    def px_variance(self) -> float:
        return self._P[0]


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
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('return_turn_angle_deg', 160.0)
        # PID gains — error normalized to [-1, +1] across the frame.
        self.declare_parameter('chase_kp', 2.0)
        self.declare_parameter('chase_ki', 0.0)
        self.declare_parameter('chase_kd', 0.15)
        self.declare_parameter('chase_deadband', 0.02)
        self.declare_parameter('chase_speed_scale_on_turn', True)
        self.declare_parameter('chase_lost_frames', 5)
        # Catch condition — when the rabbit's bbox width crosses this threshold
        # (in pixels), the rabbit is "caught" and we transition to STOP_CAUGHT.
        # The bbox grows as the wolf gets closer, so this is a proxy for range.
        # Set to <= 0 to disable the catch trigger entirely.
        self.declare_parameter('catch_bbox_width_px', 65.0)
        # How many consecutive frames the bbox must exceed the threshold before
        # we commit. 1 = trigger immediately, higher = more robust to YOLO
        # bbox jitter that briefly inflates a single frame.
        self.declare_parameter('catch_confirm_frames', 2)
        # How long (seconds) to sit still in STOP_CAUGHT before resuming PATROL.
        self.declare_parameter('catch_pause_sec', 2.0)
        # After leaving STOP_CAUGHT, suppress catch detection for this long.
        # Without this, the bbox is probably still over threshold when we
        # resume CHASE → we'd immediately bounce right back to STOP_CAUGHT.
        # Should be long enough for you to physically move the rabbit away.
        self.declare_parameter('catch_cooldown_sec', 4.0)
        # Kalman filter params
        #   pixels_per_rad: how many image pixels correspond to 1 rad of robot yaw.
        #     For a ~60° HFOV camera at 250 px wide, ≈ 250 / (π/3) ≈ 240.
        #     Measure it: start stationary, centre the target, rotate a known
        #     amount, see how far the target moved in the image.
        self.declare_parameter('kf_pixels_per_rad', 240.0)
        self.declare_parameter('kf_process_px_var', 50.0)     # (px)^2/s
        self.declare_parameter('kf_process_vx_var', 2500.0)   # (px/s)^2/s
        self.declare_parameter('kf_meas_var', 25.0)           # (px)^2 at conf=1.0
        self.declare_parameter('kf_feedforward_sec', 0.15)    # lag compensation
        self.declare_parameter('kf_sign', 1.0)                # -1 if camera inverted
        # Control loop rate — run faster than vision so the KF can fill the gap.
        self.declare_parameter('control_hz', 50.0)
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
        self.catch_bbox_width_px   = float(self.get_parameter('catch_bbox_width_px').value)
        self.catch_confirm_frames  = int(self.get_parameter('catch_confirm_frames').value)
        self.catch_pause_sec       = float(self.get_parameter('catch_pause_sec').value)
        self.catch_cooldown_sec    = float(self.get_parameter('catch_cooldown_sec').value)
        self.kf_feedforward_sec    = float(self.get_parameter('kf_feedforward_sec').value)
        self.kf_sign               = float(self.get_parameter('kf_sign').value)
        self.control_hz            = float(self.get_parameter('control_hz').value)

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

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(String,   self.vision_topic,          self.vision_callback,          10)
        self.create_subscription(String,   self.wolf_geofence_topic,   self.wolf_geofence_callback,   10)
        self.create_subscription(String,   self.rabbit_geofence_topic, self.rabbit_geofence_callback, 10)
        self.create_subscription(String,   self.game_state_topic,      self.game_state_callback,      10)
        self.create_subscription(Odometry, self.odom_topic,            self.odom_callback,            10)

        control_period = 1.0 / max(1.0, self.control_hz)
        self.timer = self.create_timer(control_period, self.step)

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

        # Last commanded omega — fed into KF predict() as control input
        self._last_cmd_omega = 0.0
        # Stamp of last vision msg we fused, so we only fuse each frame once
        self._last_vision_stamp: float | None = None
        # Catch-condition confirm counter (consecutive frames over threshold)
        self._catch_confirm_count = 0
        # Time we last left STOP_CAUGHT — catch detection is suppressed for
        # catch_cooldown_sec after this so we don't immediately re-trigger.
        self._catch_cooldown_until = self.get_clock().now()

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
            elif name == 'catch_bbox_width_px':
                self.catch_bbox_width_px = float(val)
            elif name == 'catch_confirm_frames':
                self.catch_confirm_frames = int(val)
            elif name == 'catch_pause_sec':
                self.catch_pause_sec = float(val)
            elif name == 'catch_cooldown_sec':
                self.catch_cooldown_sec = float(val)
            elif name == 'vision_timeout_sec':
                self.vision_timeout_sec = float(val)
            elif name == 'center_x_px':
                self.center_x_px = float(val)
            elif name == 'kf_pixels_per_rad':
                self.kf.set_pixels_per_rad(float(val))
            elif name == 'kf_process_px_var':
                self.kf.process_px_var = float(val)
            elif name == 'kf_process_vx_var':
                self.kf.process_vx_var = float(val)
            elif name == 'kf_meas_var':
                self.kf.meas_var = float(val)
            elif name == 'kf_feedforward_sec':
                self.kf_feedforward_sec = float(val)
            elif name == 'kf_sign':
                self.kf_sign = float(val)
            self.get_logger().info(f'Param updated: {name} = {val}')
        return SetParametersResult(successful=True)

    # ── helpers ───────────────────────────────────────────────────────────────

    def enter_state(self, new_state: str) -> None:
        if self.state != new_state:
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
                self._lost_frame_count = 0
                self._last_vision_stamp = None
                self._last_cmd_omega = 0.0
                self._catch_confirm_count = 0
            # When LEAVING STOP_CAUGHT, arm a cooldown so we don't re-trigger
            # the catch condition immediately on the same big bbox.
            if prev_state == 'STOP_CAUGHT':
                cooldown_ns = int(max(0.0, self.catch_cooldown_sec) * 1e9)
                self._catch_cooldown_until = (
                    self.get_clock().now()
                    + rclpy.duration.Duration(nanoseconds=cooldown_ns)
                )
                self._catch_confirm_count = 0
                self.get_logger().info(
                    f'Catch cooldown armed for {self.catch_cooldown_sec:.1f}s')
            self.get_logger().info(f'Wolf state -> {new_state}')

    def publish_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x  = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)
        # Remember commanded omega — next KF predict() uses it as control input
        self._last_cmd_omega = float(angular_z)

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
            #self.publish_cmd(0.0, self.patrol_turn_speed)
            pass
        else:
            #self.publish_cmd(self.patrol_linear_speed, 0.18)
            pass

    def _half_width(self) -> float:
        """Half of the image width, used to normalize pixel error to [-1, +1]."""
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
        """Return the rabbit bbox width in pixels, or None if unavailable.

        Tries several common key names so this works whether the YOLO publisher
        writes 'rabbit_bbox_width', 'rabbit_width', or includes a [x,y,w,h] list.
        """
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
        """True if catch detection is currently suppressed by post-release cooldown."""
        return self.get_clock().now() < self._catch_cooldown_until

    def _check_catch_condition(self) -> bool:
        """Update the catch-confirm counter based on current bbox width.

        Returns True when the threshold has been exceeded for
        `catch_confirm_frames` consecutive fresh frames AND we're not in
        the post-release cooldown window.
        """
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
        """If a new vision frame has arrived since last call, fuse it into the KF.
        Returns True if a fresh frame was consumed."""
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

        # STOP is still terminal — only enters if rabbit_alive=False.
        if self.state == 'STOP':
            self.publish_cmd(0.0, 0.0)
            return

        # STOP_CAUGHT is a brief pause, then back to PATROL.
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

        if self.state == 'CHASE':
            # 0. Catch check FIRST — if we're already close enough, stop now.
            if self._check_catch_condition():
                bbox_w = self._rabbit_bbox_width()
                self.get_logger().info(
                    f'[CHASE] Rabbit caught! bbox_width={bbox_w:.1f}px '
                    f'>= threshold {self.catch_bbox_width_px:.1f}px '
                    f'(confirmed over {self._catch_confirm_count} frames) — '
                    f'pausing for {self.catch_pause_sec:.1f}s')
                self.enter_state('STOP_CAUGHT')
                self.publish_cmd(0.0, 0.0)
                return

            # 1. Predict KF forward using last commanded omega as control input.
            self.kf.predict(dt, self._last_cmd_omega * self.kf_sign)

            # 2. Fuse fresh YOLO detection if one arrived since last tick.
            fused = self._fuse_fresh_vision_if_any()

            # 3. Track "lost" frames only when vision itself goes stale.
            if not self.vision_fresh():
                self._lost_frame_count += 1
            elif fused:
                self._lost_frame_count = 0

            if self._lost_frame_count >= self.chase_lost_frames:
                self.get_logger().info(
                    f'[CHASE] Rabbit lost ({self._lost_frame_count} stale frames) '
                    '— back to PATROL')
                self.enter_state('PATROL')
                self.patrol_step()
                return

            if not self.kf.initialized:
                self.publish_cmd(0.0, 0.0)
                return

            # 4. Build control error from KF estimate + velocity feedforward.
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

            self.publish_cmd(linear, omega)
            bbox_w = self._rabbit_bbox_width()
            bbox_str = f'{bbox_w:.1f}' if bbox_w is not None else '---'
            cd_str = ' COOLDOWN' if self._catch_in_cooldown() else ''
            self.get_logger().debug(
                f'[CHASE] kf_px={self.kf.px:6.1f}  kf_vx={self.kf.vx:+7.1f}  '
                f'pred={predicted_px:6.1f}  err={error:+.3f}  '
                f'omega={omega:+.3f}  linear={linear:.3f}  '
                f'bbox_w={bbox_str}  catch_cnt={self._catch_confirm_count}{cd_str}  '
                f'fused={fused}')
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
