#!/usr/bin/env python3
"""
TurtleBot4 Geofence Node
========================
Subscribes to /amcl_pose, checks the robot's position against a configured
polygon boundary, and publishes corrective /cmd_vel commands when the robot
approaches or breaches the boundary.

Zone states:
  SAFE    — robot is inside and far from boundary → pass through
  WARNING — robot is within `warning_distance` meters → slow + steer inward
  BREACH  — robot has crossed boundary → full override, drive back inside
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from builtin_interfaces.msg import Duration


# ── helpers ──────────────────────────────────────────────────────────────────

def cross2d(ox, oy, ax, ay, bx, by):
    """2-D cross product of vectors OA and OB."""
    return (ax - ox) * (by - oy) - (ay - oy) * (bx - ox)


def point_in_polygon(px, py, polygon):
    """
    Ray-casting point-in-polygon test.
    polygon: list of (x, y) tuples (open polygon — first != last).
    Returns True when (px, py) is inside.
    """
    n = len(polygon)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def closest_point_on_segment(px, py, ax, ay, bx, by):
    """
    Returns the closest point on segment AB to P, plus the distance.
    """
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-12:
        return ax, ay, math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
    cx, cy = ax + t * dx, ay + t * dy
    return cx, cy, math.hypot(px - cx, py - cy)


def nearest_boundary_point(px, py, polygon):
    """
    Finds the closest point on any polygon edge to (px, py).
    Returns (cx, cy, distance, inward_nx, inward_ny) where (inward_nx, inward_ny)
    is the unit normal pointing INTO the polygon from that edge.
    """
    best_dist = float('inf')
    best_cx, best_cy = px, py
    best_nx, best_ny = 0.0, 0.0

    n = len(polygon)
    for i in range(n):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % n]
        cx, cy, dist = closest_point_on_segment(px, py, ax, ay, bx, by)

        if dist < best_dist:
            best_dist = dist
            best_cx, best_cy = cx, cy

            # Edge normal: rotate edge vector 90° left → points left of travel direction.
            # We need the normal that points INWARD (toward polygon interior).
            ex, ey = bx - ax, by - ay  # edge direction
            nx, ny = -ey, ex           # left-hand normal (candidate inward)
            mag = math.hypot(nx, ny)
            if mag > 1e-9:
                nx, ny = nx / mag, ny / mag

            # Flip if this normal points away from a centroid approximation.
            cx_poly = sum(v[0] for v in polygon) / n
            cy_poly = sum(v[1] for v in polygon) / n
            if nx * (cx_poly - best_cx) + ny * (cy_poly - best_cy) < 0:
                nx, ny = -nx, -ny

            best_nx, best_ny = nx, ny

    return best_cx, best_cy, best_dist, best_nx, best_ny


# ── state constants ───────────────────────────────────────────────────────────

STATE_SAFE    = 'SAFE'
STATE_WARNING = 'WARNING'
STATE_BREACH  = 'BREACH'


# ── node ─────────────────────────────────────────────────────────────────────

class GeofenceNode(Node):

    def __init__(self):
        super().__init__('geofence_node')

        # ── parameters ───────────────────────────────────────────────────────
        self.declare_parameter('zone_polygon',
            # Default: a 4×4 m square centred at the origin.
            # Replace with your actual map coordinates.
            [-2.0, -2.0,
              2.0, -2.0,
              2.0,  2.0,
             -2.0,  2.0])

        self.declare_parameter('warning_distance', 0.5)   # metres
        self.declare_parameter('max_linear_speed',  0.3)  # m/s
        self.declare_parameter('max_angular_speed', 0.8)  # rad/s
        self.declare_parameter('correction_gain',   1.2)  # proportional gain
        self.declare_parameter('publish_rate',     10.0)  # Hz

        # ── load polygon ─────────────────────────────────────────────────────
        flat = self.get_parameter('zone_polygon').value
        if len(flat) < 6 or len(flat) % 2 != 0:
            self.get_logger().error(
                'zone_polygon must have an even number of values (≥6). '
                'Using default square.')
            flat = [-2.0, -2.0, 2.0, -2.0, 2.0, 2.0, -2.0, 2.0]

        self.polygon = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]
        self.get_logger().info(
            f'Geofence polygon loaded: {len(self.polygon)} vertices')

        self.warning_distance  = self.get_parameter('warning_distance').value
        self.max_linear_speed  = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.correction_gain   = self.get_parameter('correction_gain').value

        # ── state ─────────────────────────────────────────────────────────────
        self.state    = STATE_SAFE
        self.robot_x  = 0.0
        self.robot_y  = 0.0
        self.robot_yaw = 0.0
        self.pose_received = False

        # ── QoS: AMCL publishes with TRANSIENT_LOCAL / RELIABLE ──────────────
        amcl_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # ── subscribers ───────────────────────────────────────────────────────
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            amcl_qos)

        # ── publishers ────────────────────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub  = self.create_publisher(MarkerArray, '/geofence/boundary', 10)

        # ── timer ─────────────────────────────────────────────────────────────
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info('GeofenceNode started. Waiting for /amcl_pose …')

    # ── pose callback ─────────────────────────────────────────────────────────

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        /amcl_pose message structure
        ─────────────────────────────
        msg.header.stamp          : builtin_interfaces/Time
        msg.header.frame_id       : str  (usually 'map')

        msg.pose.pose.position.x  : float  ← robot X in map frame
        msg.pose.pose.position.y  : float  ← robot Y in map frame
        msg.pose.pose.position.z  : float  (always ~0 for ground robots)

        msg.pose.pose.orientation : geometry_msgs/Quaternion
            .x, .y, .z, .w       ← use to extract yaw

        msg.pose.covariance       : float[36]  ← 6×6 covariance matrix
            [0]  = variance in X
            [7]  = variance in Y
            [35] = variance in YAW
            (useful for confidence checks — skip if covariance is too high)
        """
        p = msg.pose.pose

        self.robot_x = p.position.x
        self.robot_y = p.position.y

        # Extract yaw from quaternion
        q = p.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.pose_received = True

    # ── control loop ──────────────────────────────────────────────────────────

    def control_loop(self):
        if not self.pose_received:
            return

        px, py = self.robot_x, self.robot_y
        inside = point_in_polygon(px, py, self.polygon)
        _, _, dist, nx, ny = nearest_boundary_point(px, py, self.polygon)

        # ── state machine ────────────────────────────────────────────────────
        if inside and dist > self.warning_distance:
            new_state = STATE_SAFE
        elif inside and dist <= self.warning_distance:
            new_state = STATE_WARNING
        else:
            new_state = STATE_BREACH

        if new_state != self.state:
            self.get_logger().warn(
                f'Geofence state: {self.state} → {new_state} '
                f'| pos=({px:.2f},{py:.2f}) dist={dist:.2f}m')
        self.state = new_state

        # ── publish velocity ─────────────────────────────────────────────────
        cmd = Twist()

        if self.state == STATE_SAFE:
            pass  # geofence does nothing — nav2 drives as normal

        elif self.state == STATE_WARNING:
            # Scale speed down as robot approaches boundary.
            # At warning_distance → full speed, at 0 → stopped + correcting.
            ratio = dist / self.warning_distance   # 0..1
            speed_scale = ratio ** 2               # quadratic fade

            # Inward velocity component
            vx = nx * self.max_linear_speed * (1.0 - ratio) * self.correction_gain
            vy = ny * self.max_linear_speed * (1.0 - ratio) * self.correction_gain

            # Project inward direction onto robot heading for forward/angular
            fwd   =  vx * math.cos(self.robot_yaw) + vy * math.sin(self.robot_yaw)
            strafe= -vx * math.sin(self.robot_yaw) + vy * math.cos(self.robot_yaw)

            cmd.linear.x  = max(-self.max_linear_speed,
                                min(self.max_linear_speed, fwd))
            cmd.angular.z = max(-self.max_angular_speed,
                                min(self.max_angular_speed,
                                    strafe * self.correction_gain))

            self.cmd_vel_pub.publish(cmd)

        elif self.state == STATE_BREACH:
            # Full override: drive straight toward the nearest interior point.
            vx = nx * self.max_linear_speed * self.correction_gain
            vy = ny * self.max_linear_speed * self.correction_gain

            fwd    =  vx * math.cos(self.robot_yaw) + vy * math.sin(self.robot_yaw)
            strafe = -vx * math.sin(self.robot_yaw) + vy * math.cos(self.robot_yaw)

            cmd.linear.x  = max(-self.max_linear_speed,
                                min(self.max_linear_speed, fwd))
            cmd.angular.z = max(-self.max_angular_speed,
                                min(self.max_angular_speed,
                                    strafe * self.correction_gain * 2.0))

            self.cmd_vel_pub.publish(cmd)

        # ── publish RViz markers ──────────────────────────────────────────────
        self._publish_markers()

    # ── RViz visualisation ────────────────────────────────────────────────────

    def _publish_markers(self):
        """Publishes the polygon boundary + robot state as RViz markers."""
        now = self.get_clock().now().to_msg()
        lifetime = Duration(sec=1, nanosec=0)

        # Colour by state
        state_color = {
            STATE_SAFE:    ColorRGBA(r=0.0, g=0.8, b=0.2, a=0.8),
            STATE_WARNING: ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.9),
            STATE_BREACH:  ColorRGBA(r=0.9, g=0.1, b=0.1, a=1.0),
        }
        color = state_color[self.state]

        # --- boundary line strip ---
        boundary = Marker()
        boundary.header = Header(stamp=now, frame_id='map')
        boundary.ns     = 'geofence'
        boundary.id     = 0
        boundary.type   = Marker.LINE_STRIP
        boundary.action = Marker.ADD
        boundary.scale.x = 0.05
        boundary.color   = color
        boundary.lifetime = lifetime

        for vx, vy in self.polygon:
            boundary.points.append(Point(x=vx, y=vy, z=0.0))
        # close the loop
        boundary.points.append(Point(x=self.polygon[0][0],
                                     y=self.polygon[0][1], z=0.0))

        # --- robot position sphere ---
        robot_marker = Marker()
        robot_marker.header   = Header(stamp=now, frame_id='map')
        robot_marker.ns       = 'geofence'
        robot_marker.id       = 1
        robot_marker.type     = Marker.SPHERE
        robot_marker.action   = Marker.ADD
        robot_marker.pose.position.x = self.robot_x
        robot_marker.pose.position.y = self.robot_y
        robot_marker.pose.position.z = 0.1
        robot_marker.scale.x  = 0.2
        robot_marker.scale.y  = 0.2
        robot_marker.scale.z  = 0.2
        robot_marker.color    = color
        robot_marker.lifetime = lifetime

        arr = MarkerArray()
        arr.markers = [boundary, robot_marker]
        self.marker_pub.publish(arr)


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = GeofenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
