#!/usr/bin/env python3
"""
TurtleBot4 Geofence Node
========================
Subscribes to /amcl_pose, checks the robot's position against a configured
polygon boundary, and publishes status messages only.

This node does NOT publish cmd_vel — it is purely an observer/reporter.
Other nodes (e.g. WolfFSM) subscribe to the status topic and decide how
to react to boundary events.

Status topic (default: /wolf/geofence)
───────────────────────────────────────
Publishes a plain string every control loop tick:

  "SAFE"     — robot is inside the zone and far from the boundary
  "WARNING"  — robot is inside but within warning_distance of the edge
  "BREACH"   — robot has crossed outside the polygon

Also publishes a verbose key=value string on /wolf/geofence/detail:
  outside_wolf_territory=False,near_territory_boundary=True,state=WARNING,dist=0.38
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


# ── helpers ───────────────────────────────────────────────────────────────────

def point_in_polygon(px, py, polygon):
    """
    Ray-casting point-in-polygon test.
    polygon: list of (x, y) tuples.
    Returns True when (px, py) is inside.
    """
    n = len(polygon)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > py) != (yj > py)) and \
                (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def closest_point_on_segment(px, py, ax, ay, bx, by):
    """Returns the closest point on segment AB to P, plus the distance."""
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-12:
        return ax, ay, math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
    cx, cy = ax + t * dx, ay + t * dy
    return cx, cy, math.hypot(px - cx, py - cy)


def nearest_boundary_distance(px, py, polygon):
    """Returns the distance to the nearest polygon edge."""
    best_dist = float('inf')
    n = len(polygon)
    for i in range(n):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % n]
        _, _, dist = closest_point_on_segment(px, py, ax, ay, bx, by)
        if dist < best_dist:
            best_dist = dist
    return best_dist


# ── state constants ───────────────────────────────────────────────────────────

STATE_SAFE    = 'SAFE'
STATE_WARNING = 'WARNING'
STATE_BREACH  = 'BREACH'


# ── node ──────────────────────────────────────────────────────────────────────

class GeofenceNode(Node):

    def __init__(self):
        super().__init__('geofence_node')

        # ── parameters ───────────────────────────────────────────────────────
        self.declare_parameter('zone_polygon',
            [-2.0, -2.0,
              2.0, -2.0,
              2.0,  2.0,
             -2.0,  2.0])
        self.declare_parameter('warning_distance', 0.5)   # metres
        self.declare_parameter('publish_rate',     10.0)  # Hz
        self.declare_parameter('status_topic',     '/wolf/geofence')
        self.declare_parameter('detail_topic',     '/wolf/geofence/detail')

        # ── load polygon ──────────────────────────────────────────────────────
        flat = self.get_parameter('zone_polygon').value
        if len(flat) < 6 or len(flat) % 2 != 0:
            self.get_logger().error(
                'zone_polygon must have an even number of values (≥6). '
                'Using default square.')
            flat = [-2.0, -2.0, 2.0, -2.0, 2.0, 2.0, -2.0, 2.0]

        self.polygon = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]
        self.get_logger().info(
            f'Geofence polygon loaded: {len(self.polygon)} vertices')

        self.warning_distance = self.get_parameter('warning_distance').value

        # ── internal state ────────────────────────────────────────────────────
        self.state         = STATE_SAFE
        self.robot_x       = 0.0
        self.robot_y       = 0.0
        self.pose_received = False

        # ── QoS ──────────────────────────────────────────────────────────────
        amcl_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # ── subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            amcl_qos)

        # ── publishers ────────────────────────────────────────────────────────
        # Simple state string: "SAFE" / "WARNING" / "BREACH"
        status_topic = self.get_parameter('status_topic').value
        self.status_pub = self.create_publisher(String, status_topic, 10)

        # Verbose detail string with distance info
        detail_topic = self.get_parameter('detail_topic').value
        self.detail_pub = self.create_publisher(String, detail_topic, 10)

        # RViz boundary visualisation
        self.marker_pub = self.create_publisher(
            MarkerArray, '/geofence/boundary', 10)

        self.get_logger().info(f'Status topic  : {status_topic}')
        self.get_logger().info(f'Detail topic  : {detail_topic}')

        # ── timer ─────────────────────────────────────────────────────────────
        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info('GeofenceNode started. Waiting for /amcl_pose …')

    # ── pose callback ─────────────────────────────────────────────────────────

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.pose_received = True

    # ── control loop ──────────────────────────────────────────────────────────

    def control_loop(self):
        if not self.pose_received:
            return

        px, py = self.robot_x, self.robot_y
        inside  = point_in_polygon(px, py, self.polygon)
        dist    = nearest_boundary_distance(px, py, self.polygon)

        # ── classify state ────────────────────────────────────────────────────
        if inside and dist > self.warning_distance:
            new_state = STATE_SAFE
        elif inside and dist <= self.warning_distance:
            new_state = STATE_WARNING
        else:
            new_state = STATE_BREACH

        # Log state transitions
        if new_state != self.state:
            self.get_logger().warn(
                f'Geofence state: {self.state} → {new_state} '
                f'| pos=({px:.2f},{py:.2f}) dist={dist:.2f}m')
        self.state = new_state

        # ── publish simple status ─────────────────────────────────────────────
        # Publishes exactly: "SAFE", "WARNING", or "BREACH"
        self.status_pub.publish(String(data=self.state))

        # ── publish detailed status ───────────────────────────────────────────
        # Compatible with string_msg_to_dict() in WolfFSM
        detail = (
            f"outside_wolf_territory={self.state == STATE_BREACH},"
            f"near_territory_boundary={self.state == STATE_WARNING},"
            f"state={self.state},"
            f"dist={dist:.2f}"
        )
        self.detail_pub.publish(String(data=detail))

        # ── RViz markers ──────────────────────────────────────────────────────
        self._publish_markers()

    # ── RViz visualisation ────────────────────────────────────────────────────

    def _publish_markers(self):
        now      = self.get_clock().now().to_msg()
        lifetime = Duration(sec=1, nanosec=0)

        color = {
            STATE_SAFE:    ColorRGBA(r=0.0, g=0.8, b=0.2, a=0.8),
            STATE_WARNING: ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.9),
            STATE_BREACH:  ColorRGBA(r=0.9, g=0.1, b=0.1, a=1.0),
        }[self.state]

        # Boundary line strip
        boundary          = Marker()
        boundary.header   = Header(stamp=now, frame_id='map')
        boundary.ns       = 'geofence'
        boundary.id       = 0
        boundary.type     = Marker.LINE_STRIP
        boundary.action   = Marker.ADD
        boundary.scale.x  = 0.05
        boundary.color    = color
        boundary.lifetime = lifetime

        for vx, vy in self.polygon:
            boundary.points.append(Point(x=vx, y=vy, z=0.0))
        boundary.points.append(
            Point(x=self.polygon[0][0], y=self.polygon[0][1], z=0.0))

        # Robot position sphere
        robot          = Marker()
        robot.header   = Header(stamp=now, frame_id='map')
        robot.ns       = 'geofence'
        robot.id       = 1
        robot.type     = Marker.SPHERE
        robot.action   = Marker.ADD
        robot.pose.position.x = self.robot_x
        robot.pose.position.y = self.robot_y
        robot.pose.position.z = 0.1
        robot.scale.x  = 0.2
        robot.scale.y  = 0.2
        robot.scale.z  = 0.2
        robot.color    = color
        robot.lifetime = lifetime

        arr          = MarkerArray()
        arr.markers  = [boundary, robot]
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
