#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String


class RabbitChaser(Node):
    def __init__(self):
        super().__init__('rabbit_chaser')

        self.declare_parameter('robot_namespace', 'robot04')
        ns = self.get_parameter('robot_namespace').get_parameter_value().string_value.strip('/')
        prefix = f'/{ns}' if ns else ''

        self.cmd_pub = self.create_publisher(Twist, f'{prefix}/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, f'{prefix}/robot_status', 10)

        self.image_sub = self.create_subscription(Image, f'{prefix}/camera/image_raw', self.image_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, f'{prefix}/scan', self.scan_cb, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.state = 'SEARCH'
        self.rabbit_visible = False
        self.last_seen_time = 0.0
        self.error_x = 0.0
        self.front_min = 10.0
        self.last_msg = ""

        self.kp = 0.003
        self.align_thresh = 30.0
        self.catch_dist = 0.30
        self.search_w = 0.4
        self.forward_v = 0.12
        self.lost_timeout = 1.0

        self.get_logger().info(f'Using namespace prefix: "{prefix}"')

    def say(self, text: str):
        if text == self.last_msg:
            return
        self.last_msg = text
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def image_cb(self, msg: Image):
        # TODO: Replace this with your rabbit detector
        # Expected outputs:
        # - detected (bool)
        # - rabbit_center_x (float)
        detected = False
        rabbit_center_x = 0.0
        image_center_x = msg.width / 2.0

        if detected:
            self.rabbit_visible = True
            self.last_seen_time = time.time()
            self.error_x = rabbit_center_x - image_center_x
        else:
            self.rabbit_visible = False

    def scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return
        center = n // 2
        # Roughly a 10-degree window; can be refined using angle_min/angle_increment
        win = max(1, n // 36)
        front_ranges = [r for r in msg.ranges[center - win:center + win + 1] if r > 0.0]
        if front_ranges:
            self.front_min = min(front_ranges)

    def control_loop(self):
        tw = Twist()
        now = time.time()

        if self.state == 'SEARCH':
            # Rotate in place while searching for the rabbit
            tw.angular.z = self.search_w
            tw.linear.x = 0.0
            if self.rabbit_visible:
                self.say('I find a rabbit try to chase it')
                self.state = 'CHASE'

        elif self.state == 'CHASE':
            # If the target is lost for too long, return to SEARCH
            if not self.rabbit_visible and (now - self.last_seen_time) > self.lost_timeout:
                self.say('I lost it try to find it again')
                self.state = 'SEARCH'
            else:
                # If target is off-center, rotate to re-align
                if abs(self.error_x) > self.align_thresh:
                    self.say('its moved try to set the right route')
                    tw.angular.z = -self.kp * self.error_x
                    tw.linear.x = 0.0
                else:
                    # Move forward when target is approximately centered
                    tw.angular.z = -self.kp * self.error_x
                    tw.linear.x = self.forward_v

                # Catch condition: target visible and obstacle distance < 30 cm
                if self.rabbit_visible and self.front_min < self.catch_dist:
                    self.say('I CATCH IT!')
                    self.state = 'CATCH'

        elif self.state == 'CATCH':
            # Stop the robot after catching
            tw.linear.x = 0.0
            tw.angular.z = 0.0

        self.cmd_pub.publish(tw)


def main(args=None):
    rclpy.init(args=args)
    node = RabbitChaser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()