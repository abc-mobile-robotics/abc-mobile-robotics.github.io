import time
from typing import Any, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from .utils import dict_to_string_msg

try:
    from ultralytics import YOLO
except Exception:  # pragma: no cover
    YOLO = None


class YoloDetector(Node):
    def __init__(self) -> None:
        super().__init__('yolo_detector')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('rabbit_pub_topic', '/rabbit/vision')
        self.declare_parameter('wolf_pub_topic', '/wolf/vision')
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('rabbit_label', 'rabbit')
        self.declare_parameter('wolf_label', 'wolf')
        self.declare_parameter('carrot_label', 'carrot')
        self.declare_parameter('image_width_px', 640)

        self.image_topic = self.get_parameter('image_topic').value
        self.rabbit_pub_topic = self.get_parameter('rabbit_pub_topic').value
        self.wolf_pub_topic = self.get_parameter('wolf_pub_topic').value
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.rabbit_label = self.get_parameter('rabbit_label').value
        self.wolf_label = self.get_parameter('wolf_label').value
        self.carrot_label = self.get_parameter('carrot_label').value
        self.image_width_px = float(self.get_parameter('image_width_px').value)

        self.rabbit_pub = self.create_publisher(type(dict_to_string_msg({})), self.rabbit_pub_topic, 10)
        self.wolf_pub = self.create_publisher(type(dict_to_string_msg({})), self.wolf_pub_topic, 10)
        self.sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        self.model = None
        if YOLO is not None:
            try:
                self.model = YOLO(self.model_path)
                self.get_logger().info(f'Loaded YOLO model: {self.model_path}')
            except Exception as exc:
                self.get_logger().warning(f'Failed to load YOLO model: {exc}')
        else:
            self.get_logger().warning('ultralytics is not installed. This node will publish empty detections.')

    def image_callback(self, msg: Image) -> None:
        # TODO: Replace this placeholder with cv_bridge conversion.
        rabbit_payload = self.empty_payload('rabbit')
        wolf_payload = self.empty_payload('wolf')

        if self.model is not None:
            # Starter placeholder: real implementation should convert ROS image to cv2 image.
            # results = self.model(cv_image)
            # parse detections into rabbit_payload and wolf_payload
            pass

        self.rabbit_pub.publish(dict_to_string_msg(rabbit_payload))
        self.wolf_pub.publish(dict_to_string_msg(wolf_payload))

    def empty_payload(self, role: str) -> dict:
        return {
            'role': role,
            'wolf_visible': False,
            'rabbit_visible': False,
            'carrot_visible': False,
            'wolf_confidence': 0.0,
            'rabbit_confidence': 0.0,
            'carrot_confidence': 0.0,
            'wolf_center_x': self.image_width_px / 2.0,
            'rabbit_center_x': self.image_width_px / 2.0,
            'carrot_center_x': self.image_width_px / 2.0,
            'stamp': time.time(),
        }


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
