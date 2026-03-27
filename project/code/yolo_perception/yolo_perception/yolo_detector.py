import math
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from wolf_rabbit_msgs.msg import TargetDetection

try:
    from ultralytics import YOLO
except Exception:  # pragma: no cover
    YOLO = None


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('labels', ['wolf', 'rabbit', 'carrot'])
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_width_px', 640.0)
        self.declare_parameter('focal_length_px', 600.0)
        self.declare_parameter('target_width_map.wolf', 0.18)
        self.declare_parameter('target_width_map.rabbit', 0.18)
        self.declare_parameter('target_width_map.carrot', 0.10)

        image_topic = self.get_parameter('image_topic').value
        model_path = self.get_parameter('model_path').value
        self.labels = list(self.get_parameter('labels').value)
        self.conf_th = float(self.get_parameter('confidence_threshold').value)
        self.image_width_px = float(self.get_parameter('image_width_px').value)
        self.focal_length_px = float(self.get_parameter('focal_length_px').value)
        self.target_width_map = {
            'wolf': float(self.get_parameter('target_width_map.wolf').value),
            'rabbit': float(self.get_parameter('target_width_map.rabbit').value),
            'carrot': float(self.get_parameter('target_width_map.carrot').value),
        }

        self.publishers_map = {
            label: self.create_publisher(TargetDetection, f'/detections/{label}', 10)
            for label in self.labels
        }
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)

        self.model = YOLO(model_path) if YOLO is not None else None
        if self.model is None:
            self.get_logger().warning('ultralytics/YOLO not available. Node will publish nothing until installed.')
        else:
            self.get_logger().info(f'Loaded YOLO model: {model_path}')

    def image_callback(self, msg: Image) -> None:
        if self.model is None:
            return

        try:
            results = self.model.predict(source=msg.data, verbose=False)
        except Exception as exc:
            self.get_logger().error(f'YOLO inference failed: {exc}')
            return

        if not results:
            return

        result = results[0]
        names = result.names if hasattr(result, 'names') else {}
        boxes = getattr(result, 'boxes', [])
        for box in boxes:
            conf = float(box.conf[0])
            if conf < self.conf_th:
                continue
            cls_id = int(box.cls[0])
            label = names.get(cls_id, str(cls_id))
            if label not in self.publishers_map:
                continue
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            width = max(x2 - x1, 1.0)
            center_x = (x1 + x2) * 0.5
            center_y = (y1 + y2) * 0.5

            det = TargetDetection()
            det.label = label
            det.confidence = conf
            det.center_x = center_x
            det.center_y = center_y
            det.bbox_width = width
            det.bbox_height = max(y2 - y1, 1.0)
            det.visible = True
            det.estimated_distance = self.estimate_distance(label, width)
            det.stamp = self.get_clock().now().to_msg()
            self.publishers_map[label].publish(det)

    def estimate_distance(self, label: str, pixel_width: float) -> float:
        true_width = self.target_width_map.get(label, 0.15)
        return (self.focal_length_px * true_width) / max(pixel_width, 1.0)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
