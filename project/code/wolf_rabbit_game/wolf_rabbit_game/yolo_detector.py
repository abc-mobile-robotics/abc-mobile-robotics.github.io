import time
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from .utils import dict_to_string_msg, make_vision_payload

try:
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover
    CvBridge = None

try:
    from ultralytics import YOLO
except Exception:  # pragma: no cover
    YOLO = None


class YoloDetector(Node):
    def __init__(self) -> None:
        super().__init__('yolo_detector')

        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('rabbit_pub_topic', '/rabbit/vision')
        self.declare_parameter('wolf_pub_topic', '/wolf/vision')
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('rabbit_label', 'rabbit')
        self.declare_parameter('wolf_label', 'wolf')
        self.declare_parameter('carrot_label', 'carrot')
        self.declare_parameter('image_width_px', 640)
        self.declare_parameter('image_encoding', 'bgr8')

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.rabbit_pub_topic = str(self.get_parameter('rabbit_pub_topic').value)
        self.wolf_pub_topic = str(self.get_parameter('wolf_pub_topic').value)
        self.model_path = str(self.get_parameter('model_path').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.rabbit_label = str(self.get_parameter('rabbit_label').value)
        self.wolf_label = str(self.get_parameter('wolf_label').value)
        self.carrot_label = str(self.get_parameter('carrot_label').value)
        self.image_width_px = float(self.get_parameter('image_width_px').value)
        self.image_encoding = str(self.get_parameter('image_encoding').value)

        self.rabbit_pub = self.create_publisher(type(dict_to_string_msg({})), self.rabbit_pub_topic, 10)
        self.wolf_pub = self.create_publisher(type(dict_to_string_msg({})), self.wolf_pub_topic, 10)
        self.sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        self.bridge = CvBridge() if CvBridge is not None else None
        self.model = None
        if YOLO is not None:
            try:
                self.model = YOLO(self.model_path)
                self.get_logger().info(f'Loaded YOLO model: {self.model_path}')
            except Exception as exc:
                self.get_logger().warning(f'Failed to load YOLO model: {exc}')
        else:
            self.get_logger().warning('ultralytics is not installed. This node will publish empty detections.')

        if self.bridge is None:
            self.get_logger().warning('cv_bridge is not installed. Replace node dependencies before enabling image inference.')

    def image_callback(self, msg: Image) -> None:
        stamp = time.time()
        rabbit_payload = make_vision_payload('rabbit', self.image_width_px, stamp)
        wolf_payload = make_vision_payload('wolf', self.image_width_px, stamp)

        if self.model is None or self.bridge is None:
            self.rabbit_pub.publish(dict_to_string_msg(rabbit_payload))
            self.wolf_pub.publish(dict_to_string_msg(wolf_payload))
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.image_encoding)
            detections = self.run_inference(frame)
            self.fill_payloads_from_detections(detections, rabbit_payload, wolf_payload)
        except Exception as exc:
            self.get_logger().warning(f'YOLO inference failed: {exc}')

        self.rabbit_pub.publish(dict_to_string_msg(rabbit_payload))
        self.wolf_pub.publish(dict_to_string_msg(wolf_payload))

    def run_inference(self, frame) -> Dict[str, Tuple[float, float, float]]:
        results = self.model(frame, verbose=False)
        best: Dict[str, Tuple[float, float, float]] = {}
        if not results:
            return best

        result = results[0]
        boxes = getattr(result, 'boxes', None)
        names = getattr(result, 'names', {})
        if boxes is None:
            return best

        for box in boxes:
            conf = float(box.conf[0].item())
            if conf < self.confidence_threshold:
                continue
            cls_id = int(box.cls[0].item())
            label = names.get(cls_id, str(cls_id))
            xyxy = box.xyxy[0].tolist()
            center_x = 0.5 * (float(xyxy[0]) + float(xyxy[2]))
            width = float(xyxy[2]) - float(xyxy[0])
            current = best.get(label)
            if current is None or conf > current[0]:
                best[label] = (conf, center_x, width)
        return best

    def fill_payloads_from_detections(
        self,
        detections: Dict[str, Tuple[float, float, float]],
        rabbit_payload: dict,
        wolf_payload: dict,
    ) -> None:
        self._apply_detection(detections, self.wolf_label, rabbit_payload, visible_key='wolf_visible', conf_key='wolf_confidence', center_key='wolf_center_x', width_key='wolf_bbox_width')
        self._apply_detection(detections, self.carrot_label, rabbit_payload, visible_key='carrot_visible', conf_key='carrot_confidence', center_key='carrot_center_x', width_key='carrot_bbox_width')

        self._apply_detection(detections, self.rabbit_label, wolf_payload, visible_key='rabbit_visible', conf_key='rabbit_confidence', center_key='rabbit_center_x', width_key='rabbit_bbox_width')
        self._apply_detection(detections, self.carrot_label, wolf_payload, visible_key='carrot_visible', conf_key='carrot_confidence', center_key='carrot_center_x', width_key='carrot_bbox_width')

    @staticmethod
    def _apply_detection(detections: Dict[str, Tuple[float, float, float]], label: str, payload: dict, visible_key: str, conf_key: str, center_key: str, width_key: str) -> None:
        hit = detections.get(label)
        if hit is None:
            return
        conf, center_x, width = hit
        payload[visible_key] = True
        payload[conf_key] = conf
        payload[center_key] = center_x
        payload[width_key] = width


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
