import json
import math
import time
from typing import Any

from std_msgs.msg import String


def dict_to_string_msg(payload: dict) -> String:
    msg = String()
    msg.data = json.dumps(payload)
    return msg


def string_msg_to_dict(msg: String) -> dict:
    try:
        return json.loads(msg.data)
    except Exception:
        return {}


def now_sec() -> float:
    return time.time()


def is_stale(stamp_sec: float | None, timeout_sec: float) -> bool:
    if stamp_sec is None:
        return True
    return (now_sec() - float(stamp_sec)) > float(timeout_sec)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q: Any) -> float:
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def make_vision_payload(role: str, image_width_px: float, stamp_sec: float | None = None) -> dict:
    center = float(image_width_px) / 2.0
    return {
        'role': role,
        'wolf_visible': False,
        'rabbit_visible': False,
        'carrot_visible': False,
        'wolf_confidence': 0.0,
        'rabbit_confidence': 0.0,
        'carrot_confidence': 0.0,
        'wolf_center_x': center,
        'rabbit_center_x': center,
        'carrot_center_x': center,
        'wolf_bbox_width': 0.0,
        'rabbit_bbox_width': 0.0,
        'carrot_bbox_width': 0.0,
        'stamp': now_sec() if stamp_sec is None else float(stamp_sec),
    }
