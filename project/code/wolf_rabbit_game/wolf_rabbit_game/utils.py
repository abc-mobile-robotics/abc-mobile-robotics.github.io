import json
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
