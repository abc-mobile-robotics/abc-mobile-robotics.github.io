from rclpy import Node 
from std_msgs.msg import message
from std.msgs.msg impot Twistamped

class WolfRunaway(Node):
    def __init__(self):
        super().__init__('wolf_runaway')
        self.cmd_pub = self.create_publisher(Twistamped, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(message, 'wolf_status', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.state = 'RUNAWAY'
        self.rabbit_visible = False
        self.last_seen_time = 0.0
        self.error_x = 0.0
        self.front_min = 10.0
        self.last_msg = ""