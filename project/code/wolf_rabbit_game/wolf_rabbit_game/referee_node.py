import math
import threading
from typing import List, Optional, Tuple
from typing import List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .utils import dict_to_string_msg, string_msg_to_dict

class RefereeNode(Node):
    def __init__(self) -> None:
        super().__init__('referee_node')

        # ── Topics ──────────────────────────────────────────────────────────
        self.declare_parameter('rabbit_odom_topic', '/odom')
        self.declare_parameter('wolf_odom_topic', '/wolf/odom')
        self.declare_parameter('rabbit_geofence_topic', '/rabbit/geofence')
        self.declare_parameter('wolf_vision_topic', '/wolf/vision')
        self.declare_parameter('game_state_topic', '/game/state')
        self.declare_parameter('capture_distance_m', 0.4)
        self.declare_parameter('game_ack_topic', '/game/ack')
        self.game_ack_topic = str(self.get_parameter('game_ack_topic').value)

        # Optional command topic.
        # You can publish "restart", "yes", "shutdown", or "no" here.
        self.declare_parameter('game_command_topic', '/game/command')
        self.game_command_topic = str(self.get_parameter('game_command_topic').value)

        # ── Game rules ──────────────────────────────────────────────────────
        self.declare_parameter('ask_restart_on_terminal', True)

        self.rabbit_odom_topic = str(self.get_parameter('rabbit_odom_topic').value)
        self.wolf_odom_topic = str(self.get_parameter('wolf_odom_topic').value)
        self.rabbit_geofence_topic = str(self.get_parameter('rabbit_geofence_topic').value)
        self.wolf_vision_topic = str(self.get_parameter('wolf_vision_topic').value)
        self.game_state_topic = str(self.get_parameter('game_state_topic').value)
        self.capture_distance_m = float(self.get_parameter('capture_distance_m').value)
        self.game_command_topic = str(self.get_parameter('game_command_topic').value)

        self.capture_distance_m = float(self.get_parameter('capture_distance_m').value)
        self.ask_restart_on_terminal = bool(
            self.get_parameter('ask_restart_on_terminal').value
        )

         # ── Internal state ──────────────────────────────────────────────────
        self.rabbit_pose: Optional[Tuple[float, float]] = None
        self.wolf_pose: Optional[Tuple[float, float]] = None
        
        self.rabbit_geofence = {}
        self.wolf_vision = {}
        self.phase = 'ACTIVE'
        self.game_over = False
        self.winner = 'NONE'
        self.terminal_reason = ''

        self.acks = {
            'rabbit': None,
            'wolf': None,
        }

        self.round_id = 1
        self.restart_question_active = False
        self.shutdown_requested = False

        # SET UP PUBLISHERS AND SUBSCRIBERS
        self.pub = self.create_publisher(String, self.game_state_topic, 10)
        self.create_subscription(Odometry, self.rabbit_odom_topic, self.rabbit_odom_callback, 10)
        self.create_subscription(Odometry, self.wolf_odom_topic, self.wolf_odom_callback, 10)
        self.create_subscription(String, self.rabbit_geofence_topic, self.rabbit_geofence_callback, 10)
        self.create_subscription(String, self.wolf_vision_topic, self.wolf_vision_callback, 10)
        self.create_subscription(String, self.game_ack_topic, self.game_ack_callback, 10)
        self.create_subscription(String, self.game_command_topic, self.game_command_callback, 10)

        self.timer = self.create_timer(0.1, self.publish_state)

    def rabbit_odom_callback(self, msg: Odometry) -> None:
        self.rabbit_pose = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))

    def wolf_odom_callback(self, msg: Odometry) -> None:
        self.wolf_pose = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))

    def rabbit_geofence_callback(self, msg: String) -> None:
        self.rabbit_geofence = string_msg_to_dict(msg)

    def wolf_vision_callback(self, msg: String) -> None:
        self.wolf_vision = string_msg_to_dict(msg)
    
    def game_command_callback(self, msg: String) -> None:
        command = msg.data.strip().lower()

        if command in ('restart', 'reset', 'yes', 'y'):
            self.restart_game()
            return

        if command in ('shutdown', 'stop', 'no', 'n', 'quit', 'exit'):
            self.request_shutdown()
            return

        self.get_logger().warn(
            f'Unknown game command "{msg.data}". '
            f'Use restart/yes or shutdown/no.'
        )

    def game_ack_callback(self, msg: String) -> None:
        ack = string_msg_to_dict(msg)

        robot = str(ack.get('robot', '')).lower()
        ack_phase = str(ack.get('phase', '')).upper()

        if robot not in ('rabbit', 'wolf'):
            self.get_logger().warn(f'Ignoring ack from unknown robot: {ack}')
            return

        self.acks[robot] = ack

        self.get_logger().info(
            f'ACK received from {robot}: phase={ack_phase}, ack={ack}'
        )

        def _bool_from_value(self, val, default: bool = False) -> bool:
            if isinstance(val, bool):
                return val

            if val is None:
                return default

            if isinstance(val, str):
                s = val.strip().lower()
                if s in ('true', '1', 'yes', 'y', 'on'):
                    return True
                if s in ('false', '0', 'no', 'n', 'off'):
                    return False

            if isinstance(val, (int, float)):
                return bool(val)

            return bool(val)


        def distance_between_robots(self) -> float:
            if self.rabbit_pose is None or self.wolf_pose is None:
                return -1.0

            dx = self.rabbit_pose[0] - self.wolf_pose[0]
            dy = self.rabbit_pose[1] - self.wolf_pose[1]
            return math.hypot(dx, dy)


        def rabbit_inside_wolf_territory(self) -> bool:
            return self._bool_from_value(
                self.rabbit_geofence.get('inside_wolf_territory', True),
                True,
            )


        def wolf_sees_rabbit(self) -> bool:
            return self._bool_from_value(
                self.wolf_vision.get('rabbit_visible', False),
                False,
            )


        def set_terminal_state(self, phase: str, winner: str, reason: str) -> None:
            if self.game_over:
                return

            self.phase = phase
            self.game_over = True
            self.winner = winner
            self.terminal_reason = reason

            self.get_logger().warn(
                f'GAME OVER: phase={phase}, winner={winner}, reason="{reason}"'
            )

            if self.ask_restart_on_terminal:
                self.ask_restart_async()


        def ask_restart_async(self) -> None:
            if self.restart_question_active:
                return

            self.restart_question_active = True
            thread = threading.Thread(target=self._restart_prompt_thread, daemon=True)
            thread.start()


        def _restart_prompt_thread(self) -> None:
            try:
                answer = input('\nGame over. Restart? [y/n]: ').strip().lower()
            except EOFError:
                answer = 'n'

            if answer in ('y', 'yes', 'restart', 'reset'):
                self.restart_game()
            else:
                self.request_shutdown()

            self.restart_question_active = False


        def restart_game(self) -> None:
            self.round_id += 1

            self.phase = 'RESETTING'
            self.game_over = False
            self.winner = 'NONE'
            self.terminal_reason = ''

            self.acks = {
                'rabbit': None,
                'wolf': None,
            }

            self.get_logger().info(f'Restarting game. New round_id={self.round_id}')

            self.publish_payload()

            self.phase = 'ACTIVE'


        def request_shutdown(self) -> None:
            self.phase = 'SHUTDOWN'
            self.game_over = True
            self.shutdown_requested = True
            self.terminal_reason = 'shutdown_requested'

            self.get_logger().warn('Shutdown requested. Publishing SHUTDOWN state.')
            self.publish_payload()

            rclpy.shutdown()

    def publish_state(self) -> None:
        if self.shutdown_requested:
            return

        distance = self.distance_between_robots()
        inside_wolf_territory = self.rabbit_inside_wolf_territory()
        rabbit_visible = self.wolf_sees_rabbit()

        # If game is already over, keep publishing the final state.
        # This helps both robots receive the final result.
        if self.game_over:
            self.publish_payload(distance)
            return

        # Rule 1: wolf tagged rabbit.
        if distance >= 0.0 and distance <= self.capture_distance_m:
            self.set_terminal_state(
                phase='CAPTURED',
                winner='WOLF',
                reason=f'wolf tagged rabbit at distance {distance:.2f} m',
            )
            self.publish_payload(distance)
            return

        # Rule 2: rabbit escaped wolf territory.
        if not inside_wolf_territory:
            self.set_terminal_state(
                phase='ESCAPED',
                winner='RABBIT',
                reason='rabbit escaped wolf territory',
            )
            self.publish_payload(distance)
            return

        # Rule 3: wolf sees rabbit, so this is a chase phase.
        if rabbit_visible and inside_wolf_territory:
            self.phase = 'CHASING'
        else:
            self.phase = 'ACTIVE'

        self.publish_payload(distance)

    def publish_payload(self, distance: Optional[float] = None) -> None:
        if distance is None:
            distance = self.distance_between_robots()

        rabbit_alive = self.phase != 'CAPTURED'
        rabbit_tagged = self.phase == 'CAPTURED'
        rabbit_escaped = self.phase == 'ESCAPED'
        wolf_tagged_rabbit = self.phase == 'CAPTURED'

        rabbit_ack_phase = ''
        wolf_ack_phase = ''

        if self.acks['rabbit'] is not None:
            rabbit_ack_phase = str(self.acks['rabbit'].get('phase', '')).upper()

        if self.acks['wolf'] is not None:
            wolf_ack_phase = str(self.acks['wolf'].get('phase', '')).upper()

        rabbit_acknowledged = rabbit_ack_phase == self.phase
        wolf_acknowledged = wolf_ack_phase == self.phase
        all_acknowledged = rabbit_acknowledged and wolf_acknowledged

        payload = {
            # General game state
            'round_id': self.round_id,
            'phase': self.phase,
            'game_over': self.game_over,
            'winner': self.winner,
            'terminal_reason': self.terminal_reason,

            # Rabbit-side meaning
            'rabbit_alive': rabbit_alive,
            'rabbit_tagged': rabbit_tagged,
            'rabbit_escaped': rabbit_escaped,
            'rabbit_should_return_home': rabbit_escaped or self.phase in ('RESETTING', 'SHUTDOWN'),

            # Wolf-side meaning
            'wolf_tagged_rabbit': wolf_tagged_rabbit,
            'wolf_should_return_to_zone': self.phase in ('ESCAPED', 'RESETTING', 'SHUTDOWN'),

            # Acknowledgements
            'rabbit_acknowledged': rabbit_acknowledged,
            'wolf_acknowledged': wolf_acknowledged,
            'all_acknowledged': all_acknowledged,

            # Shared rule info
            'wolf_chasing': self.phase == 'CHASING',
            'distance_m': float(distance),
            'capture_distance_m': self.capture_distance_m,
        }

        self.pub.publish(dict_to_string_msg(payload))


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RefereeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
