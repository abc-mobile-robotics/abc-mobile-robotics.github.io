import json
import math
import random
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class CarrotManager(Node):
    def __init__(self) -> None:
        super().__init__('carrot_manager')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('rabbit_odom_topic', '/rabbit/odom')
        self.declare_parameter('carrot_state_topic', '/game/carrot_state')
        self.declare_parameter('game_state_topic', '/game/state')

        self.declare_parameter('timer_period', 0.1)
        self.declare_parameter('eat_distance', 0.25)
        self.declare_parameter('respawn_delay', 2.0)
        self.declare_parameter('min_spawn_distance_to_rabbit', 0.8)
        self.declare_parameter('spawn_margin', 0.35)
        self.declare_parameter('spawn_outside_wolf_territory', False)
        self.declare_parameter('random_seed', -1)

        # Global arena
        self.declare_parameter('arena_min_x', -4.0)
        self.declare_parameter('arena_max_x', 4.0)
        self.declare_parameter('arena_min_y', -4.0)
        self.declare_parameter('arena_max_y', 4.0)

        # Wolf territory
        self.declare_parameter('wolf_min_x', -1.5)
        self.declare_parameter('wolf_max_x', 1.5)
        self.declare_parameter('wolf_min_y', -1.5)
        self.declare_parameter('wolf_max_y', 1.5)

        self.rabbit_odom_topic = self.get_parameter('rabbit_odom_topic').value
        self.carrot_state_topic = self.get_parameter('carrot_state_topic').value
        self.game_state_topic = self.get_parameter('game_state_topic').value

        self.timer_period = float(self.get_parameter('timer_period').value)
        self.eat_distance = float(self.get_parameter('eat_distance').value)
        self.respawn_delay = float(self.get_parameter('respawn_delay').value)
        self.min_spawn_distance_to_rabbit = float(
            self.get_parameter('min_spawn_distance_to_rabbit').value
        )
        self.spawn_margin = float(self.get_parameter('spawn_margin').value)
        self.spawn_outside_wolf_territory = bool(
            self.get_parameter('spawn_outside_wolf_territory').value
        )
        self.random_seed = int(self.get_parameter('random_seed').value)

        self.arena_min_x = float(self.get_parameter('arena_min_x').value)
        self.arena_max_x = float(self.get_parameter('arena_max_x').value)
        self.arena_min_y = float(self.get_parameter('arena_min_y').value)
        self.arena_max_y = float(self.get_parameter('arena_max_y').value)

        self.wolf_min_x = float(self.get_parameter('wolf_min_x').value)
        self.wolf_max_x = float(self.get_parameter('wolf_max_x').value)
        self.wolf_min_y = float(self.get_parameter('wolf_min_y').value)
        self.wolf_max_y = float(self.get_parameter('wolf_max_y').value)

        if self.random_seed >= 0:
            random.seed(self.random_seed)

        # -----------------------------
        # Internal state
        # -----------------------------
        self.rabbit_x: Optional[float] = None
        self.rabbit_y: Optional[float] = None
        self.rabbit_yaw: Optional[float] = None

        self.carrot_active: bool = False
        self.carrot_x: float = 0.0
        self.carrot_y: float = 0.0
        self.carrot_id: int = 0

        self.last_event: str = 'init'
        self.next_spawn_time_sec: float = 0.0

        # Optional game-state info
        self.rabbit_alive: bool = True
        self.game_phase: str = 'ACTIVE'

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.carrot_pub = self.create_publisher(String, self.carrot_state_topic, 10)

        self.rabbit_odom_sub = self.create_subscription(
            Odometry,
            self.rabbit_odom_topic,
            self.rabbit_odom_callback,
            10
        )

        # Optional: if your referee already publishes JSON string on /game/state
        self.game_state_sub = self.create_subscription(
            String,
            self.game_state_topic,
            self.game_state_callback,
            10
        )

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('CarrotManager started.')
        self.get_logger().info(f'Rabbit odom topic: {self.rabbit_odom_topic}')
        self.get_logger().info(f'Carrot state topic: {self.carrot_state_topic}')

    # ---------------------------------------------------
    # Callbacks
    # ---------------------------------------------------
    def rabbit_odom_callback(self, msg: Odometry) -> None:
        self.rabbit_x = msg.pose.pose.position.x
        self.rabbit_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.rabbit_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def game_state_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.game_phase = data.get('phase', self.game_phase)
            self.rabbit_alive = bool(data.get('rabbit_alive', self.rabbit_alive))
        except Exception:
            # 如果你的 /game/state 不是 JSON，也沒關係，直接忽略
            pass

    def timer_callback(self) -> None:
        now_sec = self.now_sec()

        # 若兔子已死亡，不再刷新蘿蔔
        if not self.rabbit_alive or self.game_phase == 'CAPTURED':
            if self.carrot_active:
                self.carrot_active = False
                self.last_event = 'deactivated_because_rabbit_dead'
            self.publish_carrot_state()
            return

        # 還沒生成蘿蔔 -> 到時間就生成
        if not self.carrot_active and now_sec >= self.next_spawn_time_sec:
            self.spawn_new_carrot()

        # 已有蘿蔔 -> 檢查兔子是否吃到
        if self.carrot_active and self.rabbit_x is not None and self.rabbit_y is not None:
            dist = self.distance(self.rabbit_x, self.rabbit_y, self.carrot_x, self.carrot_y)
            if dist <= self.eat_distance:
                self.carrot_active = False
                self.last_event = 'eaten'
                self.next_spawn_time_sec = now_sec + self.respawn_delay
                self.get_logger().info(
                    f'Carrot {self.carrot_id} eaten. Respawn in {self.respawn_delay:.2f}s'
                )

        self.publish_carrot_state()

    # ---------------------------------------------------
    # Core logic
    # ---------------------------------------------------
    def spawn_new_carrot(self) -> None:
        pos = self.find_valid_spawn_position(max_attempts=200)

        if pos is None:
            self.get_logger().warn('Failed to find valid carrot spawn position.')
            self.last_event = 'spawn_failed'
            return

        self.carrot_x, self.carrot_y = pos
        self.carrot_id += 1
        self.carrot_active = True
        self.last_event = 'spawned'

        self.get_logger().info(
            f'Spawned carrot {self.carrot_id} at ({self.carrot_x:.2f}, {self.carrot_y:.2f})'
        )

    def find_valid_spawn_position(self, max_attempts: int = 200) -> Optional[Tuple[float, float]]:
        x_min = self.arena_min_x + self.spawn_margin
        x_max = self.arena_max_x - self.spawn_margin
        y_min = self.arena_min_y + self.spawn_margin
        y_max = self.arena_max_y - self.spawn_margin

        if x_min >= x_max or y_min >= y_max:
            self.get_logger().error('Invalid arena range after applying spawn_margin.')
            return None

        for _ in range(max_attempts):
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)

            # 選擇是否避免生在狼領地內
            if self.spawn_outside_wolf_territory and self.is_inside_wolf_territory(x, y):
                continue

            # 避免一生成就在兔子腳邊
            if self.rabbit_x is not None and self.rabbit_y is not None:
                d_rabbit = self.distance(x, y, self.rabbit_x, self.rabbit_y)
                if d_rabbit < self.min_spawn_distance_to_rabbit:
                    continue

            return (x, y)

        return None

    # ---------------------------------------------------
    # Helpers
    # ---------------------------------------------------
    def is_inside_wolf_territory(self, x: float, y: float) -> bool:
        return (
            self.wolf_min_x <= x <= self.wolf_max_x and
            self.wolf_min_y <= y <= self.wolf_max_y
        )

    @staticmethod
    def distance(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x2 - x1, y2 - y1)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_carrot_state(self) -> None:
        msg = String()
        payload = {
            'active': self.carrot_active,
            'carrot_id': self.carrot_id,
            'x': self.carrot_x if self.carrot_active else None,
            'y': self.carrot_y if self.carrot_active else None,
            'eat_distance': self.eat_distance,
            'event': self.last_event,
            'rabbit_alive': self.rabbit_alive,
            'phase': self.game_phase
        }
        msg.data = json.dumps(payload)
        self.carrot_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CarrotManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()