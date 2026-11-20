from math import radians
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unitree_go.msg import LowCmd

from collision import CollisionGuard


MOTORS = 12
SERVO_MODE = 0x0A
MAX_STEP = radians(5) - 1e-6  # Leave room for float32 quantization on the wire.
HOLD_AFTER_NS = 450_000_000
HOLD_KP = 40.0
HOLD_KD = 2.0
PUBLISH_PERIOD = 0.01


class SafetyNode(Node):
    def __init__(self):
        super().__init__('go1_safety_block')
        self.guard = CollisionGuard(Path(__file__).with_name('model'))
        minimum, maximum = self.guard.limits.T
        lo, hi = minimum.astype(np.float32), maximum.astype(np.float32)
        self.minimum = np.where(lo < minimum, np.nextafter(lo, np.float32(np.inf)), lo)
        self.maximum = np.where(hi > maximum, np.nextafter(hi, np.float32(-np.inf)), hi)
        self.target = self.guard.home.astype(np.float32).astype(np.float64)
        if (np.any(self.target < self.minimum) or np.any(self.target > self.maximum)
                or not self.guard.pose_is_safe(self.target)):
            raise ValueError('The Go1 home pose failed safety checks')

        self.active = self.hold = self._hold_message()
        self.last_accepted_ns = None
        self.pending = None
        self.output = self.create_publisher(LowCmd, '/lowcmd', 5)
        self.create_subscription(
            LowCmd, '/lowcmd_bare', self._receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_timer(PUBLISH_PERIOD, self._publish)
        self.get_logger().info('Safety block ready, using simulation clock')

    def _receive(self, msg):
        self.pending = self.get_clock().now().nanoseconds, msg

    def _hold_message(self):
        msg = LowCmd()
        for index, angle in enumerate(self.target):
            motor = msg.motor_cmd[index]
            motor.mode = SERVO_MODE
            motor.q = float(angle)
            motor.kp = HOLD_KP
            motor.kd = HOLD_KD
        return msg

    def _accept(self, msg):
        values = np.asarray([(m.q, m.dq, m.tau, m.kp, m.kd) for m in msg.motor_cmd])
        if not np.isfinite(values).all():
            return False
        live = values[:MOTORS]
        clipped = np.clip(live[:, 0], self.minimum, self.maximum)
        delta = clipped - self.target
        largest = float(np.max(np.abs(delta)))
        candidate = clipped if largest <= MAX_STEP else self.target + delta * (MAX_STEP / largest)
        candidate = candidate.astype(np.float32).astype(np.float64)
        if not self.guard.transition_is_safe(self.target, candidate):
            return False

        controls = np.clip(live[:, 1:], (-20.0, -100.0, 0.0, 0.0), (20.0, 100.0, 1000.0, 100.0))
        active = LowCmd()
        for index, (angle, control) in enumerate(zip(candidate, controls)):
            motor = active.motor_cmd[index]
            motor.mode = SERVO_MODE
            motor.q = float(angle)
            motor.dq, motor.tau, motor.kp, motor.kd = map(float, control)
        self.target = candidate
        self.active = active
        self.hold = self._hold_message()
        return True

    def _publish(self):
        now = self.get_clock().now().nanoseconds
        received, self.pending = self.pending, None
        if received is not None:
            stamp, msg = received
            if 0 <= now - stamp < HOLD_AFTER_NS and self._accept(msg):
                self.last_accepted_ns = stamp
        active = self.last_accepted_ns is not None and 0 <= now - self.last_accepted_ns < HOLD_AFTER_NS
        self.output.publish(self.active if active else self.hold)


def main():
    rclpy.init()
    try:
        node = SafetyNode()
        try:
            rclpy.spin(node)
        except (ExternalShutdownException, KeyboardInterrupt):
            pass
        finally:
            node.destroy_node()
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
