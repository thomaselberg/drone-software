#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from interfaces.msg import ServoCommand


class GimbalPitchSweep(Node):

    # ── tuneable ─────────────────────────────────────────────────────
    MIN_VALUE        = -1.0    # normalized servo min
    MAX_VALUE        =  1.0    # normalized servo max
    SWEEP_PERIOD_S   =  4.0    # time for one full sweep (min → max → min)
    PUBLISH_RATE_HZ  = 10.0
    SERVO_AUX_INDEX  = 0       # 0 = AUX1, 1 = AUX2, ...
    # ─────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__("gimbal_pitch_sweep")

        qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.VOLATILE,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 10,
        )

        self._pub = self.create_publisher(
            ServoCommand,
            "/asr/thyra/in/servo_command",
            qos,
        )

        self._start_time = None
        self._dt = 1.0 / self.PUBLISH_RATE_HZ
        self.create_timer(self._dt, self._timer_cb)
        self.get_logger().info(
            f"Sweeping via /asr/thyra/in/servo_command "
            f"aux_index={self.SERVO_AUX_INDEX} "
            f"{self.MIN_VALUE} ↔ {self.MAX_VALUE} "
            f"over {self.SWEEP_PERIOD_S}s period"
        )

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1_000)

    def _elapsed(self) -> float:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._start_time is None:
            self._start_time = now
        return now - self._start_time

    def _sweep_value(self, elapsed: float) -> float:
        mid  = (self.MAX_VALUE + self.MIN_VALUE) / 2.0
        half = (self.MAX_VALUE - self.MIN_VALUE) / 2.0
        return mid + half * math.sin(2.0 * math.pi * elapsed / self.SWEEP_PERIOD_S)

    def _timer_cb(self):
        elapsed = self._elapsed()
        value   = self._sweep_value(elapsed)

        msg = ServoCommand()
        msg.timestamp = self._now_us()
        msg.aux_index = self.SERVO_AUX_INDEX
        msg.id = 0
        msg.value = value
     
        self._pub.publish(msg)

        self.get_logger().info(
            f"servo = {value:+.3f}  ({elapsed:.2f}s)",
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = GimbalPitchSweep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()