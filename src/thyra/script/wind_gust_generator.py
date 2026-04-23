#!/usr/bin/env python3
"""
wind_gust_generator.py
----------------------
Injects reproducible stochastic wind gusts into Gazebo via the
/world/<world_name>/set_wind ROS 2 service or, if unavailable,
publishes wind commands to a topic that Gazebo can consume.

Three pre-defined gust scenarios (gust1, gust2, gust3) produce
identical wind profiles every time they are run, ensuring fair
comparison between Mode A and Mode B.

Each scenario is a sequence of (time_offset, duration, speed, direction)
tuples. Between gusts the wind is calm (0 m/s).

Usage:
  ros2 run thyra wind_gust_generator.py --ros-args \
       -p scenario:=gust1 -p world_name:=default

Parameters:
  scenario     'gust1' | 'gust2' | 'gust3'
  world_name   Gazebo world name (for service path)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import math
import time
import numpy as np


# ═══════════════════════════════════════════════════════════════════════
# Gust scenario definitions
# Each entry: (start_time_s, duration_s, speed_m_s, heading_deg_NED)
# ═══════════════════════════════════════════════════════════════════════

GUST_SCENARIOS = {
    'gust1': [
        #  Gentle crosswind early, strong headwind mid-mission
        ( 8.0,  4.0,  5.0,  90.0),   # 5 m/s from east
        (20.0,  3.0,  8.0,   0.0),   # 8 m/s from north (headwind)
        (35.0,  5.0,  7.0, 270.0),   # 7 m/s from west
        (50.0,  2.0, 10.0,  45.0),   # 10 m/s NE gust
        (65.0,  3.0,  6.0, 180.0),   # 6 m/s tailwind
    ],
    'gust2': [
        # Sustained strong crosswind with brief calm windows
        ( 5.0,  8.0,  9.0, 270.0),   # 9 m/s from west, sustained
        (18.0,  2.0, 10.0, 315.0),   # 10 m/s NW burst
        (30.0,  6.0,  7.0,  90.0),   # 7 m/s from east, sustained
        (45.0,  4.0,  8.0, 135.0),   # 8 m/s SE
        (60.0,  3.0,  5.0,   0.0),   # 5 m/s from north
    ],
    'gust3': [
        # Turbulent: frequent short bursts from multiple directions
        ( 3.0,  1.5,  6.0,  60.0),
        ( 7.0,  2.0,  8.0, 150.0),
        (12.0,  1.0, 10.0, 240.0),
        (16.0,  2.5,  7.0,  30.0),
        (22.0,  1.5,  9.0, 180.0),
        (28.0,  2.0,  6.0, 330.0),
        (34.0,  3.0,  8.0,  90.0),
        (42.0,  1.0, 10.0, 210.0),
        (50.0,  2.0,  7.0, 120.0),
        (58.0,  2.5,  9.0,   0.0),
    ],
}


class WindGustGenerator(Node):
    """Publishes deterministic wind gust vectors on a fixed schedule."""

    def __init__(self):
        super().__init__('wind_gust_generator')

        self.declare_parameter('scenario', 'gust1')
        self.declare_parameter('world_name', 'default')

        scenario_name = self.get_parameter('scenario').value
        self.world = self.get_parameter('world_name').value

        if scenario_name not in GUST_SCENARIOS:
            self.get_logger().error(
                f'Unknown scenario "{scenario_name}". '
                f'Choose from: {list(GUST_SCENARIOS.keys())}')
            raise SystemExit(1)

        self.gusts = GUST_SCENARIOS[scenario_name]
        self.get_logger().info(
            f'Wind scenario: {scenario_name}  ({len(self.gusts)} gusts)')

        # Publish wind as a Vector3 topic that can be bridged to Gazebo
        # or consumed directly by the simulation
        self.pub_wind = self.create_publisher(
            Vector3, '/asr/sim/wind_vector', 10)

        self.t0 = time.monotonic()
        self.current_wind = Vector3()
        self.active_gust_idx = -1

        self.create_timer(0.1, self._tick)  # 10 Hz

    def _tick(self):
        t = time.monotonic() - self.t0

        # Find if any gust is active right now
        wind_speed = 0.0
        wind_heading = 0.0
        active = False

        for start, dur, speed, heading in self.gusts:
            if start <= t < start + dur:
                # Smooth envelope: half-sine ramp
                frac = (t - start) / dur
                envelope = math.sin(math.pi * frac)  # 0→1→0
                wind_speed = speed * envelope
                wind_heading = heading
                active = True
                break

        # Convert polar to NED Cartesian
        heading_rad = math.radians(wind_heading)
        wx = wind_speed * math.cos(heading_rad)   # North component
        wy = wind_speed * math.sin(heading_rad)   # East component
        wz = 0.0

        msg = Vector3()
        msg.x = wx
        msg.y = wy
        msg.z = wz
        self.pub_wind.publish(msg)

        if active:
            self.get_logger().info(
                f'GUST: {wind_speed:.1f} m/s @ {wind_heading:.0f}° '
                f'NED=({wx:.1f}, {wy:.1f})',
                throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = WindGustGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
