#!/usr/bin/env python3
"""
kpi_logger_real.py
------------------
Real-flight KPI logger for vision_landing_mission.py.

Subscribes to:
  /asr/mission/state                  (JSON String published by the mission)
  /asr/thyra/out/drone_state          (drone position/orientation)
  /fmu/out/vehicle_local_position     (altitude — backup, mission state already has it)
  /asr/comparison/aruco_pixel_error   (pixel error + lock flag from detector)

No truth source on real flight, so no linear/rotation error columns.
Records what we *can* measure: drone pose, altitude, gimbal target,
pixel error, lock flag, mission state.

  Filename: real_{scenario}_{HHMMSS}.csv
  Columns:  time_ms, scenario,
            drone_x, drone_y, drone_yaw_rad, altitude_m,
            pixel_err_x, pixel_err_y, ground_err_m,
            gimbal_norm, locked, state

Recording starts at first lock (mission state shows first_lock_ns > 0),
samples every 100 ms, and stops when state == DONE.
"""

import csv
import json
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String
from interfaces.msg import DroneState
from px4_msgs.msg import VehicleLocalPosition


class KpiLoggerReal(Node):

    SAMPLE_HZ = 10.0

    def __init__(self):
        super().__init__('kpi_logger_real')

        self.declare_parameter('scenario', 'DYNAMIC')
        self.declare_parameter('output_dir', os.path.expanduser('~/drone-software/results'))

        self.scenario_tag = self.get_parameter('scenario').value.lower()
        self.output_dir   = self.get_parameter('output_dir').value
        os.makedirs(self.output_dir, exist_ok=True)

        # State
        self.latest_state  = None
        self.latest_drone  = None
        self.latest_lpos   = None
        self.first_lock_ns = 0
        self.recording     = False
        self.saved         = False
        self.rows          = []

        # QoS for PX4 sensor topic
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        # Subscribers
        self.create_subscription(
            String, '/asr/mission/state', self._state_cb, 10)
        self.create_subscription(
            DroneState, '/asr/thyra/out/drone_state', self._drone_cb, 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._lpos_cb, qos_sensor)

        self.create_timer(1.0 / self.SAMPLE_HZ, self._sample_tick)

        self.get_logger().info(
            f'KPI logger (real) started  scenario={self.scenario_tag}  out={self.output_dir}')

    # ── Callbacks ─────────────────────────────────────────────────────
    def _state_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.latest_state = payload

        # Allow mission to override scenario tag
        sc = payload.get('scenario', '').lower()
        if sc:
            self.scenario_tag = sc

        flock = int(payload.get('first_lock_ns', 0) or 0)
        if flock > 0 and self.first_lock_ns == 0:
            self.first_lock_ns = flock
            self.recording = True
            self.get_logger().info('First lock detected → recording')

        if payload.get('state') == 'DONE' and self.recording and not self.saved:
            self._append_row(payload)
            self._write_csv()

    def _drone_cb(self, msg: DroneState):
        self.latest_drone = msg

    def _lpos_cb(self, msg: VehicleLocalPosition):
        self.latest_lpos = msg

    # ── Sampling ──────────────────────────────────────────────────────
    def _sample_tick(self):
        if not self.recording or self.saved:
            return
        if self.latest_state is None:
            return
        self._append_row(self.latest_state)

    def _append_row(self, st: dict):
        now_ns = int(st.get('now_ns', 0) or 0)
        time_ms = max(0, int((now_ns - self.first_lock_ns) / 1e6))

        dx = float(self.latest_drone.position[0]) if (
            self.latest_drone and len(self.latest_drone.position) >= 1) else 0.0
        dy = float(self.latest_drone.position[1]) if (
            self.latest_drone and len(self.latest_drone.position) >= 2) else 0.0
        d_yaw = float(self.latest_drone.orientation[2]) if (
            self.latest_drone and len(self.latest_drone.orientation) >= 3) else 0.0

        # Prefer altitude from local_pos if available (more precise than mission state)
        if self.latest_lpos is not None:
            alt = float(-self.latest_lpos.z)
        else:
            alt = float(st.get('altitude_m', 0.0))

        px_x = float(st.get('pixel_err_x', 0.0))
        px_y = float(st.get('pixel_err_y', 0.0))
        ground_err = float(st.get('ground_err_m', math.hypot(px_x, px_y)))
        gimbal_norm = float(st.get('gimbal_norm', 0.0))
        locked = bool(st.get('locked', False))
        state = st.get('state', '')

        self.rows.append({
            'time_ms':       time_ms,
            'drone_x':       f'{dx:.4f}',
            'drone_y':       f'{dy:.4f}',
            'drone_yaw_rad': f'{d_yaw:.4f}',
            'altitude_m':    f'{alt:.3f}',
            'pixel_err_x':   f'{px_x:.4f}',
            'pixel_err_y':   f'{px_y:.4f}',
            'ground_err_m':  f'{ground_err:.4f}',
            'gimbal_norm':   f'{gimbal_norm:.3f}',
            'locked':        '1' if locked else '0',
            'state':         state,
        })

    # ── CSV writer ────────────────────────────────────────────────────
    def _write_csv(self):
        if self.saved:
            return
        self.saved = True
        self.recording = False

        try:
            now_str = datetime.now().strftime('%H%M%S')
            filename = f'real_{self.scenario_tag}_{now_str}.csv'
            filepath = os.path.join(self.output_dir, filename)

            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'time_ms', 'scenario',
                    'drone_x', 'drone_y', 'drone_yaw_rad', 'altitude_m',
                    'pixel_err_x', 'pixel_err_y', 'ground_err_m',
                    'gimbal_norm', 'locked', 'state'])
                for row in self.rows:
                    writer.writerow([
                        row['time_ms'], self.scenario_tag,
                        row['drone_x'], row['drone_y'], row['drone_yaw_rad'], row['altitude_m'],
                        row['pixel_err_x'], row['pixel_err_y'], row['ground_err_m'],
                        row['gimbal_norm'], row['locked'], row['state']])

            self.get_logger().info(
                f'KPI saved → {filepath}  ({len(self.rows)} rows)')
        except Exception as e:
            self.get_logger().error(f'KPI write failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = KpiLoggerReal()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        if not node.saved and node.recording:
            node.get_logger().info('Logger shutting down — flushing CSV')
            node._write_csv()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
