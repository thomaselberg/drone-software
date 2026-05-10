#!/usr/bin/env python3
"""
kpi_logger_sim.py
-----------------
Sim-only KPI logger for vision_landing_mission.py.

Subscribes to:
  /asr/mission/state            (JSON String published by the mission)
  /asr/sim/true_target_state    (TwistStamped — synthetic_cam_comparison truth)

Writes CSV with the same format and filename pattern that the old
mission_comparison.py wrote, so existing analysis tooling and the
batch runner's filesystem-snapshot detection keep working unchanged:

  Filename: {mode_tag}_{scenario_tag}_{wind_scenario}_{HHMMSS}.csv
  Where:    mode_tag     = 'static' | 'gimbal'
            scenario_tag = 'static' | 'dynamic'

  Columns:  time_ms, mode, scenario, wind_scenario,
            linear_error_m, rotation_error_deg, engagement_duration_s,
            drone_x, drone_y, target_x, target_y,
            altitude_m, state

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

from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from interfaces.msg import DroneState


class KpiLoggerSim(Node):
    """Subscribes to mission state + sim truth, writes a CSV time series."""

    SAMPLE_HZ = 10.0

    def __init__(self):
        super().__init__('kpi_logger_sim')

        # Tags only used to decide the filename if the mission state
        # arrives before we get to record (it always does in practice).
        self.declare_parameter('mode', 'GIMBAL')
        self.declare_parameter('scenario', 'DYNAMIC')
        self.declare_parameter('wind_scenario', 'none')
        self.declare_parameter('output_dir', os.path.expanduser('~/drone-software/results'))

        self.mode_tag      = ('static' if self.get_parameter('mode').value.upper() == 'STATIC'
                              else 'gimbal')
        self.scenario_tag  = self.get_parameter('scenario').value.lower()
        self.wind_scenario = self.get_parameter('wind_scenario').value
        self.output_dir    = self.get_parameter('output_dir').value
        os.makedirs(self.output_dir, exist_ok=True)

        # State
        self.latest_state  = None     # parsed mission/state dict
        self.latest_truth  = None     # TwistStamped
        self.latest_drone  = None     # DroneState (for orientation/yaw)
        self.first_lock_ns = 0
        self.recording     = False
        self.saved         = False
        self.rows          = []

        # Subscribers
        self.create_subscription(
            String, '/asr/mission/state', self._state_cb, 10)
        self.create_subscription(
            TwistStamped, '/asr/sim/true_target_state', self._truth_cb, 10)
        self.create_subscription(
            DroneState, '/asr/thyra/out/drone_state', self._drone_cb, 10)

        # Sample timer (always running; gated by self.recording)
        self.create_timer(1.0 / self.SAMPLE_HZ, self._sample_tick)

        self.get_logger().info(
            f'KPI logger (sim) started  '
            f'mode={self.mode_tag}  scenario={self.scenario_tag}  '
            f'wind={self.wind_scenario}  out={self.output_dir}')

    # ── Callbacks ─────────────────────────────────────────────────────
    def _state_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.latest_state = payload

        # Refresh tags from the mission so we never disagree with it
        self.mode_tag      = ('static' if payload.get('mode', '').upper() == 'STATIC'
                              else 'gimbal')
        self.scenario_tag  = payload.get('scenario', '').lower() or self.scenario_tag
        self.wind_scenario = payload.get('wind_scenario', self.wind_scenario)

        flock = int(payload.get('first_lock_ns', 0) or 0)
        if flock > 0 and self.first_lock_ns == 0:
            self.first_lock_ns = flock
            self.recording = True
            self.get_logger().info('First lock detected → recording')

        # End-of-mission: write CSV once
        if payload.get('state') == 'DONE' and self.recording and not self.saved:
            self._record_final_row(payload)
            self._write_csv()

    def _truth_cb(self, msg: TwistStamped):
        self.latest_truth = msg

    def _drone_cb(self, msg: DroneState):
        self.latest_drone = msg

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

        if self.latest_truth is not None:
            tx = float(self.latest_truth.twist.linear.x)
            ty = float(self.latest_truth.twist.linear.y)
            t_heading = float(self.latest_truth.twist.angular.z)
        else:
            tx = ty = t_heading = 0.0

        linear_err = math.hypot(dx - tx, dy - ty)
        rot_err_rad = abs(d_yaw - t_heading)
        if rot_err_rad > math.pi:
            rot_err_rad = 2 * math.pi - rot_err_rad
        rot_err_deg = math.degrees(rot_err_rad)

        engagement_s = (now_ns - self.first_lock_ns) / 1e9 if self.first_lock_ns else 0.0
        alt = float(st.get('altitude_m', 0.0))

        self.rows.append({
            'time_ms':                time_ms,
            'linear_error_m':         f'{linear_err:.4f}',
            'rotation_error_deg':     f'{rot_err_deg:.2f}',
            'engagement_duration_s':  f'{engagement_s:.2f}',
            'drone_x':                f'{dx:.4f}',
            'drone_y':                f'{dy:.4f}',
            'target_x':               f'{tx:.4f}',
            'target_y':               f'{ty:.4f}',
            'altitude_m':             f'{alt:.3f}',
            'state':                  st.get('state', ''),
        })

    def _record_final_row(self, st: dict):
        # Tag the touchdown moment explicitly so analysis tools find it
        self._append_row(st)

    # ── CSV writer ────────────────────────────────────────────────────
    def _write_csv(self):
        if self.saved:
            return
        self.saved = True
        self.recording = False

        try:
            now_str = datetime.now().strftime('%H%M%S')
            filename = f'{self.mode_tag}_{self.scenario_tag}_{self.wind_scenario}_{now_str}.csv'
            filepath = os.path.join(self.output_dir, filename)

            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'time_ms', 'mode', 'scenario', 'wind_scenario',
                    'linear_error_m', 'rotation_error_deg',
                    'engagement_duration_s',
                    'drone_x', 'drone_y', 'target_x', 'target_y',
                    'altitude_m', 'state'])
                for row in self.rows:
                    writer.writerow([
                        row['time_ms'], self.mode_tag, self.scenario_tag,
                        self.wind_scenario,
                        row['linear_error_m'], row['rotation_error_deg'],
                        row['engagement_duration_s'],
                        row['drone_x'], row['drone_y'],
                        row['target_x'], row['target_y'],
                        row['altitude_m'], row['state']])

            last = self.rows[-1] if self.rows else None
            if last:
                self.get_logger().info(
                    f'KPI saved → {filepath}\n'
                    f'  Distance Error : {last["linear_error_m"]} m\n'
                    f'  Rotation Error : {last["rotation_error_deg"]}°\n'
                    f'  Engagement Time: {last["engagement_duration_s"]} s\n'
                    f'  Rows           : {len(self.rows)}')
            else:
                self.get_logger().warn(f'KPI saved → {filepath} (empty)')
        except Exception as e:
            self.get_logger().error(f'KPI write failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = KpiLoggerSim()
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
