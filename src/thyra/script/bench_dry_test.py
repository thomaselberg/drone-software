#!/usr/bin/env python3
"""
bench_dry_test.py
-----------------
Bench / no-prop dry test. Walks the same state machine as
vision_landing_mission.py (mode=GIMBAL) but never spins the motors:

  • DOES NOT send arm / takeoff / manual / land DroneCommand actions
  • DOES NOT publish /asr/thyra/in/manual_input  (autopilot stays idle)
  • Velocity & thrust commands that *would* have been sent are written
    to the ROS log at 20 Hz.
  • Gimbal commands are published normally (servo is harmless).
  • /asr/mission/state JSON is published so kpi_logger_real picks it up
    and writes a CSV to ~/drone-software/results/.

State machine (mirrors GIMBAL mode):
  IDLE → SIM_TAKEOFF (5s) → SEARCH → TRACKING_2M (2s tracking)
       → DESCEND_TO_1M (3s linear) → STABILIZE_1M (1s)
       → TERMINAL_LAND (5s gimbal sweep 0 → -1) → DONE

Altitude is simulated internally (virt_alt) because the drone is on
the bench. Thrust is computed against virt_alt so the logged values are
representative rather than saturated.

Run alongside start_base.sh:

  Terminal A:  ./start_base.sh
  Terminal B:  ros2 run thyra bench_dry_test.py
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float64, String
from geometry_msgs.msg import Vector3Stamped
from interfaces.msg import GcsHeartbeat, DroneState, ServoCommand
from px4_msgs.msg import VehicleLocalPosition


class S:
    IDLE          = 'IDLE'
    SIM_TAKEOFF   = 'SIM_TAKEOFF'
    SEARCH        = 'SEARCH'
    TRACKING_2M   = 'TRACKING_2M'
    DESCEND_TO_1M = 'DESCEND_TO_1M'
    STABILIZE_1M  = 'STABILIZE_1M'
    TERMINAL_LAND = 'TERMINAL_LAND'
    DONE          = 'DONE'


class BenchDryTest(Node):

    KP_HIGH         = 0.10
    KP_LOW          = 0.30
    KP_ALT          = 0.30
    KP_YAW          = 0.02
    SLANT_SWEEP_S   = 5.0
    TAKEOFF_SIM_S   = 5.0
    DESCEND_SIM_S   = 3.0
    TRACK_DWELL_S   = 2.0
    STABILIZE_LOW_S = 1.0
    GROUND_ERR_GATE = 2.0

    def __init__(self):
        super().__init__('bench_dry_test')

        self.declare_parameter('takeoff_alt', 2.0)
        self.declare_parameter('descend_alt', 1.0)
        self.declare_parameter('descend_vz', 0.3)
        self.declare_parameter('scenario', 'BENCH_DRY')

        self.takeoff_alt = float(self.get_parameter('takeoff_alt').value)
        self.descend_alt = float(self.get_parameter('descend_alt').value)
        self.descend_vz  = float(self.get_parameter('descend_vz').value)
        self.scenario    = str(self.get_parameter('scenario').value)

        qos_hb = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        self.pub_heartbeat = self.create_publisher(
            GcsHeartbeat, '/asr/thyra/in/gcs_heartbeat', qos_hb)
        self.pub_gimbal = self.create_publisher(
            Float64, '/gimbal/cmd_pitch', 10)
        self.pub_servo = self.create_publisher(
            ServoCommand, '/asr/thyra/in/servo_command', 10)
        self.pub_state = self.create_publisher(
            String, '/asr/mission/state', 10)

        self.create_subscription(
            DroneState, '/asr/thyra/out/drone_state', self._drone_cb, 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._lpos_cb, qos_sensor)
        self.create_subscription(
            Vector3Stamped, '/asr/comparison/aruco_pixel_error',
            self._pixel_cb, 10)

        self.state = S.IDLE
        self.state_start = self.get_clock().now()
        self.first_lock_time = None
        self.terminal_start = None

        self.drone_state = DroneState()
        self.local_pos   = VehicleLocalPosition()

        self.pixel_err_x = 0.0
        self.pixel_err_y = 0.0
        self.locked = False
        self.relative_yaw_deg = 0.0

        self.gimbal_angle_norm = 0.0
        self.last_cmd_pitch = 0.0
        self.last_cmd_roll  = 0.0
        self.last_cmd_yaw   = 0.0
        self.last_cmd_thrust = 0.0
        self.virt_alt = 0.0

        self.create_timer(0.1, self._heartbeat_tick)
        self.create_timer(0.05, self._mission_tick)
        self.create_timer(0.05, self._publish_mission_state)

        self.get_logger().info(
            'BENCH_DRY_TEST started — motors will NOT be commanded. '
            f'takeoff_alt={self.takeoff_alt} descend_alt={self.descend_alt}')

    def _drone_cb(self, msg: DroneState):
        self.drone_state = msg

    def _lpos_cb(self, msg: VehicleLocalPosition):
        self.local_pos = msg

    def _pixel_cb(self, msg: Vector3Stamped):
        self.pixel_err_x = msg.vector.x
        self.pixel_err_y = msg.vector.y
        was_locked = self.locked
        self.locked = msg.vector.z > 0.5
        if self.locked and not was_locked and self.first_lock_time is None:
            self.first_lock_time = self.get_clock().now()
        try:
            self.relative_yaw_deg = float(msg.header.frame_id)
        except (ValueError, TypeError):
            pass

    def _heartbeat_tick(self):
        msg = GcsHeartbeat()
        msg.timestamp = float(self.get_clock().now().nanoseconds / 1e9)
        self.pub_heartbeat.publish(msg)

    def _elapsed(self):
        return (self.get_clock().now() - self.state_start).nanoseconds / 1e9

    def _transition(self, new_state):
        self.get_logger().info(f'STATE: {self.state} → {new_state}')
        self.state = new_state
        self.state_start = self.get_clock().now()
        if new_state == S.TERMINAL_LAND:
            self.terminal_start = self.get_clock().now()

    def _set_gimbal(self, val):
        self.gimbal_angle_norm = val

        msg_f64 = Float64()
        msg_f64.data = float(val)
        self.pub_gimbal.publish(msg_f64)

        msg_servo = ServoCommand()
        msg_servo.timestamp = int(self.get_clock().now().nanoseconds / 1_000)
        msg_servo.aux_index = 0
        msg_servo.id = 0
        msg_servo.value = float(val)
        self.pub_servo.publish(msg_servo)

    def _alt_hold_thrust(self, target_alt):
        alt_err = target_alt - self.virt_alt
        return -self.KP_ALT * alt_err

    def _log_cmd(self, pitch, roll, yaw_vel, thrust):
        pitch  = max(-1.0, min(1.0, pitch))
        roll   = max(-1.0, min(1.0, roll))
        yaw_vel = max(-1.0, min(1.0, yaw_vel))
        thrust = max(-1.0, min(1.0, thrust))

        self.last_cmd_pitch  = pitch
        self.last_cmd_roll   = roll
        self.last_cmd_yaw    = yaw_vel
        self.last_cmd_thrust = thrust

        self.get_logger().info(
            f'[DRY] {self.state:<14s} virt_alt={self.virt_alt:.2f}m '
            f'gimbal={self.gimbal_angle_norm:+.2f} '
            f'err=({self.pixel_err_x:+.2f},{self.pixel_err_y:+.2f}) lock={int(self.locked)} '
            f'cmd: pitch={pitch:+.3f} roll={roll:+.3f} '
            f'yaw_vel={yaw_vel:+.3f} thrust={thrust:+.3f}')

    def _track_target(self, descend_rate=0.0, kp=None):
        if kp is None:
            kp = self.KP_LOW
        if not self.locked:
            self._log_cmd(self.last_cmd_pitch, self.last_cmd_roll, 0.0, descend_rate)
            return

        yaw = self.drone_state.orientation[2] if len(self.drone_state.orientation) >= 3 else 0.0
        cy, sy = math.cos(yaw), math.sin(yaw)
        err_fwd  =  self.pixel_err_x * cy + self.pixel_err_y * sy
        err_side = -self.pixel_err_x * sy + self.pixel_err_y * cy

        cmd_pitch = kp * err_fwd
        cmd_roll  = kp * err_side
        yaw_cmd   = self.relative_yaw_deg * self.KP_YAW
        self._log_cmd(cmd_pitch, cmd_roll, yaw_cmd, descend_rate)

    def _mission_tick(self):
        elapsed = self._elapsed()

        if self.state == S.IDLE:
            self._set_gimbal(0.0)
            self._transition(S.SIM_TAKEOFF)

        elif self.state == S.SIM_TAKEOFF:
            self._set_gimbal(0.0)
            frac = min(elapsed / self.TAKEOFF_SIM_S, 1.0)
            self.virt_alt = self.takeoff_alt * frac
            thrust = self._alt_hold_thrust(self.takeoff_alt)
            self._log_cmd(0.0, 0.0, 0.0, thrust)
            if frac >= 1.0:
                self._transition(S.SEARCH)

        elif self.state == S.SEARCH:
            self._set_gimbal(0.0)
            self.virt_alt = self.takeoff_alt
            thrust = self._alt_hold_thrust(self.takeoff_alt)
            self._log_cmd(0.0, 0.0, 0.0, thrust)
            if self.locked:
                d_ground = math.hypot(self.pixel_err_x, self.pixel_err_y)
                if d_ground < self.GROUND_ERR_GATE:
                    self.get_logger().info(
                        f'ArUco LOCKED & ground err {d_ground:.2f}m < '
                        f'{self.GROUND_ERR_GATE}m → TRACKING_2M')
                    self._transition(S.TRACKING_2M)

        elif self.state == S.TRACKING_2M:
            self._set_gimbal(0.0)
            self.virt_alt = self.takeoff_alt
            thrust = self._alt_hold_thrust(self.takeoff_alt)
            self._track_target(descend_rate=thrust, kp=self.KP_HIGH)
            if elapsed >= self.TRACK_DWELL_S:
                self.get_logger().info(
                    f'Tracked for {self.TRACK_DWELL_S}s → DESCEND_TO_1M')
                self._transition(S.DESCEND_TO_1M)

        elif self.state == S.DESCEND_TO_1M:
            self._set_gimbal(0.0)
            frac = min(elapsed / self.DESCEND_SIM_S, 1.0)
            self.virt_alt = self.takeoff_alt + frac * (self.descend_alt - self.takeoff_alt)
            self._track_target(descend_rate=self.descend_vz, kp=self.KP_LOW)
            if frac >= 1.0:
                self.get_logger().info(f'Reached {self.descend_alt}m → STABILIZE_1M')
                self._transition(S.STABILIZE_1M)

        elif self.state == S.STABILIZE_1M:
            self._set_gimbal(0.0)
            self.virt_alt = self.descend_alt
            thrust = self._alt_hold_thrust(self.descend_alt)
            self._track_target(descend_rate=thrust, kp=self.KP_LOW)
            if elapsed >= self.STABILIZE_LOW_S:
                self.get_logger().info('Stable at low alt → TERMINAL_LAND (gimbal sweep)')
                self._transition(S.TERMINAL_LAND)

        elif self.state == S.TERMINAL_LAND:
            self.virt_alt = self.descend_alt
            dt_sweep = (self.get_clock().now() - self.terminal_start).nanoseconds / 1e9
            frac = min(dt_sweep / self.SLANT_SWEEP_S, 1.0)
            self._set_gimbal(0.0 - frac * 1.0)
            thrust = self._alt_hold_thrust(self.descend_alt)
            self._track_target(descend_rate=thrust, kp=self.KP_LOW)
            if frac >= 1.0 and dt_sweep > self.SLANT_SWEEP_S + 1.0:
                self.get_logger().info(
                    'Gimbal at -1.0 (straight down) — place drone on ground. DONE.')
                self._transition(S.DONE)

        elif self.state == S.DONE:
            self._log_cmd(0.0, 0.0, 0.0, 0.0)
            if elapsed > 2.0:
                self.get_logger().info('Bench dry test complete — shutting down.')
                raise SystemExit(0)

    def _publish_mission_state(self):
        first_lock_ns = (self.first_lock_time.nanoseconds
                         if self.first_lock_time is not None else 0)
        payload = {
            'state':            self.state,
            'mode':             'BENCH_DRY',
            'scenario':         self.scenario,
            'wind_scenario':    'none',
            'locked':           bool(self.locked),
            'altitude_m':       float(self.virt_alt),
            'pixel_err_x':      float(self.pixel_err_x),
            'pixel_err_y':      float(self.pixel_err_y),
            'ground_err_m':     float(math.hypot(self.pixel_err_x, self.pixel_err_y)),
            'gimbal_norm':      float(self.gimbal_angle_norm),
            'last_cmd_pitch':   float(self.last_cmd_pitch),
            'last_cmd_roll':    float(self.last_cmd_roll),
            'relative_yaw_deg': float(self.relative_yaw_deg),
            'first_lock_ns':    int(first_lock_ns),
            'now_ns':           int(self.get_clock().now().nanoseconds),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BenchDryTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
