#!/usr/bin/env python3
"""
real_flight_test.py
-------------------
Unified real-world flight test script with 4 modes.

Usage:
  ros2 run thyra real_flight_test.py --ros-args -p mode:=HANDHELD
  ros2 run thyra real_flight_test.py --ros-args -p mode:=HOVER_TRACK -p takeoff_alt:=3.0
  ros2 run thyra real_flight_test.py --ros-args -p mode:=STATIC_LAND -p pid_kp:=0.3 -p max_vel:=0.8
  ros2 run thyra real_flight_test.py --ros-args -p mode:=DYNAMIC_LAND
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from interfaces.action import DroneCommand
from interfaces.msg import ManualControlInput, GcsHeartbeat, DroneState, ServoCommand
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
from px4_msgs.msg import VehicleLocalPosition

import time
import math


class FlightMode:
    HANDHELD     = 'HANDHELD'
    HOVER_TRACK  = 'HOVER_TRACK'
    STATIC_LAND  = 'STATIC_LAND'
    DYNAMIC_LAND = 'DYNAMIC_LAND'


class MissionState:
    WAITING       = 'WAITING'       # HANDHELD only
    IDLE          = 'IDLE'
    ARMING        = 'ARMING'
    TAKEOFF       = 'TAKEOFF'
    SEARCH        = 'SEARCH'
    TRACKING      = 'TRACKING'      # HOVER_TRACK only
    STABILIZE_5M  = 'STABILIZE_5M'  # DYNAMIC_LAND only
    DESCEND_TO_1M = 'DESCEND_TO_1M'
    STABILIZE_1M  = 'STABILIZE_1M'
    DESCEND_DEMO  = 'DESCEND_DEMO'  # HANDHELD only
    TERMINAL_LAND = 'TERMINAL_LAND'
    DONE          = 'DONE'
    RTL           = 'RTL'


class RealFlightTest(Node):
    def __init__(self):
        super().__init__('real_flight_test')

        self.declare_parameter('mode', 'HANDHELD')
        self.declare_parameter('takeoff_alt', 3.0)
        self.declare_parameter('descend_vz', 0.3)
        self.declare_parameter('pid_kp', 0.3)
        self.declare_parameter('max_vel', 0.8)
        self.declare_parameter('slant_sweep_time', 8.0)
        self.declare_parameter('lock_loss_timeout', 3.0)
        self.declare_parameter('landing_alt_trigger', 0.4)
        self.declare_parameter('kp_yaw', 0.02)
        self.declare_parameter('kp_alt', 0.3)

        self.flight_mode = self.get_parameter('mode').value.upper()
        self.takeoff_alt = self.get_parameter('takeoff_alt').value
        self.descend_vz = self.get_parameter('descend_vz').value
        self.pid_kp = self.get_parameter('pid_kp').value
        self.max_vel = self.get_parameter('max_vel').value
        self.slant_sweep_time = self.get_parameter('slant_sweep_time').value
        self.lock_loss_timeout = self.get_parameter('lock_loss_timeout').value
        self.landing_alt_trigger = self.get_parameter('landing_alt_trigger').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.kp_alt = self.get_parameter('kp_alt').value

        # QoS Profiles
        qos_hb = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_manual = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        # Publishers
        self.pub_manual = self.create_publisher(
            ManualControlInput, '/asr/thyra/in/manual_input', qos_manual)
        self.pub_heartbeat = self.create_publisher(
            GcsHeartbeat, '/asr/thyra/in/gcs_heartbeat', qos_hb)
        self.pub_gimbal = self.create_publisher(
            Float64, '/gimbal/cmd_pitch', 10)
        self.pub_servo = self.create_publisher(
            ServoCommand, '/asr/thyra/in/servo_command', 10)

        # Subscribers
        self.create_subscription(
            DroneState, '/asr/thyra/out/drone_state', self._drone_cb, 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._lpos_cb, qos_sensor)
        self.create_subscription(
            Vector3Stamped, '/asr/comparison/aruco_pixel_error',
            self._pixel_cb, 10)

        # Action Client
        self.cmd_client = ActionClient(self, DroneCommand, '/asr/thyra/in/drone_command')

        # State Variables
        self.state = MissionState.WAITING if self.flight_mode == FlightMode.HANDHELD else MissionState.IDLE
        self.state_start = self.get_clock().now()

        self.drone_state = DroneState()
        self.local_pos = VehicleLocalPosition()

        self.pixel_err_x = 0.0
        self.pixel_err_y = 0.0
        self.locked = False
        self.last_lock_time = self.get_clock().now()
        self.relative_yaw_deg = 0.0

        self.gimbal_angle_norm = 0.0
        self.terminal_start = None

        self.last_cmd_pitch = 0.0
        self.last_cmd_roll  = 0.0

        # Timers
        self.create_timer(0.1, self._heartbeat_tick)
        self.create_timer(0.05, self._mission_tick)   # 20 Hz

        self.get_logger().info(f'Real Flight Test started. Mode: {self.flight_mode}')

    def _drone_cb(self, msg: DroneState):
        self.drone_state = msg

    def _lpos_cb(self, msg: VehicleLocalPosition):
        self.local_pos = msg

    def _pixel_cb(self, msg: Vector3Stamped):
        self.pixel_err_x = msg.vector.x
        self.pixel_err_y = msg.vector.y
        self.locked = msg.vector.z > 0.5
        if self.locked:
            self.last_lock_time = self.get_clock().now()
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
        self.get_logger().info(f'STATE TRANSITION: {self.state} → {new_state}')
        self.state = new_state
        self.state_start = self.get_clock().now()
        if new_state == MissionState.TERMINAL_LAND:
            self.terminal_start = self.get_clock().now()

    def _set_gimbal(self, val):
        self.gimbal_angle_norm = val
        msg_f64 = Float64()
        msg_f64.data = float(val)
        self.pub_gimbal.publish(msg_f64)

        msg_servo = ServoCommand()
        msg_servo.timestamp = int(self.get_clock().now().nanoseconds / 1_000)
        msg_servo.aux_index = 0  # AUX1
        msg_servo.id = 0
        msg_servo.value = float(val)
        self.pub_servo.publish(msg_servo)

    def _send_cmd(self, cmd_type, target_pose=None, yaw=0.0):
        goal = DroneCommand.Goal()
        goal.command_type = cmd_type
        goal.yaw = float(yaw)
        if target_pose:
            goal.target_pose = [float(v) for v in target_pose]
        self.get_logger().info(f'CMD → {cmd_type}  pose={target_pose}')
        self.cmd_client.wait_for_server(timeout_sec=2.0)
        self.cmd_client.send_goal_async(goal)

    def _send_vel(self, vx_b, vy_b, thrust, yaw_rate=0.0):
        msg = ManualControlInput()
        msg.pitch = float(max(-1.0, min(1.0, vx_b)))
        msg.roll  = float(max(-1.0, min(1.0, vy_b)))
        msg.thrust = float(max(0.0, min(1.0, thrust)))
        msg.yaw_velocity = float(max(-1.0, min(1.0, yaw_rate)))
        msg.arm = 1
        msg.estop = 0
        msg.selfdestruct = 0
        self.pub_manual.publish(msg)

    def _alt_hold_thrust(self, target_z, z_vel_ff=0.0):
        current_z = self.local_pos.z
        z_err = target_z - current_z
        cmd = self.kp_alt * z_err + z_vel_ff
        thrust = 0.5 - cmd
        return max(0.0, min(1.0, thrust))

    def _track_target(self, target_z, z_vel_ff=0.0):
        if not self.locked:
            dt = (self.get_clock().now() - self.last_lock_time).nanoseconds / 1e9
            if dt > self.lock_loss_timeout:
                self.get_logger().error(f'Lock lost for {dt:.1f}s — RTL!')
                self._send_cmd('land')
                self._transition(MissionState.RTL)
                return
            thrust = self._alt_hold_thrust(target_z, z_vel_ff)
            self._send_vel(self.last_cmd_pitch, self.last_cmd_roll, thrust)
            return

        cmd_x = self.pixel_err_y * self.pid_kp
        cmd_y = self.pixel_err_x * self.pid_kp

        mag = math.hypot(cmd_x, cmd_y)
        if mag > self.max_vel:
            cmd_x = (cmd_x / mag) * self.max_vel
            cmd_y = (cmd_y / mag) * self.max_vel

        # Yaw alignment
        yaw_rate = self.relative_yaw_deg * self.kp_yaw

        thrust = self._alt_hold_thrust(target_z, z_vel_ff)

        self.last_cmd_pitch = cmd_x
        self.last_cmd_roll  = cmd_y
        self._send_vel(cmd_x, cmd_y, thrust, yaw_rate=yaw_rate)

    def _execute_slant_landing(self):
        elapsed = (self.get_clock().now() - self.terminal_start).nanoseconds / 1e9
        frac = min(elapsed / self.slant_sweep_time, 1.0)
        self._set_gimbal(0.0 - frac * 1.0)

        # target_z from 1.0m to 0.0m
        target_z = -1.0 + frac * (1.0 - 0.0) 

        if not self.locked:
            self.get_logger().info('Slant landing: Lock lost (blind plunge phase)')
            thrust = self._alt_hold_thrust(0.0, z_vel_ff=0.3)
            self._send_vel(self.last_cmd_pitch, self.last_cmd_roll, thrust)
        else:
            self._track_target(target_z, z_vel_ff=0.3 * frac)

        alt = -self.local_pos.z
        if alt < self.landing_alt_trigger and elapsed > self.slant_sweep_time:
            self.get_logger().info('Touchdown altitude reached!')
            self._send_cmd('land')
            self._transition(MissionState.DONE)

    def _mission_tick(self):
        if self.flight_mode == FlightMode.HANDHELD:
            self._tick_handheld()
        elif self.flight_mode == FlightMode.HOVER_TRACK:
            self._tick_hover_track()
        elif self.flight_mode == FlightMode.STATIC_LAND:
            self._tick_static_land()
        elif self.flight_mode == FlightMode.DYNAMIC_LAND:
            self._tick_dynamic_land()

    def _tick_handheld(self):
        if self.state == MissionState.WAITING:
            self._set_gimbal(0.0)
            if self.locked:
                self._transition(MissionState.SEARCH)

        elif self.state == MissionState.SEARCH:
            self._set_gimbal(0.0)
            d_ground = math.hypot(self.pixel_err_x, self.pixel_err_y)
            if self.locked:
                if self._elapsed() > 1.0: # Print occasionally
                    self.get_logger().info(f'Target seen, ground error = {d_ground:.2f}m', throttle_duration_sec=1.0)
                if d_ground < 3.0:
                    self.get_logger().info('Target within 3m — BEGIN DESCENT MODE')
                    self._transition(MissionState.DESCEND_DEMO)

        elif self.state == MissionState.DESCEND_DEMO:
            frac = min(self._elapsed() / self.slant_sweep_time, 1.0)
            self._set_gimbal(0.0 - frac * 1.0)
            if self._elapsed() > self.slant_sweep_time + 2.0:
                self.get_logger().info('Gimbal sweep complete — place drone down')
                self._transition(MissionState.DONE)

        elif self.state == MissionState.DONE:
            raise SystemExit(0)

    def _tick_hover_track(self):
        if self.state == MissionState.IDLE:
            if self.drone_state.arming_state != 2:
                self._send_cmd('arm')
                self._transition(MissionState.ARMING)
        elif self.state == MissionState.ARMING:
            if self.drone_state.arming_state == 2:
                self._send_cmd('takeoff', target_pose=[-self.takeoff_alt])
                self._transition(MissionState.TAKEOFF)
        elif self.state == MissionState.TAKEOFF:
            self._set_gimbal(0.0)
            if -self.local_pos.z > self.takeoff_alt - 0.5:
                self._send_cmd('manual')
                self._transition(MissionState.SEARCH)
        elif self.state == MissionState.SEARCH:
            self._set_gimbal(0.0)
            self._alt_hold_thrust(-self.takeoff_alt)
            if self.locked:
                self._transition(MissionState.TRACKING)
        elif self.state == MissionState.TRACKING:
            self._set_gimbal(0.0)
            self._track_target(-self.takeoff_alt)
        elif self.state in [MissionState.DONE, MissionState.RTL]:
            pass

    def _tick_static_land(self):
        # IDLE -> ARMING -> TAKEOFF -> SEARCH -> DESCEND_TO_1M -> STABILIZE_1M -> TERMINAL_LAND -> DONE
        if self.state == MissionState.IDLE:
            if self.drone_state.arming_state != 2:
                self._send_cmd('arm')
                self._transition(MissionState.ARMING)
        elif self.state == MissionState.ARMING:
            if self.drone_state.arming_state == 2:
                self._send_cmd('takeoff', target_pose=[-self.takeoff_alt])
                self._transition(MissionState.TAKEOFF)
        elif self.state == MissionState.TAKEOFF:
            self._set_gimbal(0.0)
            if -self.local_pos.z > self.takeoff_alt - 0.5:
                self._send_cmd('manual')
                self._transition(MissionState.SEARCH)
        elif self.state == MissionState.SEARCH:
            self._set_gimbal(0.0)
            thrust = self._alt_hold_thrust(-self.takeoff_alt)
            self._send_vel(0.0, 0.0, thrust)
            if self.locked:
                self._transition(MissionState.DESCEND_TO_1M)
        elif self.state == MissionState.DESCEND_TO_1M:
            self._set_gimbal(0.0)
            current_alt = -self.local_pos.z
            target_z = min(-1.0, current_alt + self.descend_vz * 0.05)
            self._track_target(target_z, z_vel_ff=self.descend_vz)
            if current_alt < 1.2:
                self._transition(MissionState.STABILIZE_1M)
        elif self.state == MissionState.STABILIZE_1M:
            self._set_gimbal(0.0)
            self._track_target(-1.0)
            if self._elapsed() > 1.0:
                self._transition(MissionState.TERMINAL_LAND)
        elif self.state == MissionState.TERMINAL_LAND:
            self._execute_slant_landing()
        elif self.state in [MissionState.DONE, MissionState.RTL]:
            pass

    def _tick_dynamic_land(self):
        # Includes STABILIZE_5M
        if self.state == MissionState.IDLE:
            if self.drone_state.arming_state != 2:
                self._send_cmd('arm')
                self._transition(MissionState.ARMING)
        elif self.state == MissionState.ARMING:
            if self.drone_state.arming_state == 2:
                self._send_cmd('takeoff', target_pose=[-self.takeoff_alt])
                self._transition(MissionState.TAKEOFF)
        elif self.state == MissionState.TAKEOFF:
            self._set_gimbal(0.0)
            if -self.local_pos.z > self.takeoff_alt - 0.5:
                self._send_cmd('manual')
                self._transition(MissionState.SEARCH)
        elif self.state == MissionState.SEARCH:
            self._set_gimbal(0.0)
            thrust = self._alt_hold_thrust(-self.takeoff_alt)
            self._send_vel(0.0, 0.0, thrust)
            if self.locked:
                self._transition(MissionState.STABILIZE_5M)
        elif self.state == MissionState.STABILIZE_5M:
            self._set_gimbal(0.0)
            self._track_target(-self.takeoff_alt)
            if self._elapsed() > 1.0:
                self._transition(MissionState.DESCEND_TO_1M)
        elif self.state == MissionState.DESCEND_TO_1M:
            self._set_gimbal(0.0)
            current_alt = -self.local_pos.z
            target_z = min(-1.0, current_alt + self.descend_vz * 0.05)
            self._track_target(target_z, z_vel_ff=self.descend_vz)
            if current_alt < 1.2:
                self._transition(MissionState.STABILIZE_1M)
        elif self.state == MissionState.STABILIZE_1M:
            self._set_gimbal(0.0)
            self._track_target(-1.0)
            if self._elapsed() > 1.0:
                self._transition(MissionState.TERMINAL_LAND)
        elif self.state == MissionState.TERMINAL_LAND:
            self._execute_slant_landing()
        elif self.state in [MissionState.DONE, MissionState.RTL]:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RealFlightTest()
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
