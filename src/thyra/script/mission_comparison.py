#!/usr/bin/env python3
"""
mission_comparison.py
---------------------
Mission controller for the Fair Comparison scenario (Mode A / Mode B).

State machine:
  IDLE → TAKEOFF → SEARCH → STABILIZE_5M → DESCEND_TO_1M →
  STABILIZE_1M → TERMINAL_LAND → DONE  (or RTL on lock loss)

Parameters:
  mode             'STATIC' or 'GIMBAL'   (default: GIMBAL)
  wind_scenario    'gust1' / 'gust2' / 'gust3'  (for CSV filename)

Publishes:
  /asr/thyra/in/manual_input         (ManualControlInput)  velocity control
  /asr/thyra/in/gcs_heartbeat        (GcsHeartbeat)        keep-alive
  /asr/sim/gimbal_pitch_deg          (Float64)             camera pitch for cam node

Subscribes:
  /asr/thyra/out/drone_state          (DroneState)
  /asr/comparison/aruco_pixel_error   (Vector3Stamped)
  /asr/sim/true_target_state          (TwistStamped)

Uses DroneCommand action for: arm, takeoff, manual_aided, land, rtl
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from interfaces.action import DroneCommand
from interfaces.msg import ManualControlInput, GcsHeartbeat, DroneState
from geometry_msgs.msg import Vector3Stamped, TwistStamped
from std_msgs.msg import Float64
from px4_msgs.msg import VehicleLocalPosition

import time
import math
import csv
import os
from datetime import datetime


# ═══════════════════════════════════════════════════════════════════════
class MissionState:
    IDLE          = 'IDLE'
    ARMING        = 'ARMING'
    TAKEOFF       = 'TAKEOFF'
    SEARCH        = 'SEARCH'
    STABILIZE_5M  = 'STABILIZE_5M'
    DESCEND_TO_1M = 'DESCEND_TO_1M'
    STABILIZE_1M  = 'STABILIZE_1M'
    TERMINAL_LAND = 'TERMINAL_LAND'
    DONE          = 'DONE'
    RTL           = 'RTL'


# ═══════════════════════════════════════════════════════════════════════
class MissionComparison(Node):
    """Full state-machine mission controller for the A/B comparison."""

    # ── Tuning constants ──────────────────────────────────────────────
    TAKEOFF_ALT       = 5.0    # m AGL
    DESCEND_ALT       = 1.0    # m AGL
    STABILIZE_TIME    = 1.0    # shortened from 10.0s
    LOCK_LOSS_TIMEOUT = 5.0    # seconds → RTL (increased to prevent premature failsafe)
    SLANT_SWEEP_TIME  = 5.0    # seconds for 45°→90° sweep
    BLIND_PLUNGE_VZ   = 0.5    # m/s descent rate for Mode A
    PID_KP            = 0.8    # Increased to reduce lag during sweep
    PID_KD            = 0.0    # Removed D to prevent jerkiness
    DESCEND_VZ        = 0.5    # m/s target descent rate (NED +Z)
    MAX_VEL           = 2.0    # m/s max horizontal velocity command

    def __init__(self):
        super().__init__('mission_comparison')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('mode', 'GIMBAL')
        self.declare_parameter('wind_scenario', 'none')
        self.declare_parameter('target_start_x', 10.0)
        self.declare_parameter('target_start_y', 0.0)
        self.mode = self.get_parameter('mode').value.upper()
        self.wind_scenario = self.get_parameter('wind_scenario').value
        self.target_start_x = self.get_parameter('target_start_x').value
        self.target_start_y = self.get_parameter('target_start_y').value

        # ── QoS profiles (matched to asr_autopilot) ──────────────────
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

        # ── Publishers ────────────────────────────────────────────────
        self.pub_manual = self.create_publisher(
            ManualControlInput, '/asr/thyra/in/manual_input', qos_manual)
        self.pub_heartbeat = self.create_publisher(
            GcsHeartbeat, '/asr/thyra/in/gcs_heartbeat', qos_hb)
        self.pub_gimbal = self.create_publisher(
            Float64, '/asr/sim/gimbal_pitch_deg', 10)

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(
            DroneState, '/asr/thyra/out/drone_state', self._drone_cb, 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._lpos_cb, qos_sensor)
        self.create_subscription(
            Vector3Stamped, '/asr/comparison/aruco_pixel_error',
            self._pixel_cb, 10)
        self.create_subscription(
            TwistStamped, '/asr/sim/true_target_state',
            self._truth_cb, 10)

        # ── Action client ─────────────────────────────────────────────
        self.cmd_client = ActionClient(self, DroneCommand, '/asr/thyra/in/drone_command')

        # ── State variables ───────────────────────────────────────────
        self.state = MissionState.IDLE
        self.state_start = self.get_clock().now()

        self.drone_state = DroneState()
        self.local_pos = VehicleLocalPosition()

        self.pixel_err_x = 0.0
        self.pixel_err_y = 0.0
        self.prev_err_x  = 0.0
        self.prev_err_y  = 0.0
        self.locked = False
        self.last_lock_time = self.get_clock().now()
        self.first_lock_time = None        # for KPI: engagement duration
        self.relative_yaw_deg = 0.0

        # Mode A one-shot capture
        self.one_shot_truth = None         # TwistStamped at terminal entry
        self.one_shot_time  = None

        # Mode B gimbal state
        self.gimbal_angle_deg = 45.0
        self.terminal_start   = None

        # Latest true target state (for KPI at touchdown)
        self.latest_truth = None
        self.lock_confirm_start = None   # For 3s continuous lock check in SEARCH

        # Last valid commands for continuity on lock loss
        self.last_cmd_pitch = 0.0
        self.last_cmd_roll  = 0.0
        self.blind_plunge_active = False  # Track if Mode A started its final drop

        # ── Timers ────────────────────────────────────────────────────
        self.create_timer(0.1, self._heartbeat_tick)
        self.create_timer(0.05, self._mission_tick)   # 20 Hz

        self.get_logger().info(
            f'Mission Comparison started  mode={self.mode}  '
            f'wind={self.wind_scenario}')
        
        # Ensure results directory exists
        os.makedirs(os.path.expanduser('~/drone-software/results'), exist_ok=True)

    # ── Callbacks ─────────────────────────────────────────────────────
    def _drone_cb(self, msg: DroneState):
        self.drone_state = msg

    def _lpos_cb(self, msg: VehicleLocalPosition):
        self.local_pos = msg

    def _pixel_cb(self, msg: Vector3Stamped):
        self.pixel_err_x = msg.vector.x
        self.pixel_err_y = msg.vector.y
        was_locked = self.locked
        self.locked = msg.vector.z > 0.5
        if self.locked:
            self.last_lock_time = self.get_clock().now()
            if self.first_lock_time is None:
                self.first_lock_time = self.get_clock().now()
                self.get_logger().info('FIRST VISUAL LOCK acquired')
        try:
            self.relative_yaw_deg = float(msg.header.frame_id)
        except (ValueError, TypeError):
            pass

    def _truth_cb(self, msg: TwistStamped):
        self.latest_truth = msg
        # One-shot capture for Mode A
        if (self.state == MissionState.TERMINAL_LAND
                and self.mode == 'STATIC'
                and self.one_shot_truth is None):
            self.one_shot_truth = msg
            self.one_shot_time  = time.monotonic()
            self.get_logger().info(
                'STATIC MODE: Captured one-shot truth  '
                f'pos=({msg.twist.linear.x:.2f}, {msg.twist.linear.y:.2f})  '
                f'vel=({msg.twist.angular.x:.2f}, {msg.twist.angular.y:.2f})')

    # ── Heartbeat ─────────────────────────────────────────────────────
    def _heartbeat_tick(self):
        msg = GcsHeartbeat()
        msg.timestamp = float(self.get_clock().now().nanoseconds / 1e9)
        self.pub_heartbeat.publish(msg)

    # ── Action helper ─────────────────────────────────────────────────
    def _send_cmd(self, cmd_type, target_pose=None, yaw=0.0):
        goal = DroneCommand.Goal()
        goal.command_type = cmd_type
        goal.yaw = yaw
        if target_pose:
            goal.target_pose = [float(v) for v in target_pose]
        self.get_logger().info(f'CMD → {cmd_type}  pose={target_pose}')
        self.cmd_client.wait_for_server(timeout_sec=2.0)
        self.cmd_client.send_goal_async(goal)

    # ── Velocity command helper ───────────────────────────────────────
    def _send_vel(self, pitch=0.0, roll=0.0, yaw_vel=0.0, thrust=0.0):
        # Store for continuity
        self.last_cmd_pitch = pitch
        self.last_cmd_roll = roll
        
        msg = ManualControlInput()
        msg.pitch = max(-1.0, min(1.0, pitch))
        msg.roll  = max(-1.0, min(1.0, roll))
        msg.yaw_velocity = max(-1.0, min(1.0, yaw_vel))
        msg.thrust = max(-1.0, min(1.0, thrust))
        self.pub_manual.publish(msg)

    # ── Gimbal command helper ─────────────────────────────────────────
    def _set_gimbal(self, deg):
        self.gimbal_angle_deg = deg
        msg = Float64()
        msg.data = deg
        self.pub_gimbal.publish(msg)

    # ── Elapsed time in current state ─────────────────────────────────
    def _elapsed(self):
        return (self.get_clock().now() - self.state_start).nanoseconds / 1e9

    def _transition(self, new_state):
        self.get_logger().info(f'STATE: {self.state} → {new_state}')
        self.state = new_state
        self.state_start = self.get_clock().now()

    # ── Visual-servoing PD controller ─────────────────────────────────
    def _track_target(self, descend_rate=0.0):
        """
        Minimize ground-projected error via body-frame velocity commands.
        North/East error -> Forward/Right error based on Drone Yaw.
        """
        # Thrust: 0.0 = hover, DESCEND_VZ = descent velocity in m/s
        thrust_val = descend_rate

        if not self.locked:
            # Maintain previous velocity on lock loss
            self._send_vel(pitch=self.last_cmd_pitch, roll=self.last_cmd_roll, 
                           thrust=thrust_val)
            return

        dt = 0.05
        # 1. Rotate NED errors into Body Frame
        # self.pixel_err_x = North_m, self.pixel_err_y = East_m
        yaw = self.drone_state.orientation[2]
        cy, sy = math.cos(yaw), math.sin(yaw)
        
        err_fwd  = self.pixel_err_x * cy + self.pixel_err_y * sy
        err_side = -self.pixel_err_x * sy + self.pixel_err_y * cy
        
        # 2. PD Control in Body Frame
        d_err_fwd = (err_fwd - self.prev_err_x) / dt
        d_err_side = (err_side - self.prev_err_y) / dt
        self.prev_err_x = err_fwd
        self.prev_err_y = err_side

        cmd_pitch = self.PID_KP * err_fwd + self.PID_KD * d_err_fwd
        cmd_roll  = self.PID_KP * err_side + self.PID_KD * d_err_side
        
        # 3. Clamp and Send
        cmd_pitch = max(-1.0, min(1.0, cmd_pitch))
        cmd_roll  = max(-1.0, min(1.0, cmd_roll))

        self._send_vel(pitch=cmd_pitch, roll=cmd_roll, thrust=thrust_val)

    # ══════════════════════════════════════════════════════════════════
    #  MAIN STATE MACHINE
    # ══════════════════════════════════════════════════════════════════
    def _mission_tick(self):
        elapsed = self._elapsed()

        # ── Failsafe: 2 s lock loss → RTL ────────────────────────────
        #    Exception: IDLE, ARMING, TAKEOFF, SEARCH (no lock yet), DONE,
        #               and TERMINAL_LAND in STATIC mode (blind landing)
        if self.state not in (MissionState.IDLE, MissionState.ARMING,
                              MissionState.TAKEOFF, MissionState.SEARCH,
                              MissionState.DONE, MissionState.RTL):
            in_static_terminal = (self.state == MissionState.TERMINAL_LAND
                                  and self.mode == 'STATIC')
            if not in_static_terminal:
                lock_age = (self.get_clock().now() - self.last_lock_time).nanoseconds / 1e9
                if lock_age > self.LOCK_LOSS_TIMEOUT:
                    self.get_logger().warn(
                        f'LOCK LOST for {lock_age:.1f}s → ABORTING')
                    self._record_kpi()  # Log where we were when we lost lock
                    self._transition(MissionState.RTL)
                    return
                elif lock_age > 0.5 and not self.locked:
                    # Short lock loss: maintain continuity via state-specific handlers
                    pass

        # ── State transitions ─────────────────────────────────────────
        if self.state == MissionState.IDLE:
            if elapsed > 5.0:
                self._send_cmd('arm')
                self._transition(MissionState.ARMING)

        elif self.state == MissionState.ARMING:
            if self.drone_state.arming_state == 1:
                self.get_logger().info('Armed → Takeoff')
                self._send_cmd('takeoff',
                               target_pose=[-self.TAKEOFF_ALT])
                self._transition(MissionState.TAKEOFF)

        elif self.state == MissionState.TAKEOFF:
            alt = -self.local_pos.z
            if alt >= self.TAKEOFF_ALT - 0.5:
                self.get_logger().info(f'Alt {alt:.1f}m reached → SEARCH')
                self._send_cmd('manual_aided')
                self._set_gimbal(45.0)
                self._transition(MissionState.SEARCH)

        elif self.state == MissionState.SEARCH:
            # Fly forward searching for ArUco lock
            # Using 1.0 pitch for max approach speed (~2.1 m/s)
            self._send_vel(pitch=1.0, yaw_vel=0.0, thrust=0.0)
            
            # Transition immediately on lock
            if self.locked:
                self.get_logger().info('ArUco LOCKED → Immediate DESCEND')
                self._transition(MissionState.DESCEND_TO_1M)

        elif self.state == MissionState.DESCEND_TO_1M:
            alt = -self.local_pos.z
            if alt <= self.DESCEND_ALT + 0.3:
                self.get_logger().info(f'Alt {alt:.1f}m → STABILIZE_1M')
                self._transition(MissionState.STABILIZE_1M)
            else:
                self._track_target(descend_rate=self.DESCEND_VZ)

        elif self.state == MissionState.STABILIZE_1M:
            self._track_target()
            if elapsed >= self.STABILIZE_TIME:
                self.get_logger().info('Stable at 1 m → TERMINAL_LAND')
                self.terminal_start = time.monotonic()
                self._transition(MissionState.TERMINAL_LAND)

        elif self.state == MissionState.TERMINAL_LAND:
            if self.mode == 'STATIC':
                self._execute_blind_landing()
            else:
                self._execute_slant_landing()

        elif self.state == MissionState.RTL:
            if elapsed < 0.5:
                # 'rtl' is not supported by this autopilot version, using 'land' as fallback
                self._send_cmd('land')
            # Wait for disarm
            if self.drone_state.arming_state == 0 and elapsed > 3.0:
                self._transition(MissionState.DONE)

        elif self.state == MissionState.DONE:
            pass  # mission complete

    # ── Mode A: Static 45° blind landing ──────────────────────────────
    def _execute_blind_landing(self):
        """
        1. At entry: capture one-shot truth (pos + vel).
        2. Extrapolate target position forward in time.
        3. Fly horizontally towards extrapolated position.
        4. When close horizontally, plunge at 0.5 m/s.
        """
        if self.one_shot_truth is None:
            return  # waiting for truth capture

        dt_since_capture = time.monotonic() - self.one_shot_time
        t = self.one_shot_truth.twist

        # Extrapolated target position
        ex = t.linear.x + t.angular.x * dt_since_capture
        ey = t.linear.y + t.angular.y * dt_since_capture

        # Current drone position
        dx = self.drone_state.position[0] if len(self.drone_state.position) >= 1 else 0.0
        dy = self.drone_state.position[1] if len(self.drone_state.position) >= 2 else 0.0

        # Horizontal error
        err_x = ex - dx
        err_y = ey - dy
        dist = math.hypot(err_x, err_y)

        # Horizontal Tracking (Always active)
        # Chase: velocity proportional to error, capped at MAX_VEL
        # We increase the gain to 1.2 for the blind phase to ensure snappy tracking
        pitch_cmd = max(-1.0, min(1.0, (err_x * 1.2) / self.MAX_VEL))
        roll_cmd  = max(-1.0, min(1.0, (err_y * 1.2) / self.MAX_VEL))

        # Vertical Trigger
        if dist < 0.2:   # Relaxed slightly to 0.2m for better reliability
            self.blind_plunge_active = True

        vz = self.BLIND_PLUNGE_VZ if self.blind_plunge_active else 0.0

        self._send_vel(pitch=pitch_cmd, roll=roll_cmd, thrust=vz)

        # Touchdown check
        alt = -self.local_pos.z
        if alt < 0.3:
            self.get_logger().info('TOUCHDOWN (STATIC) – predicted intercept')
            self._record_kpi()
            self._send_cmd('land')
            self._transition(MissionState.DONE)

    # ── Mode B: Gimbal slant landing ──────────────────────────────────
    def _execute_slant_landing(self):
        """
        Sweep gimbal from 45° → 90° over SLANT_SWEEP_TIME seconds.
        Velocity controller keeps target centered the whole time.
        At 90° with low pixel error → land.
        """
        dt_sweep = time.monotonic() - self.terminal_start
        frac = min(dt_sweep / self.SLANT_SWEEP_TIME, 1.0)
        target_pitch = 45.0 - frac * 45.0   # 45 → 0 (Straight Down)
        self._set_gimbal(target_pitch)

        # Active visual servoing
        # Wait 1s after sweep completes before descending
        vz = 0.5 if dt_sweep > (self.SLANT_SWEEP_TIME + 1.0) else 0.0
        self._track_target(descend_rate=vz)

        # Check touchdown condition: gimbal at 0° and altitude low
        if dt_sweep > (self.SLANT_SWEEP_TIME + 1.0):
            alt = -self.local_pos.z
            if alt < 0.3:
                self.get_logger().info('TOUCHDOWN (GIMBAL) – target reached')
                self._record_kpi()
                self._send_cmd('land')
                self._transition(MissionState.DONE)

    # ── KPI recording ─────────────────────────────────────────────────
    def _record_kpi(self):
        """Write a single-row CSV with the three KPIs."""
        try:
            # Drone position at touchdown
            dx = self.drone_state.position[0] if len(self.drone_state.position) >= 1 else 0.0
            dy = self.drone_state.position[1] if len(self.drone_state.position) >= 2 else 0.0
            d_yaw = 0.0
            if len(self.drone_state.orientation) >= 3:
                d_yaw = self.drone_state.orientation[2]

            # Target state at touchdown
            tx, ty, t_heading = 0.0, 0.0, 0.0
            if self.latest_truth is not None:
                tx = self.latest_truth.twist.linear.x
                ty = self.latest_truth.twist.linear.y
                t_heading = self.latest_truth.twist.angular.z

            # KPI 1: Linear distance error
            linear_error = math.hypot(dx - tx, dy - ty)

            # KPI 2: Rotation error
            rot_error_rad = abs(d_yaw - t_heading)
            if rot_error_rad > math.pi:
                rot_error_rad = 2 * math.pi - rot_error_rad
            rot_error_deg = math.degrees(rot_error_rad)

            # KPI 3: Engagement duration
            engagement_s = 0.0
            if self.first_lock_time is not None:
                engagement_s = (self.get_clock().now() - self.first_lock_time).nanoseconds / 1e9

            # Write CSV
            mode_tag = 'static' if self.mode == 'STATIC' else 'gimbal'
            now_str = datetime.now().strftime('%H%M')
            filename = f'{mode_tag}_{self.wind_scenario}_{now_str}.csv'
            csv_dir = os.path.expanduser('~/drone-software/results')
            os.makedirs(csv_dir, exist_ok=True)
            filepath = os.path.join(csv_dir, filename)

            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'mode', 'wind_scenario',
                    'linear_error_m', 'rotation_error_deg',
                    'engagement_duration_s',
                    'drone_x', 'drone_y', 'target_x', 'target_y'])
                writer.writerow([
                    mode_tag, self.wind_scenario,
                    f'{linear_error:.4f}', f'{rot_error_deg:.2f}',
                    f'{engagement_s:.2f}',
                    f'{dx:.4f}', f'{dy:.4f}',
                    f'{tx:.4f}', f'{ty:.4f}'])

            self.get_logger().info(
                f'KPI saved → {filepath}\n'
                f'  Distance Error : {linear_error:.4f} m\n'
                f'  Rotation Error : {rot_error_deg:.2f}°\n'
                f'  Engagement Time: {engagement_s:.2f} s')

        except Exception as e:
            self.get_logger().error(f'KPI recording failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionComparison()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
