#!/usr/bin/env python3
"""
mission_comparison.py
---------------------
Mission controller for the Fair Comparison scenario (Mode A / Mode B).

State machine:
  IDLE → TAKEOFF → SEARCH → [STABILIZE_5M (DYNAMIC only)] → DESCEND_TO_1M →
  STABILIZE_1M → TERMINAL_LAND → DONE  (or RTL on lock loss)

Yaw alignment: KP_YAW = 0.01 proportional controller using ground-projected
relative yaw from the ArUco detector.
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


class MissionComparison(Node):
    """Full state-machine mission controller for the A/B comparison."""

    # ── Tuning constants ──────────────────────────────────────────────
    TAKEOFF_ALT       = 5.0
    DESCEND_ALT       = 1.0
    STABILIZE_TIME    = 1.0
    LOCK_LOSS_TIMEOUT = 5.0
    SLANT_SWEEP_TIME  = 5.0
    BLIND_PLUNGE_VZ   = 0.5
    PID_KP            = 0.5
    PID_KD            = 0.0
    DESCEND_VZ        = 0.5
    MAX_VEL           = 1.5   # Must match autopilot max_horizontal_velocity
    KP_YAW            = 0.02
    KP_ALT            = 0.3

    def __init__(self):
        super().__init__('mission_comparison')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('mode', 'GIMBAL')
        self.declare_parameter('scenario', 'DYNAMIC')
        self.declare_parameter('wind_scenario', 'none')
        self.declare_parameter('target_start_x', 10.0)
        self.declare_parameter('target_start_y', 0.0)
        self.mode = self.get_parameter('mode').value.upper()
        self.scenario = self.get_parameter('scenario').value.upper()
        self.wind_scenario = self.get_parameter('wind_scenario').value
        self.target_start_x = self.get_parameter('target_start_x').value
        self.target_start_y = self.get_parameter('target_start_y').value

        # ── QoS profiles ─────────────────────────────────────────────
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
            Float64, '/gimbal/cmd_pitch', 10)

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
        self.first_lock_time = None
        self.relative_yaw_deg = 0.0

        # Mode A one-shot capture
        self.one_shot_truth = None
        self.one_shot_time  = None

        # Mode B gimbal state
        self.gimbal_angle_norm = 0.0
        self.terminal_start    = None

        # Latest true target state
        self.latest_truth = None
        self.lock_confirm_start = None

        # Last valid commands for continuity on lock loss
        self.last_cmd_pitch = 0.0
        self.last_cmd_roll  = 0.0
        self.blind_plunge_active = False

        # KPI save guard
        self.kpi_saved = False

        # Time-series recording (10 Hz from first lock to touchdown)
        self.timeseries_rows = []
        self.recording_active = False

        # ── Timers ────────────────────────────────────────────────────
        self.create_timer(0.1, self._heartbeat_tick)
        self.create_timer(0.05, self._mission_tick)   # 20 Hz

        self.get_logger().info(
            f'Mission Comparison started  mode={self.mode}  '
            f'scenario={self.scenario}  wind={self.wind_scenario}')

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
                if not self.recording_active:
                    self.recording_active = True
                    self.create_timer(0.1, self._record_sample)
        try:
            self.relative_yaw_deg = float(msg.header.frame_id)
        except (ValueError, TypeError):
            pass

    def _truth_cb(self, msg: TwistStamped):
        self.latest_truth = msg
        if (self.state == MissionState.TERMINAL_LAND
                and self.mode == 'STATIC'
                and self.one_shot_truth is None):
            self.one_shot_truth = msg
            self.one_shot_time  = time.monotonic()
            self.get_logger().info(
                'STATIC MODE: Captured one-shot truth  '
                f'pos=({msg.twist.linear.x:.2f}, {msg.twist.linear.y:.2f})  '
                f'vel=({msg.twist.angular.x:.2f}, {msg.twist.angular.y:.2f})')

    # ── Time-series sample (10 Hz) ────────────────────────────────────
    def _record_sample(self):
        """Capture one row of time-series data at 10 Hz."""
        if not self.recording_active or self.first_lock_time is None:
            return

        elapsed_ms = int((self.get_clock().now() - self.first_lock_time).nanoseconds / 1e6)

        dx = self.drone_state.position[0] if len(self.drone_state.position) >= 1 else 0.0
        dy = self.drone_state.position[1] if len(self.drone_state.position) >= 2 else 0.0
        d_yaw = self.drone_state.orientation[2] if len(self.drone_state.orientation) >= 3 else 0.0
        alt = -self.local_pos.z

        tx, ty, t_heading = 0.0, 0.0, 0.0
        if self.latest_truth is not None:
            tx = self.latest_truth.twist.linear.x
            ty = self.latest_truth.twist.linear.y
            t_heading = self.latest_truth.twist.angular.z

        linear_error = math.hypot(dx - tx, dy - ty)
        rot_err = abs(d_yaw - t_heading)
        if rot_err > math.pi:
            rot_err = 2 * math.pi - rot_err

        engagement_s = (self.get_clock().now() - self.first_lock_time).nanoseconds / 1e9

        self.timeseries_rows.append({
            'time_ms': elapsed_ms,
            'linear_error_m': f'{linear_error:.4f}',
            'rotation_error_deg': f'{math.degrees(rot_err):.2f}',
            'engagement_duration_s': f'{engagement_s:.2f}',
            'drone_x': f'{dx:.4f}', 'drone_y': f'{dy:.4f}',
            'target_x': f'{tx:.4f}', 'target_y': f'{ty:.4f}',
            'altitude_m': f'{alt:.3f}',
            'state': self.state,
        })

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
        self.last_cmd_pitch = pitch
        self.last_cmd_roll = roll

        msg = ManualControlInput()
        msg.pitch = max(-1.0, min(1.0, pitch))
        msg.roll  = max(-1.0, min(1.0, roll))
        msg.yaw_velocity = max(-1.0, min(1.0, yaw_vel))
        msg.thrust = max(-1.0, min(1.0, thrust))
        self.pub_manual.publish(msg)

    # ── Gimbal command helper ─────────────────────────────────────────
    def _set_gimbal(self, val):
        self.gimbal_angle_norm = val
        msg = Float64()
        msg.data = float(val)
        self.pub_gimbal.publish(msg)

    # ── Elapsed time in current state ─────────────────────────────────
    def _elapsed(self):
        return (self.get_clock().now() - self.state_start).nanoseconds / 1e9

    def _transition(self, new_state):
        self.get_logger().info(f'STATE: {self.state} → {new_state}')
        self.state = new_state
        self.state_start = self.get_clock().now()

    # ── Altitude hold thrust ──────────────────────────────────────────
    def _alt_hold_thrust(self, target_alt=5.0):
        """P-controller to hold altitude. Returns thrust value.
        NED convention: positive thrust = downward, so negate the error."""
        alt = -self.local_pos.z
        alt_err = target_alt - alt  # positive = too low
        return -self.KP_ALT * alt_err  # negative thrust = climb

    # ── Visual-servoing PD controller ─────────────────────────────────
    def _track_target(self, descend_rate=0.0, use_derivative=False):
        """
        Minimize ground-projected error via body-frame velocity commands.
        North/East error -> Forward/Right error based on Drone Yaw.
        """
        thrust_val = descend_rate

        if not self.locked:
            self._send_vel(pitch=self.last_cmd_pitch, roll=self.last_cmd_roll,
                           yaw_vel=self.relative_yaw_deg * self.KP_YAW,
                           thrust=thrust_val)
            return

        dt = 0.05
        yaw = self.drone_state.orientation[2]
        cy, sy = math.cos(yaw), math.sin(yaw)

        err_fwd  = self.pixel_err_x * cy + self.pixel_err_y * sy
        err_side = -self.pixel_err_x * sy + self.pixel_err_y * cy

        # Dynamic KP for DESCEND_TO_1M state
        alt = -self.local_pos.z
        if self.state == MissionState.DESCEND_TO_1M:
            kp = 1.0 if alt < 3.0 else 0.4
        else:
            kp = self.PID_KP

        # Derivative term (only for STABILIZE_5M)
        kd = 0.25 if use_derivative else 0.0
        d_err_fwd = (err_fwd - self.prev_err_x) / dt
        d_err_side = (err_side - self.prev_err_y) / dt
        self.prev_err_x = err_fwd
        self.prev_err_y = err_side

        cmd_pitch = kp * err_fwd + kd * d_err_fwd
        cmd_roll  = kp * err_side + kd * d_err_side

        cmd_pitch = max(-1.0, min(1.0, cmd_pitch))
        cmd_roll  = max(-1.0, min(1.0, cmd_roll))

        # Yaw alignment controller
        yaw_cmd = self.relative_yaw_deg * self.KP_YAW

        self._send_vel(pitch=cmd_pitch, roll=cmd_roll, yaw_vel=yaw_cmd, thrust=thrust_val)

    # ══════════════════════════════════════════════════════════════════
    #  MAIN STATE MACHINE
    # ══════════════════════════════════════════════════════════════════
    def _mission_tick(self):
        elapsed = self._elapsed()

        # ── Failsafe: lock loss → RTL ────────────────────────────────
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
                    self._record_kpi()
                    self._transition(MissionState.RTL)
                    return
                elif lock_age > 0.5 and not self.locked:
                    pass

        # ── KPI save trigger: altitude < 0.4m ────────────────────────
        if self.state in (MissionState.TERMINAL_LAND, MissionState.DESCEND_TO_1M):
            alt = -self.local_pos.z
            if alt < 0.4 and not self.kpi_saved:
                self.get_logger().info(f'ALT {alt:.2f}m < 0.4m → saving KPI')
                self._record_kpi()
                self._send_cmd('land')
                self._transition(MissionState.DONE)
                return

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
                self._set_gimbal(0.0)  # 45 deg
                self._transition(MissionState.SEARCH)

        elif self.state == MissionState.SEARCH:
            # Altitude hold in DYNAMIC scenario
            if self.scenario == 'DYNAMIC':
                thrust = self._alt_hold_thrust(self.TAKEOFF_ALT)
                self._send_vel(pitch=0.0, roll=0.0, thrust=thrust)
            else:
                # STATIC: Fly toward target
                err_x = self.target_start_x - self.drone_state.position[0]
                err_y = self.target_start_y - self.drone_state.position[1]
                thrust = self._alt_hold_thrust(self.TAKEOFF_ALT)
                self._send_vel(pitch=err_x * 0.5, roll=err_y * 0.5, thrust=thrust)

            # Transition on lock AND ground error < 3m
            if self.locked:
                d_ground = math.hypot(self.pixel_err_x, self.pixel_err_y)
                if d_ground < 3.0:
                    self.get_logger().info(
                        f'ArUco LOCKED & Ground Err {d_ground:.2f}m < 3m')
                    if self.scenario == 'DYNAMIC':
                        self.get_logger().info('→ STABILIZE_5M (DYNAMIC)')
                        self._transition(MissionState.STABILIZE_5M)
                    else:
                        self.get_logger().info('→ DESCEND_TO_1M (STATIC, skip STABILIZE_5M)')
                        self._transition(MissionState.DESCEND_TO_1M)

        elif self.state == MissionState.STABILIZE_5M:
            # DYNAMIC only: hold altitude, track with D-term
            thrust = self._alt_hold_thrust(self.TAKEOFF_ALT)
            self._track_target(descend_rate=thrust, use_derivative=True)
            d_ground = math.hypot(self.pixel_err_x, self.pixel_err_y)
            if d_ground < 0.5:
                self.get_logger().info(
                    f'Ground err {d_ground:.2f}m < 0.5m → DESCEND_TO_1M')
                self._transition(MissionState.DESCEND_TO_1M)
            elif elapsed >= 10.0:
                self.get_logger().warn(
                    f'STABILIZE_5M timeout (10s), ground err {d_ground:.2f}m → forcing descent')
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
                self.get_logger().info('Stable at 1m → TERMINAL_LAND')
                self.terminal_start = time.monotonic()
                self._transition(MissionState.TERMINAL_LAND)

        elif self.state == MissionState.TERMINAL_LAND:
            if self.mode == 'STATIC':
                self._execute_blind_landing()
            else:
                self._execute_slant_landing()

        elif self.state == MissionState.RTL:
            if elapsed < 0.5:
                self._send_cmd('land')
            if self.drone_state.arming_state == 0 and elapsed > 3.0:
                self._transition(MissionState.DONE)

        elif self.state == MissionState.DONE:
            if self.kpi_saved and elapsed > 2.0:
                self.get_logger().info('Mission complete — shutting down node')
                raise SystemExit(0)

    # ── Mode A: Static 45° blind landing ──────────────────────────────
    def _execute_blind_landing(self):
        """
        Feed-forward + P-correction blind landing.
        All commands are normalized to [-1, 1] range.
        Command = (target_velocity / MAX_VEL) + P * (position_error / MAX_VEL)
        """
        if self.one_shot_truth is None:
            return

        dt_since_capture = time.monotonic() - self.one_shot_time
        t = self.one_shot_truth.twist

        # Extrapolated target position
        ex = t.linear.x + t.angular.x * dt_since_capture
        ey = t.linear.y + t.angular.y * dt_since_capture

        # Current drone position
        dx = self.drone_state.position[0] if len(self.drone_state.position) >= 1 else 0.0
        dy = self.drone_state.position[1] if len(self.drone_state.position) >= 2 else 0.0

        # Position error
        err_x = ex - dx
        err_y = ey - dy
        dist = math.hypot(err_x, err_y)

        # Feed-forward: normalized target velocity (m/s → [-1, 1])
        ff_pitch = t.angular.x / self.MAX_VEL
        ff_roll  = t.angular.y / self.MAX_VEL

        # P-correction on position error (also normalized)
        p_pitch = (err_x * 1.0) / self.MAX_VEL
        p_roll  = (err_y * 1.0) / self.MAX_VEL

        # Total = feed-forward + correction, clamped to [-1, 1]
        pitch_cmd = max(-1.0, min(1.0, ff_pitch + p_pitch))
        roll_cmd  = max(-1.0, min(1.0, ff_roll  + p_roll))

        # Only plunge once within 0.2m of the extrapolated position
        if dist < 0.2:
            self.blind_plunge_active = True
        vz = self.BLIND_PLUNGE_VZ if self.blind_plunge_active else 0.0

        # Yaw correction using last known relative yaw
        yaw_cmd = self.relative_yaw_deg * self.KP_YAW

        self._send_vel(pitch=pitch_cmd, roll=roll_cmd, yaw_vel=yaw_cmd, thrust=vz)

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
        """
        dt_sweep = time.monotonic() - self.terminal_start
        frac = min(dt_sweep / self.SLANT_SWEEP_TIME, 1.0)
        target_val = 0.0 - frac * 1.0   # 0.0 (45°) → -1.0 (Down)
        self._set_gimbal(target_val)

        vz = 0.5 if dt_sweep > (self.SLANT_SWEEP_TIME + 1.0) else 0.0
        self._track_target(descend_rate=vz)

        if dt_sweep > (self.SLANT_SWEEP_TIME + 1.0):
            alt = -self.local_pos.z
            if alt < 0.3:
                self.get_logger().info('TOUCHDOWN (GIMBAL) – target reached')
                self._record_kpi()
                self._send_cmd('land')
                self._transition(MissionState.DONE)

    # ── KPI recording ─────────────────────────────────────────────────
    def _record_kpi(self):
        """Write time-series CSV with all accumulated rows + final touchdown row."""
        if self.kpi_saved:
            return
        self.kpi_saved = True
        self.recording_active = False  # Stop 10Hz timer

        try:
            dx = self.drone_state.position[0] if len(self.drone_state.position) >= 1 else 0.0
            dy = self.drone_state.position[1] if len(self.drone_state.position) >= 2 else 0.0
            d_yaw = 0.0
            if len(self.drone_state.orientation) >= 3:
                d_yaw = self.drone_state.orientation[2]

            tx, ty, t_heading = 0.0, 0.0, 0.0
            if self.latest_truth is not None:
                tx = self.latest_truth.twist.linear.x
                ty = self.latest_truth.twist.linear.y
                t_heading = self.latest_truth.twist.angular.z

            linear_error = math.hypot(dx - tx, dy - ty)

            rot_error_rad = abs(d_yaw - t_heading)
            if rot_error_rad > math.pi:
                rot_error_rad = 2 * math.pi - rot_error_rad
            rot_error_deg = math.degrees(rot_error_rad)

            engagement_s = 0.0
            if self.first_lock_time is not None:
                engagement_s = (self.get_clock().now() - self.first_lock_time).nanoseconds / 1e9

            # Add final touchdown row
            final_time_ms = int((self.get_clock().now() - self.first_lock_time).nanoseconds / 1e6) if self.first_lock_time else 0
            self.timeseries_rows.append({
                'time_ms': final_time_ms,
                'linear_error_m': f'{linear_error:.4f}',
                'rotation_error_deg': f'{rot_error_deg:.2f}',
                'engagement_duration_s': f'{engagement_s:.2f}',
                'drone_x': f'{dx:.4f}', 'drone_y': f'{dy:.4f}',
                'target_x': f'{tx:.4f}', 'target_y': f'{ty:.4f}',
                'altitude_m': f'{-self.local_pos.z:.3f}',
                'state': self.state,
            })

            mode_tag = 'static' if self.mode == 'STATIC' else 'gimbal'
            scenario_tag = self.scenario.lower()
            now_str = datetime.now().strftime('%H%M%S')
            filename = f'{mode_tag}_{scenario_tag}_{self.wind_scenario}_{now_str}.csv'
            csv_dir = os.path.expanduser('~/drone-software/results')
            os.makedirs(csv_dir, exist_ok=True)
            filepath = os.path.join(csv_dir, filename)

            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'time_ms', 'mode', 'scenario', 'wind_scenario',
                    'linear_error_m', 'rotation_error_deg',
                    'engagement_duration_s',
                    'drone_x', 'drone_y', 'target_x', 'target_y',
                    'altitude_m', 'state'])
                for row in self.timeseries_rows:
                    writer.writerow([
                        row['time_ms'], mode_tag, scenario_tag,
                        self.wind_scenario,
                        row['linear_error_m'], row['rotation_error_deg'],
                        row['engagement_duration_s'],
                        row['drone_x'], row['drone_y'],
                        row['target_x'], row['target_y'],
                        row['altitude_m'], row['state']])

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
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if not node.kpi_saved:
            node.get_logger().info('Node shutting down — saving KPI as fallback')
            node._record_kpi()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
