#!/usr/bin/env python3
"""
vision_landing_mission.py
-------------------------
Unified vision-based landing mission. One algorithm for sim and real flight.

The shell launcher decides the environment:
  - start_comparison_sim.sh  (single sim run, with synthetic cam + wind + KPI logger)
  - start_batch_sim.sh       (multi-run sim sweep)
  - start_real_flight.sh     (real drone, RealSense, no wind, no synthetic cam)

This script is environment-agnostic. It never touches the camera source,
never starts Gazebo, never writes CSVs. KPI logging is handled by separate
nodes (kpi_logger_sim.py / kpi_logger_real.py) that subscribe to the
/asr/mission/state topic this node publishes.

State machine:
  IDLE → ARMING → TAKEOFF → SEARCH →
        [STABILIZE_HIGH (DYNAMIC only)] → DESCEND_TO_LOW →
        STABILIZE_LOW → TERMINAL_LAND → DONE

Lock-loss recovery (any tracking state — same for sim and real):
  HOLD phase 1  (0-5 s)  : pitch=0, roll=0, alt-hold thrust at altitude-of-loss
  HOLD phase 2  (>5 s)   : pitch=0, roll=0, descend at descend_vz m/s
  Touchdown trigger      : alt < terminal_alt_trigger → send 'land' (PX4 takes over)
  If lock returns        : exit HOLD, resume the previous tracking state

The 'land' command is bit-identical to the GUI Land button — both call
DroneCommand action with command_type='land', which routes to the
autopilot's executeLand → setDroneMode(BEGIN_LAND_POSITION) → landPositionMode().
On real flight, PX4's land mode uses the bottom distance sensor for the
final touchdown.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from interfaces.action import DroneCommand
from interfaces.msg import ManualControlInput, GcsHeartbeat, DroneState, ServoCommand
from geometry_msgs.msg import Vector3Stamped, TwistStamped
from std_msgs.msg import Float64, String
from px4_msgs.msg import VehicleLocalPosition

from thyra.mission_params import MissionParams

import json
import math
import time


class MissionState:
    IDLE           = 'IDLE'
    ARMING         = 'ARMING'
    TAKEOFF        = 'TAKEOFF'
    SEARCH         = 'SEARCH'
    STABILIZE_HIGH = 'STABILIZE_HIGH'
    DESCEND_TO_LOW = 'DESCEND_TO_LOW'
    STABILIZE_LOW  = 'STABILIZE_LOW'
    TERMINAL_LAND  = 'TERMINAL_LAND'
    HOLD           = 'HOLD'
    DONE           = 'DONE'


# States in which lock loss should trigger HOLD behavior
_TRACKING_STATES = (
    MissionState.STABILIZE_HIGH,
    MissionState.DESCEND_TO_LOW,
    MissionState.STABILIZE_LOW,
    MissionState.TERMINAL_LAND,
)


class VisionLandingMission(Node):
    """Single state-machine mission used by both sim and real flight.

    All tuning constants (gains, altitudes, dwell times, thresholds) live in
    thyra.mission_params.MissionParams. Each field is also exposed as a ROS
    parameter so it can be overridden from the launcher with --ros-args -p.
    Bench_dry_test.py uses the same dataclass for byte-identical numbers.
    """

    def __init__(self):
        super().__init__('vision_landing_mission')

        # ── Runtime config (mission identity, not tuning) ─────────────
        self.declare_parameter('mode', 'GIMBAL')
        self.declare_parameter('scenario', 'DYNAMIC')
        self.declare_parameter('wind_scenario', 'none')

        self.mode          = self.get_parameter('mode').value.upper()
        self.scenario      = self.get_parameter('scenario').value.upper()
        self.wind_scenario = self.get_parameter('wind_scenario').value

        # ── Tuning params (all MissionParams fields are ROS-overridable) ─
        defaults = MissionParams()
        for field_name in defaults.__dataclass_fields__:
            default = getattr(defaults, field_name)
            self.declare_parameter(field_name, default)
            setattr(self, field_name, float(self.get_parameter(field_name).value))
        # Backwards-compat alias used elsewhere in the file:
        self.terminal_trig = self.terminal_alt_trigger

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
        self.pub_manual    = self.create_publisher(
            ManualControlInput, '/asr/thyra/in/manual_input', qos_manual)
        self.pub_heartbeat = self.create_publisher(
            GcsHeartbeat, '/asr/thyra/in/gcs_heartbeat', qos_hb)
        self.pub_gimbal    = self.create_publisher(
            Float64, '/gimbal/cmd_pitch', 10)
        self.pub_servo     = self.create_publisher(
            ServoCommand, '/asr/thyra/in/servo_command', 10)
        self.pub_state     = self.create_publisher(
            String, '/asr/mission/state', 10)

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(
            DroneState, '/asr/thyra/out/drone_state', self._drone_cb, 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._lpos_cb, qos_sensor)
        self.create_subscription(
            Vector3Stamped, '/asr/comparison/aruco_pixel_error',
            self._pixel_cb, 10)
        # Truth subscriber is sim-only; the synthetic cam publishes it.
        # Used by STATIC mode's blind landing. On real flight nothing
        # publishes this, so latest_truth stays None and STATIC blind
        # landing simply cannot trigger — but we never run STATIC on real.
        self.create_subscription(
            TwistStamped, '/asr/sim/true_target_state',
            self._truth_cb, 10)

        # ── Action client ─────────────────────────────────────────────
        self.cmd_client = ActionClient(self, DroneCommand, '/asr/thyra/in/drone_command')

        # ── State variables ───────────────────────────────────────────
        self.state         = MissionState.IDLE
        self.state_start   = self.get_clock().now()
        self.return_state  = None    # State to resume after HOLD ends

        self.drone_state = DroneState()
        self.local_pos   = VehicleLocalPosition()

        self.pixel_err_x      = 0.0
        self.pixel_err_y      = 0.0
        self.locked           = False
        self.last_lock_time   = self.get_clock().now()
        self.first_lock_time  = None
        self.relative_yaw_deg = 0.0

        # STATIC mode (sim only) one-shot truth capture for blind landing
        self.latest_truth      = None
        self.one_shot_truth    = None
        self.one_shot_time     = None
        self.blind_plunge_active = False

        # GIMBAL mode terminal sweep timer
        self.terminal_start = None

        # Gimbal angle (-1.0 = down, 0.0 = 45°, +1.0 = horizon)
        self.gimbal_angle_norm = 0.0

        # Last commanded velocity (kept for state publisher visibility)
        self.last_cmd_pitch = 0.0
        self.last_cmd_roll  = 0.0

        # Lock-loss bookkeeping for HOLD state
        self.lock_loss_start = None
        self.lock_loss_alt   = None    # altitude at moment of loss

        # STABILIZE_HIGH dwell bookkeeping (lower bound + fixed dwell)
        self.stab_high_threshold_seen = False
        self.stab_high_dwell_start    = None

        # ── Timers ────────────────────────────────────────────────────
        self.create_timer(0.1, self._heartbeat_tick)        # 10 Hz heartbeat
        self.create_timer(0.05, self._mission_tick)         # 20 Hz state machine
        self.create_timer(0.1, self._publish_mission_state) # 10 Hz state pub

        self.get_logger().info(
            f'VisionLandingMission started  mode={self.mode}  '
            f'scenario={self.scenario}  takeoff_alt={self.takeoff_alt:.2f}m')

    # ══════════════════════════════════════════════════════════════════
    #  Callbacks
    # ══════════════════════════════════════════════════════════════════
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
            if self.first_lock_time is None:
                self.first_lock_time = self.get_clock().now()
                self.get_logger().info('FIRST VISUAL LOCK acquired')
        try:
            self.relative_yaw_deg = float(msg.header.frame_id)
        except (ValueError, TypeError):
            pass

    def _truth_cb(self, msg: TwistStamped):
        """Sim-only. Used by STATIC mode's blind landing."""
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

    # ══════════════════════════════════════════════════════════════════
    #  Heartbeat & action helpers
    # ══════════════════════════════════════════════════════════════════
    def _heartbeat_tick(self):
        msg = GcsHeartbeat()
        msg.timestamp = float(self.get_clock().now().nanoseconds / 1e9)
        self.pub_heartbeat.publish(msg)

    def _send_cmd(self, cmd_type, target_pose=None, yaw=0.0):
        goal = DroneCommand.Goal()
        goal.command_type = cmd_type
        goal.yaw = float(yaw)
        if target_pose:
            goal.target_pose = [float(v) for v in target_pose]
        self.get_logger().info(f'CMD → {cmd_type}  pose={target_pose}')
        self.cmd_client.wait_for_server(timeout_sec=2.0)
        self.cmd_client.send_goal_async(goal)

    def _send_vel(self, pitch=0.0, roll=0.0, yaw_vel=0.0, thrust=0.0):
        self.last_cmd_pitch = pitch
        self.last_cmd_roll  = roll
        msg = ManualControlInput()
        msg.pitch        = max(-1.0, min(1.0, pitch))
        msg.roll         = max(-1.0, min(1.0, roll))
        msg.yaw_velocity = max(-1.0, min(1.0, yaw_vel))
        msg.thrust       = max(-1.0, min(1.0, thrust))
        self.pub_manual.publish(msg)

    def _set_gimbal(self, val):
        """Publish gimbal target on both topics:
           - Float64 /gimbal/cmd_pitch  (consumed by detector + synthetic cam)
           - ServoCommand /asr/thyra/in/servo_command (drives real AUX servo)
        Convention: -1.0 = straight down, 0.0 = 45°, +1.0 = horizon."""
        self.gimbal_angle_norm = val

        msg_f64 = Float64()
        msg_f64.data = float(val)
        self.pub_gimbal.publish(msg_f64)

        msg_servo = ServoCommand()
        msg_servo.timestamp = int(self.get_clock().now().nanoseconds / 1_000)
        msg_servo.aux_index = 0     # AUX1
        msg_servo.id        = 0     # GimbalState::Auto
        msg_servo.value     = float(val)
        self.pub_servo.publish(msg_servo)

    def _elapsed(self):
        return (self.get_clock().now() - self.state_start).nanoseconds / 1e9

    def _transition(self, new_state):
        self.get_logger().info(f'STATE: {self.state} → {new_state}')
        # Reset STABILIZE_HIGH dwell tracking on entry — every entry (from
        # SEARCH or from HOLD recovery) re-confirms convergence from scratch.
        if new_state == MissionState.STABILIZE_HIGH:
            self.stab_high_threshold_seen = False
            self.stab_high_dwell_start    = None
        self.state = new_state
        self.state_start = self.get_clock().now()

    # ══════════════════════════════════════════════════════════════════
    #  Controllers
    # ══════════════════════════════════════════════════════════════════
    def _alt_hold_thrust(self, target_alt):
        """P-controller on altitude. Returns thrust value in [-1, 1].
        manual_aided interprets thrust as vz target (negative = climb)."""
        alt = -self.local_pos.z
        alt_err = target_alt - alt          # positive = too low
        return -self.KP_ALT * alt_err       # negative = climb

    def _track_target(self, descend_rate=0.0, kp=None):
        """
        Visual servo: pixel-error (NED ground meters) → body-frame velocity.
        Caller picks descend_rate (vz target, positive down) and which kp.
        Lock loss is handled centrally in _mission_tick — this method
        assumes self.locked is True at call time.
        """
        if kp is None:
            kp = self.KP_LOW

        yaw = self.drone_state.orientation[2] if len(self.drone_state.orientation) >= 3 else 0.0
        cy, sy = math.cos(yaw), math.sin(yaw)

        err_fwd  =  self.pixel_err_x * cy + self.pixel_err_y * sy
        err_side = -self.pixel_err_x * sy + self.pixel_err_y * cy

        cmd_pitch = max(-1.0, min(1.0, kp * err_fwd))
        cmd_roll  = max(-1.0, min(1.0, kp * err_side))

        yaw_cmd = self.relative_yaw_deg * self.KP_YAW
        self._send_vel(pitch=cmd_pitch, roll=cmd_roll,
                       yaw_vel=yaw_cmd, thrust=descend_rate)

    # ══════════════════════════════════════════════════════════════════
    #  HOLD behavior (lock-loss recovery)
    # ══════════════════════════════════════════════════════════════════
    def _enter_hold(self):
        """Transition to HOLD, remembering where to return on re-lock."""
        if self.state == MissionState.HOLD:
            return
        self.return_state = self.state
        self.lock_loss_start = self.get_clock().now()
        self.lock_loss_alt = -self.local_pos.z
        self._transition(MissionState.HOLD)
        self.get_logger().warn(
            f'LOCK LOST in {self.return_state} at alt={self.lock_loss_alt:.2f}m '
            f'→ HOLD (hover {self.hold_hover_s:.0f}s, then descend)')

    def _tick_hold(self):
        """While in HOLD: hover for hold_hover_s, then descend until ground."""
        # Re-lock → resume previous state
        if self.locked and self.return_state is not None:
            self.get_logger().info(f'LOCK RECOVERED → resuming {self.return_state}')
            self._transition(self.return_state)
            self.return_state = None
            self.lock_loss_start = None
            self.lock_loss_alt = None
            return

        elapsed_lost = (self.get_clock().now() - self.lock_loss_start).nanoseconds / 1e9

        # Touchdown handoff to PX4 land mode (works for sim and real)
        alt = -self.local_pos.z
        if alt < self.terminal_trig:
            self.get_logger().info(
                f'HOLD: alt {alt:.2f}m < {self.terminal_trig:.2f}m → land (PX4 takes over)')
            self._send_cmd('land')
            self._transition(MissionState.DONE)
            return

        if elapsed_lost < self.hold_hover_s:
            # Phase 1: hover at altitude where we lost lock
            thrust = self._alt_hold_thrust(self.lock_loss_alt)
            self._send_vel(pitch=0.0, roll=0.0, yaw_vel=0.0, thrust=thrust)
        else:
            # Phase 2: controlled descent at descend_vz
            self._send_vel(pitch=0.0, roll=0.0, yaw_vel=0.0,
                           thrust=self.descend_vz)

    # ══════════════════════════════════════════════════════════════════
    #  Main state machine
    # ══════════════════════════════════════════════════════════════════
    def _mission_tick(self):
        # ── HOLD has its own dispatcher ──────────────────────────────
        if self.state == MissionState.HOLD:
            self._tick_hold()
            return

        # ── Lock-loss check (excluding STATIC blind landing, which is
        #    intentionally lock-free) ────────────────────────────────
        if self.state in _TRACKING_STATES and not self.locked:
            in_static_terminal = (self.state == MissionState.TERMINAL_LAND
                                  and self.mode == 'STATIC')
            if not in_static_terminal:
                self._enter_hold()
                return

        elapsed = self._elapsed()

        # ── Touchdown trigger common to descent states ───────────────
        if self.state in (MissionState.DESCEND_TO_LOW,
                          MissionState.STABILIZE_LOW,
                          MissionState.TERMINAL_LAND):
            alt = -self.local_pos.z
            if alt < self.terminal_trig:
                self.get_logger().info(
                    f'ALT {alt:.2f}m < {self.terminal_trig:.2f}m → land (PX4 takes over)')
                self._send_cmd('land')
                self._transition(MissionState.DONE)
                return

        # ── State dispatch ───────────────────────────────────────────
        if self.state == MissionState.IDLE:
            if elapsed > 5.0:
                self._send_cmd('arm')
                self._transition(MissionState.ARMING)

        elif self.state == MissionState.ARMING:
            if self.drone_state.arming_state == 1:   # ARMED
                self.get_logger().info('Armed → Takeoff')
                self._send_cmd('takeoff', target_pose=[-self.takeoff_alt])
                self._transition(MissionState.TAKEOFF)

        elif self.state == MissionState.TAKEOFF:
            alt = -self.local_pos.z
            if alt >= self.takeoff_alt - 0.5:
                self.get_logger().info(f'Alt {alt:.2f}m reached → SEARCH')
                self._send_cmd('manual_aided')
                self._set_gimbal(0.0)   # 45°
                self._transition(MissionState.SEARCH)

        elif self.state == MissionState.SEARCH:
            if self.scenario == 'DYNAMIC':
                # Hold position, wait for the moving target to enter FoV
                thrust = self._alt_hold_thrust(self.takeoff_alt)
                self._send_vel(pitch=0.0, roll=0.0, thrust=thrust)
            else:
                # STATIC: fly toward the known marker location
                err_x = self.target_start_x - (self.drone_state.position[0] if len(self.drone_state.position) >= 1 else 0.0)
                err_y = self.target_start_y - (self.drone_state.position[1] if len(self.drone_state.position) >= 2 else 0.0)
                thrust = self._alt_hold_thrust(self.takeoff_alt)
                self._send_vel(pitch=err_x * self.search_kp,
                               roll =err_y * self.search_kp,
                               thrust=thrust)

            if self.locked:
                d_ground = math.hypot(self.pixel_err_x, self.pixel_err_y)
                if d_ground < 3.0:
                    self.get_logger().info(
                        f'ArUco LOCKED & ground err {d_ground:.2f}m < 3m')
                    if self.scenario == 'DYNAMIC':
                        self._transition(MissionState.STABILIZE_HIGH)
                    else:
                        self._transition(MissionState.DESCEND_TO_LOW)

        elif self.state == MissionState.STABILIZE_HIGH:
            # Lock guaranteed (lock-loss check above would have gone to HOLD)
            thrust = self._alt_hold_thrust(self.takeoff_alt)
            self._track_target(descend_rate=thrust, kp=self.KP_HIGH)
            d_ground = math.hypot(self.pixel_err_x, self.pixel_err_y)

            # Lower bound: arm the dwell timer the first time ground_err drops
            # below the threshold. The timer never resets once started — even
            # if ground_err climbs back above threshold, we proceed once
            # stabilize_high_time has elapsed.
            if not self.stab_high_threshold_seen and d_ground < self.ground_err_thresh:
                self.stab_high_threshold_seen = True
                self.stab_high_dwell_start = self.get_clock().now()
                self.get_logger().info(
                    f'Ground err {d_ground:.2f}m < {self.ground_err_thresh:.2f}m '
                    f'— STABILIZE_HIGH dwell started ({self.stabilize_high_time:.1f}s)')

            # Exit: dwell complete
            if self.stab_high_threshold_seen:
                dwell_s = (self.get_clock().now() - self.stab_high_dwell_start).nanoseconds / 1e9
                if dwell_s >= self.stabilize_high_time:
                    self.get_logger().info(
                        f'STABILIZE_HIGH dwell complete ({dwell_s:.2f}s) '
                        f'→ DESCEND_TO_LOW')
                    self._transition(MissionState.DESCEND_TO_LOW)
                    return

            # Safety cap: never exceed stabilize_high_timeout even if the
            # lower bound is never met (drone drifted, marker partially seen,
            # etc.). Forces descent so the mission cannot hang indefinitely.
            if elapsed >= self.stabilize_high_timeout:
                self.get_logger().warn(
                    f'STABILIZE_HIGH timeout ({self.stabilize_high_timeout:.1f}s, '
                    f'ground err {d_ground:.2f}m, '
                    f'threshold_seen={self.stab_high_threshold_seen}) '
                    f'→ forcing descent')
                self._transition(MissionState.DESCEND_TO_LOW)

        elif self.state == MissionState.DESCEND_TO_LOW:
            alt = -self.local_pos.z
            if alt <= self.descend_alt + 0.3:
                self.get_logger().info(f'Alt {alt:.2f}m → STABILIZE_LOW')
                self._transition(MissionState.STABILIZE_LOW)
            else:
                self._track_target(descend_rate=self.descend_vz, kp=self.KP_LOW)

        elif self.state == MissionState.STABILIZE_LOW:
            thrust = self._alt_hold_thrust(self.descend_alt)
            self._track_target(descend_rate=thrust, kp=self.KP_LOW)
            if elapsed >= self.stabilize_low_time:
                self.get_logger().info('Stable at low alt → TERMINAL_LAND')
                self.terminal_start = time.monotonic()
                self._transition(MissionState.TERMINAL_LAND)

        elif self.state == MissionState.TERMINAL_LAND:
            if self.mode == 'STATIC':
                self._execute_blind_landing()
            else:
                self._execute_slant_landing()

        elif self.state == MissionState.DONE:
            # Mission complete — keep node alive briefly so KPI logger flushes
            if elapsed > 2.0:
                self.get_logger().info('Mission complete — shutting down node')
                raise SystemExit(0)

    # ══════════════════════════════════════════════════════════════════
    #  Mode A — STATIC blind landing (sim only, uses one-shot truth)
    # ══════════════════════════════════════════════════════════════════
    def _execute_blind_landing(self):
        if self.one_shot_truth is None:
            return

        dt_since_capture = time.monotonic() - self.one_shot_time
        t = self.one_shot_truth.twist

        ex = t.linear.x + t.angular.x * dt_since_capture
        ey = t.linear.y + t.angular.y * dt_since_capture

        dx = self.drone_state.position[0] if len(self.drone_state.position) >= 1 else 0.0
        dy = self.drone_state.position[1] if len(self.drone_state.position) >= 2 else 0.0

        err_x = ex - dx
        err_y = ey - dy
        dist = math.hypot(err_x, err_y)

        ff_pitch = t.angular.x / self.MAX_VEL
        ff_roll  = t.angular.y / self.MAX_VEL

        p_pitch = err_x / self.MAX_VEL
        p_roll  = err_y / self.MAX_VEL

        pitch_cmd = max(-1.0, min(1.0, ff_pitch + p_pitch))
        roll_cmd  = max(-1.0, min(1.0, ff_roll  + p_roll))

        if dist < 0.2:
            self.blind_plunge_active = True
        vz = self.blind_plunge_vz if self.blind_plunge_active else 0.0

        yaw_cmd = self.relative_yaw_deg * self.KP_YAW
        self._send_vel(pitch=pitch_cmd, roll=roll_cmd,
                       yaw_vel=yaw_cmd, thrust=vz)

        # Touchdown trigger handled in _mission_tick

    # ══════════════════════════════════════════════════════════════════
    #  Mode B — GIMBAL slant landing
    # ══════════════════════════════════════════════════════════════════
    def _execute_slant_landing(self):
        """Sweep gimbal from 45° (0.0) → straight down (-1.0) over
        slant_sweep_time, then begin vertical descent. Visual servo keeps
        the marker centred for the duration."""
        dt_sweep = time.monotonic() - self.terminal_start
        frac = min(dt_sweep / self.slant_sweep_time, 1.0)
        target_val = 0.0 - frac * 1.0       # 0.0 (45°) → -1.0 (down)
        self._set_gimbal(target_val)

        vz = self.descend_vz if dt_sweep > (self.slant_sweep_time + 1.0) else 0.0
        self._track_target(descend_rate=vz, kp=self.KP_LOW)
        # Touchdown trigger handled in _mission_tick

    # ══════════════════════════════════════════════════════════════════
    #  /asr/mission/state — telemetry for KPI loggers and the GUI
    # ══════════════════════════════════════════════════════════════════
    def _publish_mission_state(self):
        first_lock_ns = (self.first_lock_time.nanoseconds
                         if self.first_lock_time is not None else 0)
        payload = {
            'state':            self.state,
            'mode':             self.mode,
            'scenario':         self.scenario,
            'wind_scenario':    self.wind_scenario,
            'locked':           bool(self.locked),
            'altitude_m':       float(-self.local_pos.z),
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
    node = VisionLandingMission()
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
