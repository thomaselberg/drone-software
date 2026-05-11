#!/usr/bin/env python3
"""
bench_dry_test.py
-----------------
Walks the exact same state machine as vision_landing_mission.py but never
arms the drone, never sends DroneCommand actions, and never publishes
/asr/thyra/in/manual_input. Used for bench / no-prop tests where the
operator holds the drone by hand and the camera+detector run normally.

Run alongside the real-flight base stack:
  Terminal A:  ./start_base.sh
  Terminal B:  ros2 run thyra bench_dry_test.py --ros-args \
                   -p mode:=GIMBAL -p scenario:=DYNAMIC

State names, dwell logic, lock-loss HOLD behavior, and tuning constants
are byte-identical to vision_landing_mission.py — both import the same
thyra.mission_params.MissionParams. The only difference is that every
velocity/thrust command and every DroneCommand action is logged ("would
have sent ...") instead of dispatched. The /asr/mission/state JSON
includes last_cmd_pitch / roll / yaw_vel / thrust so kpi_logger_real
records what the controller would have done.

Altitude is simulated (virt_alt) because the drone is stationary on the
bench. The detector still uses the real camera, so ground_err and lock
state come from a real marker observed by the held drone.
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from interfaces.msg import GcsHeartbeat, DroneState, ServoCommand
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64, String
from px4_msgs.msg import VehicleLocalPosition

from thyra.mission_params import MissionParams


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


_TRACKING_STATES = (
    MissionState.STABILIZE_HIGH,
    MissionState.DESCEND_TO_LOW,
    MissionState.STABILIZE_LOW,
    MissionState.TERMINAL_LAND,
)


class BenchDryTest(Node):
    """No-motor walkthrough of the real-flight state machine."""

    # Bench-only timings (not in MissionParams — only the dry test needs these
    # because the real autopilot's arm/takeoff actions handle these phases on
    # the real drone).
    BENCH_IDLE_TIME         = 2.0   # IDLE dwell before simulated arm
    BENCH_TAKEOFF_SIM_TIME  = 3.0   # virt_alt ramps 0 → takeoff_alt over this

    def __init__(self):
        super().__init__('bench_dry_test')

        # ── Runtime config (mirror vision_landing_mission) ────────────
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
        self.terminal_trig = self.terminal_alt_trigger

        # ── QoS ───────────────────────────────────────────────────────
        qos_hb = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        # ── Publishers (no /asr/thyra/in/manual_input — bench never
        #    commands the motors) ──────────────────────────────────────
        self.pub_heartbeat = self.create_publisher(
            GcsHeartbeat, '/asr/thyra/in/gcs_heartbeat', qos_hb)
        self.pub_gimbal = self.create_publisher(
            Float64, '/gimbal/cmd_pitch', 10)
        self.pub_servo = self.create_publisher(
            ServoCommand, '/asr/thyra/in/servo_command', 10)
        self.pub_state = self.create_publisher(
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

        # ── State variables ───────────────────────────────────────────
        self.state         = MissionState.IDLE
        self.state_start   = self.get_clock().now()
        self.return_state  = None

        self.drone_state = DroneState()
        self.local_pos   = VehicleLocalPosition()

        self.pixel_err_x      = 0.0
        self.pixel_err_y      = 0.0
        self.locked           = False
        self.first_lock_time  = None
        self.relative_yaw_deg = 0.0

        self.gimbal_angle_norm = 0.0
        self.last_cmd_pitch    = 0.0
        self.last_cmd_roll     = 0.0
        self.last_cmd_yaw      = 0.0
        self.last_cmd_thrust   = 0.0

        # Simulated altitude (drone is stationary on the bench)
        self.virt_alt = 0.0

        # HOLD bookkeeping
        self.lock_loss_start    = None
        self.lock_loss_alt      = None
        self.hold_descent_start = None

        # STABILIZE_HIGH dwell bookkeeping
        self.stab_high_threshold_seen = False
        self.stab_high_dwell_start    = None

        # TERMINAL_LAND simulation timers
        self.terminal_start          = None
        self.terminal_descent_start  = None

        # ── Timers ────────────────────────────────────────────────────
        self.create_timer(0.1, self._heartbeat_tick)
        self.create_timer(0.05, self._mission_tick)
        self.create_timer(0.1, self._publish_mission_state)

        self.get_logger().info(
            f'BenchDryTest started — motors will NOT be commanded.  '
            f'mode={self.mode}  scenario={self.scenario}  '
            f'takeoff_alt={self.takeoff_alt:.2f}m')

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
        was_locked = self.locked
        self.locked = msg.vector.z > 0.5
        if self.locked and not was_locked and self.first_lock_time is None:
            self.first_lock_time = self.get_clock().now()
        try:
            self.relative_yaw_deg = float(msg.header.frame_id)
        except (ValueError, TypeError):
            pass

    # ══════════════════════════════════════════════════════════════════
    #  Helpers
    # ══════════════════════════════════════════════════════════════════
    def _heartbeat_tick(self):
        msg = GcsHeartbeat()
        msg.timestamp = float(self.get_clock().now().nanoseconds / 1e9)
        self.pub_heartbeat.publish(msg)

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

    def _log_cmd(self, pitch=0.0, roll=0.0, yaw_vel=0.0, thrust=0.0):
        """Capture and log the velocity/thrust command that would have been
        sent on /asr/thyra/in/manual_input in real flight."""
        pitch   = max(-1.0, min(1.0, pitch))
        roll    = max(-1.0, min(1.0, roll))
        yaw_vel = max(-1.0, min(1.0, yaw_vel))
        thrust  = max(-1.0, min(1.0, thrust))
        self.last_cmd_pitch  = pitch
        self.last_cmd_roll   = roll
        self.last_cmd_yaw    = yaw_vel
        self.last_cmd_thrust = thrust
        self.get_logger().info(
            f'[DRY] {self.state:<14s} virt_alt={self.virt_alt:.2f}m '
            f'gimbal={self.gimbal_angle_norm:+.2f} '
            f'err=({self.pixel_err_x:+.2f},{self.pixel_err_y:+.2f}) '
            f'lock={int(self.locked)} cmd: pitch={pitch:+.3f} '
            f'roll={roll:+.3f} yaw_vel={yaw_vel:+.3f} thrust={thrust:+.3f}',
            throttle_duration_sec=0.5)

    def _log_action(self, cmd_type, target_pose=None):
        """Log the DroneCommand action that would have been sent."""
        self.get_logger().info(
            f'[DRY] {self.state:<14s} would have sent action: {cmd_type} '
            f'pose={target_pose}')

    def _elapsed(self):
        return (self.get_clock().now() - self.state_start).nanoseconds / 1e9

    def _transition(self, new_state):
        self.get_logger().info(f'STATE: {self.state} → {new_state}')
        if new_state == MissionState.STABILIZE_HIGH:
            self.stab_high_threshold_seen = False
            self.stab_high_dwell_start    = None
        if new_state == MissionState.TERMINAL_LAND:
            self.terminal_start         = self.get_clock().now()
            self.terminal_descent_start = None
        self.state       = new_state
        self.state_start = self.get_clock().now()

    # ══════════════════════════════════════════════════════════════════
    #  Controllers (mirror vision_landing_mission)
    # ══════════════════════════════════════════════════════════════════
    def _alt_hold_thrust(self, target_alt):
        alt_err = target_alt - self.virt_alt
        return -self.KP_ALT * alt_err

    def _track_target(self, descend_rate=0.0, kp=None):
        if kp is None:
            kp = self.KP_LOW
        if not self.locked:
            # Lock-loss in tracking states is normally caught by _enter_hold
            # in the central tick; this branch is the fall-back for STATIC
            # TERMINAL_LAND (intentionally lock-free).
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

    # ══════════════════════════════════════════════════════════════════
    #  HOLD (lock-loss recovery — mirrors vision_landing_mission)
    # ══════════════════════════════════════════════════════════════════
    def _enter_hold(self):
        if self.state == MissionState.HOLD:
            return
        self.return_state    = self.state
        self.lock_loss_start = self.get_clock().now()
        self.lock_loss_alt   = self.virt_alt
        self.hold_descent_start = None
        self._transition(MissionState.HOLD)
        self.get_logger().warn(
            f'[DRY] LOCK LOST in {self.return_state} at virt_alt={self.lock_loss_alt:.2f}m '
            f'→ HOLD (hover {self.hold_hover_s:.0f}s, then descend)')

    def _tick_hold(self):
        # Re-lock → resume previous state
        if self.locked and self.return_state is not None:
            self.get_logger().info(f'[DRY] LOCK RECOVERED → resuming {self.return_state}')
            self._transition(self.return_state)
            self.return_state    = None
            self.lock_loss_start = None
            self.lock_loss_alt   = None
            return

        elapsed_lost = (self.get_clock().now() - self.lock_loss_start).nanoseconds / 1e9

        # Touchdown handoff
        if self.virt_alt < self.terminal_trig:
            self._log_action('land')
            self._transition(MissionState.DONE)
            return

        if elapsed_lost < self.hold_hover_s:
            # Phase 1: hover at altitude of loss
            self.virt_alt = self.lock_loss_alt
            thrust = self._alt_hold_thrust(self.lock_loss_alt)
            self._log_cmd(0.0, 0.0, 0.0, thrust)
        else:
            # Phase 2: controlled descent at descend_vz
            if self.hold_descent_start is None:
                self.hold_descent_start = self.get_clock().now()
            descent_s = (self.get_clock().now() - self.hold_descent_start).nanoseconds / 1e9
            self.virt_alt = max(0.0, self.lock_loss_alt - self.descend_vz * descent_s)
            self._log_cmd(0.0, 0.0, 0.0, self.descend_vz)

    # ══════════════════════════════════════════════════════════════════
    #  Main state machine — line-for-line mirror of vision_landing_mission
    # ══════════════════════════════════════════════════════════════════
    def _mission_tick(self):
        # HOLD dispatcher
        if self.state == MissionState.HOLD:
            self._tick_hold()
            return

        # Lock-loss check (STATIC TERMINAL_LAND is intentionally lock-free)
        if self.state in _TRACKING_STATES and not self.locked:
            in_static_terminal = (self.state == MissionState.TERMINAL_LAND
                                  and self.mode == 'STATIC')
            if not in_static_terminal:
                self._enter_hold()
                return

        elapsed = self._elapsed()

        # Touchdown trigger common to descent states
        if self.state in (MissionState.DESCEND_TO_LOW,
                          MissionState.STABILIZE_LOW,
                          MissionState.TERMINAL_LAND):
            if self.virt_alt < self.terminal_trig:
                self._log_action('land')
                self._transition(MissionState.DONE)
                return

        # ── State dispatch ───────────────────────────────────────────
        if self.state == MissionState.IDLE:
            self.virt_alt = 0.0
            self._set_gimbal(0.0)
            self._log_cmd(0.0, 0.0, 0.0, 0.0)
            if elapsed > self.BENCH_IDLE_TIME:
                self._log_action('arm')
                self._transition(MissionState.ARMING)

        elif self.state == MissionState.ARMING:
            # Real flight waits for arming_state==1. Bench transitions
            # immediately — there's no autopilot to arm.
            self.virt_alt = 0.0
            self._set_gimbal(0.0)
            self._log_cmd(0.0, 0.0, 0.0, 0.0)
            self._log_action('takeoff', [-self.takeoff_alt])
            self._transition(MissionState.TAKEOFF)

        elif self.state == MissionState.TAKEOFF:
            frac = min(elapsed / self.BENCH_TAKEOFF_SIM_TIME, 1.0)
            self.virt_alt = self.takeoff_alt * frac
            self._set_gimbal(0.0)
            thrust = self._alt_hold_thrust(self.takeoff_alt)
            self._log_cmd(0.0, 0.0, 0.0, thrust)
            if frac >= 1.0:
                self._log_action('manual_aided')
                self._set_gimbal(0.0)  # 45° gimbal mount angle
                self._transition(MissionState.SEARCH)

        elif self.state == MissionState.SEARCH:
            self.virt_alt = self.takeoff_alt
            self._set_gimbal(0.0)
            if self.scenario == 'DYNAMIC':
                thrust = self._alt_hold_thrust(self.takeoff_alt)
                self._log_cmd(0.0, 0.0, 0.0, thrust)
            else:
                # STATIC: simulated fly-toward the known marker location.
                # No real drone position to drive the controller off, so we
                # log a fixed commanded velocity proportional to the marker
                # offset from origin. Operator can interpret this.
                err_x = self.target_start_x
                err_y = self.target_start_y
                thrust = self._alt_hold_thrust(self.takeoff_alt)
                self._log_cmd(err_x * self.search_kp,
                              err_y * self.search_kp, 0.0, thrust)

            if self.locked:
                d_ground = math.hypot(self.pixel_err_x, self.pixel_err_y)
                if d_ground < 3.0:
                    self.get_logger().info(
                        f'[DRY] ArUco LOCKED & ground err {d_ground:.2f}m < 3m')
                    if self.scenario == 'DYNAMIC':
                        self._transition(MissionState.STABILIZE_HIGH)
                    else:
                        self._transition(MissionState.DESCEND_TO_LOW)

        elif self.state == MissionState.STABILIZE_HIGH:
            self.virt_alt = self.takeoff_alt
            thrust = self._alt_hold_thrust(self.takeoff_alt)
            self._track_target(descend_rate=thrust, kp=self.KP_HIGH)
            d_ground = math.hypot(self.pixel_err_x, self.pixel_err_y)

            # Lower bound: arm the dwell timer the first time ground_err
            # drops below threshold.
            if not self.stab_high_threshold_seen and d_ground < self.ground_err_thresh:
                self.stab_high_threshold_seen = True
                self.stab_high_dwell_start    = self.get_clock().now()
                self.get_logger().info(
                    f'[DRY] Ground err {d_ground:.2f}m < {self.ground_err_thresh:.2f}m '
                    f'— STABILIZE_HIGH dwell started ({self.stabilize_high_time:.1f}s)')

            if self.stab_high_threshold_seen:
                dwell_s = (self.get_clock().now() - self.stab_high_dwell_start).nanoseconds / 1e9
                if dwell_s >= self.stabilize_high_time:
                    self.get_logger().info(
                        f'[DRY] STABILIZE_HIGH dwell complete ({dwell_s:.2f}s) '
                        f'→ DESCEND_TO_LOW')
                    self._transition(MissionState.DESCEND_TO_LOW)
                    return

            if elapsed >= self.stabilize_high_timeout:
                self.get_logger().warn(
                    f'[DRY] STABILIZE_HIGH timeout ({self.stabilize_high_timeout:.1f}s, '
                    f'ground err {d_ground:.2f}m, '
                    f'threshold_seen={self.stab_high_threshold_seen}) '
                    f'→ forcing descent')
                self._transition(MissionState.DESCEND_TO_LOW)

        elif self.state == MissionState.DESCEND_TO_LOW:
            self._set_gimbal(0.0)
            # Simulate linear descent at descend_vz, floor at descend_alt
            self.virt_alt = max(self.descend_alt,
                                self.takeoff_alt - self.descend_vz * elapsed)
            if self.virt_alt <= self.descend_alt + 0.05:
                self.get_logger().info(
                    f'[DRY] Reached {self.descend_alt}m → STABILIZE_LOW')
                self._transition(MissionState.STABILIZE_LOW)
            else:
                self._track_target(descend_rate=self.descend_vz, kp=self.KP_LOW)

        elif self.state == MissionState.STABILIZE_LOW:
            self.virt_alt = self.descend_alt
            self._set_gimbal(0.0)
            thrust = self._alt_hold_thrust(self.descend_alt)
            self._track_target(descend_rate=thrust, kp=self.KP_LOW)
            if elapsed >= self.stabilize_low_time:
                self.get_logger().info('[DRY] Stable at low alt → TERMINAL_LAND')
                self._transition(MissionState.TERMINAL_LAND)

        elif self.state == MissionState.TERMINAL_LAND:
            # Gimbal slant sweep (same shape as real)
            dt_sweep = (self.get_clock().now() - self.terminal_start).nanoseconds / 1e9
            frac = min(dt_sweep / self.slant_sweep_time, 1.0)
            self._set_gimbal(0.0 - frac * 1.0)

            if dt_sweep > (self.slant_sweep_time + 1.0):
                # Simulated descent at descend_vz after sweep
                if self.terminal_descent_start is None:
                    self.terminal_descent_start = self.get_clock().now()
                descent_s = (self.get_clock().now() - self.terminal_descent_start).nanoseconds / 1e9
                self.virt_alt = max(0.0, self.descend_alt - self.descend_vz * descent_s)
                self._track_target(descend_rate=self.descend_vz, kp=self.KP_LOW)
            else:
                # Hold at descend_alt during the sweep (matches real flight
                # behavior: descend_rate=0 in this phase)
                self.virt_alt = self.descend_alt
                self._track_target(descend_rate=0.0, kp=self.KP_LOW)
            # Touchdown trigger handled centrally above.

        elif self.state == MissionState.DONE:
            self._log_cmd(0.0, 0.0, 0.0, 0.0)
            if elapsed > 2.0:
                self.get_logger().info('[DRY] Bench dry test complete — shutting down.')
                raise SystemExit(0)

    # ══════════════════════════════════════════════════════════════════
    #  /asr/mission/state — telemetry for KPI loggers
    # ══════════════════════════════════════════════════════════════════
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
            'last_cmd_yaw_vel': float(self.last_cmd_yaw),
            'last_cmd_thrust':  float(self.last_cmd_thrust),
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
