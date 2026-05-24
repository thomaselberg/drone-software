"""
Shared tuning + behavior constants for vision_landing_mission.py and
bench_dry_test.py.

Both scripts import this module so the algorithm runs with identical
numbers in sim, on the real drone, and in bench-dry mode. ROS params
override these defaults at runtime via the apply_overrides() helper on
each node.

Convention: this dataclass holds the *defaults*. The ROS parameter
declarations live in the consuming nodes and shadow these defaults
when set via --ros-args -p name:=value.
"""
from dataclasses import dataclass


@dataclass
class MissionParams:
    # ── Controller gains ────────────────────────────────────────────
    # KP is linearly interpolated by altitude: KP_HIGH at takeoff_alt,
    # KP_LOW at terminal_alt_trigger. Larger gains near the ground
    # compensate for the shrinking pixel→world conversion.
    KP_HIGH: float = 0.5   # P-gain at takeoff_alt (interpolation top end)
    KP_LOW:  float = 1.0    # P-gain at terminal_alt_trigger (bottom end)
    KP_ALT:  float = 0.30   # Altitude-hold P gain
    KP_YAW:  float = 0.04   # Yaw alignment P gain

    # ── Altitudes (metres, positive up) ─────────────────────────────
    takeoff_alt:          float = 2.0
    descend_alt:          float = 1.0
    terminal_alt_trigger: float = 0.4   # → hand off to PX4 land mode
    descend_vz:           float = 0.3   # m/s downward

    # ── State-machine timing ────────────────────────────────────────
    stabilize_high_time:    float = 0.5   # dwell after ground-error lower bound met
    stabilize_high_timeout: float = 2.0  # safety cap on STABILIZE_HIGH
    stabilize_low_time:     float = 1.0   # dwell at descend_alt before TERMINAL_LAND
    slant_sweep_time:       float = 5.0   # gimbal 0 → -1 sweep duration
    hold_hover_s:           float = 1.0   # HOLD phase-1 (hover) duration before descending
    coast_decay_time:       float = 4.0   # Time to decay velocity to zero after lock loss
    lpos_timeout:           float = 0.5   # local_pos staleness threshold (mid-flight loss)
    lpos_grace_s:           float = 5.0   # level-hold time before failsafe land on stale local_pos

    # ── Thresholds ──────────────────────────────────────────────────
    ground_err_thresh: float = 1.0   # SEARCH lock gate AND STABILIZE_HIGH lower bound
    search_kp:         float = 0.5   # P gain for STATIC SEARCH fly-toward
    blind_plunge_vz:   float = 0.5   # STATIC mode dead-reckon plunge speed

    # ── Default known marker position (STATIC scenario) ─────────────
    target_start_x: float = 10.0  # metres North in NED (STATIC target is always at 10m N)
    target_start_y: float = 0.0   # metres East  in NED

    # ── Hardware geometry ───────────────────────────────────────────
    # Camera body-frame offset from drone COM (FRD: +x fwd, +y right,
    # +z down). Mounted 12 cm forward, 2.5 cm left (→ y = -0.025), and
    # 6 cm below COM. The controller biases the visual servo by the
    # (x,y) component, rotated into the marker frame, so the drone
    # *center* — not the camera — ends up over the target. Detector
    # and synthetic cam use the full 3-vector to compute the actual
    # camera world position so sim and real agree.
    cam_offset_x: float =  0.12
    cam_offset_y: float = -0.025
    cam_offset_z: float =  0.06

    # ── Gimbal servo calibration (real flight only) ─────────────────
    # The mission's logical gimbal value is -1.0=down, 0.0=45°, +1.0=
    # horizon. _set_gimbal linearly maps the logical value to the
    # measured servo command for the ServoCommand topic only; the
    # logical value still goes out on /gimbal/cmd_pitch unchanged, so
    # the synthetic cam is unaffected. Re-measure and override these
    # via ROS params before a real flight if the servo shifts.
    # Calibrated 2026-05-24: -0.860 = down, +0.10 = 45° slant.
    gimbal_down_cmd: float = -0.860  # servo value for straight-down
    gimbal_45_cmd:   float =  0.10   # servo value for 45° slant
