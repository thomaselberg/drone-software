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
    KP_HIGH: float = 0.10   # STABILIZE_HIGH (gentle settling at altitude)
    KP_LOW:  float = 0.30   # DESCEND_TO_LOW / STABILIZE_LOW / TERMINAL_LAND
    KP_ALT:  float = 0.30   # Altitude-hold P gain
    KP_YAW:  float = 0.02   # Yaw alignment P gain
    MAX_VEL: float = 1.0    # Must match autopilot max_horizontal_velocity

    # ── Altitudes (metres, positive up) ─────────────────────────────
    takeoff_alt:          float = 1.5
    descend_alt:          float = 0.5
    terminal_alt_trigger: float = 0.4   # → hand off to PX4 land mode
    descend_vz:           float = 0.3   # m/s downward

    # ── State-machine timing ────────────────────────────────────────
    stabilize_high_time:    float = 2.0   # dwell after ground-error lower bound met
    stabilize_high_timeout: float = 10.0  # safety cap on STABILIZE_HIGH
    stabilize_low_time:     float = 1.0   # dwell at descend_alt before TERMINAL_LAND
    slant_sweep_time:       float = 5.0   # gimbal 0 → -1 sweep duration
    hold_hover_s:           float = 5.0   # HOLD phase-1 (hover) duration before descending
    coast_decay_time:       float = 2.0   # Time to decay velocity to zero after lock loss

    # ── Thresholds ──────────────────────────────────────────────────
    ground_err_thresh: float = 0.5   # SEARCH lock gate AND STABILIZE_HIGH lower bound
    search_kp:         float = 0.5   # P gain for STATIC SEARCH fly-toward
    blind_plunge_vz:   float = 0.5   # STATIC mode dead-reckon plunge speed

    # ── Default known marker position (STATIC scenario) ─────────────
    target_start_x: float = 10.0  # metres North in NED (STATIC target is always at 10m N)
    target_start_y: float = 0.0   # metres East  in NED
