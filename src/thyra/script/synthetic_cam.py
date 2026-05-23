#!/usr/bin/env python3
"""
synthetic_cam.py
----------------
Sim-only synthetic camera. Renders a single 0.6 m ArUco marker on a
ground plane from the drone's current pose, publishes the resulting
image on /camera/camera/color/image_raw, and publishes ground truth
on /asr/sim/true_target_state.

Scenarios:
  STATIC       Target never moves.
  MOVING       Target drifts North at ~0.2–0.4 m/s with small lateral
               sinusoidal wiggle; triggered once the drone passes 2 m AGL.
  MOVING_EASY  Same motion model, scaled by 0.1 across the board (slow
               target for tuning).

MOVING velocity model:
  North:   ((sin + 1) · vx_max / 2 + 0.2) · vel_scale, always > 0.
  Lateral: 3 overlapping sinusoids with irrational frequency ratios
           (π/7, √2/3, e/11), weighted 50/30/20 %, scaled by vel_scale.
           Clamped to ±0.25 × forward velocity (max ~14° turns).

Gimbal: subscribes to the raw-servo topic /asr/thyra/in/servo_command
(same topic the mission and the GUI slider publish to) and maps the raw
servo value to camera pitch via the measured calibration endpoints —
gimbal_down_cmd → straight down (0 rad), gimbal_45_cmd → 45° (π/4 rad).

The camera is rendered from drone_pos + R_drone @ [cam_offset_x, 0, 0],
so body pitch/roll/yaw shift the camera viewpoint just like the real
forward-mounted RealSense.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from interfaces.msg import DroneState, ServoCommand
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time


# ---------------------------------------------------------------------------
# ArUco helpers
# ---------------------------------------------------------------------------
_ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Pre-rendered high-resolution marker, used as the source for a per-frame
# perspective warp. The previous renderer drew individual fillPoly bit
# cells whose int32 rounding introduced sub-pixel gaps/overlaps at slants
# — the detector's adaptive threshold then failed to decode the pattern
# when the drone got close at 45°. With a warp from a clean reference
# image, edges are interpolated correctly at any pose.
# 180 = 6×30: a 4X4 marker is a 6×6 cell grid, so this gives clean cells.
_MARKER_PX  = 180
MARKER_IMG  = cv2.cvtColor(
    cv2.aruco.drawMarker(_ARUCO_DICT, 0, _MARKER_PX),
    cv2.COLOR_GRAY2BGR)
_SRC_CORNERS = np.array([
    [0,             0            ],
    [_MARKER_PX-1,  0            ],
    [_MARKER_PX-1,  _MARKER_PX-1 ],
    [0,             _MARKER_PX-1 ],
], dtype=np.float32)


class SyntheticCam(Node):
    def __init__(self):
        super().__init__('synthetic_cam')

        self.declare_parameter('target_start_x', 20.0)
        self.declare_parameter('target_start_y', 0.0)
        self.declare_parameter('whiteout_interval_s', 10.0)
        self.declare_parameter('whiteout_duration_s', 0.2)
        self.declare_parameter('camera_pitch_deg', 45.0)
        self.declare_parameter('marker_size_m', 0.3)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('hfov_deg', 85.0)
        self.declare_parameter('scenario', 'MOVING')  # STATIC, MOVING, or MOVING_EASY
        self.declare_parameter('cam_offset_x', 0.15)  # camera fwd of drone center [m]
        # Gimbal servo calibration — must match MissionParams.gimbal_*.
        # The /asr/thyra/in/servo_command topic carries raw servo values;
        # these endpoints map a raw value to the camera pitch.
        self.declare_parameter('gimbal_down_cmd', -0.95)  # raw servo → straight down
        self.declare_parameter('gimbal_45_cmd',    0.10)  # raw servo → 45° slant

        self.scenario = self.get_parameter('scenario').value.upper()
        self.target_x = self.get_parameter('target_start_x').value
        self.target_y = self.get_parameter('target_start_y').value
        self.cam_offset_x    = self.get_parameter('cam_offset_x').value
        self.gimbal_down_cmd = self.get_parameter('gimbal_down_cmd').value
        self.gimbal_45_cmd   = self.get_parameter('gimbal_45_cmd').value

        self.whiteout_interval = self.get_parameter('whiteout_interval_s').value
        self.whiteout_duration = self.get_parameter('whiteout_duration_s').value
        self.mount_pitch = math.radians(self.get_parameter('camera_pitch_deg').value)
        self.marker_size = self.get_parameter('marker_size_m').value
        self.width  = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        hfov_rad    = math.radians(self.get_parameter('hfov_deg').value)
        self.f_px   = (self.width / 2.0) / math.tan(hfov_rad / 2.0)

        # DYNAMIC velocity constants
        self.VX_MAX = 0.2  # m/s max forward (North) velocity

        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_heading = 0.0
        self.pos = [0.0, 0.0, 0.0]
        self.att = [0.0, 0.0, 0.0]
        self.gimbal_pitch_override = None
        self.grid_spacing = 5.0
        self.grid_extent = 100.0
        self.bridge = CvBridge()

        # Altitude trigger for DYNAMIC target
        self.target_started = False
        self.drone_state_received = False

        self.pub_img = self.create_publisher(Image, '/camera/camera/color/image_raw', 1)
        self.pub_truth = self.create_publisher(TwistStamped, '/asr/sim/true_target_state', 1)
        self.create_subscription(DroneState, '/asr/thyra/out/drone_state', self._drone_cb, 10)
        # Same raw-servo topic the mission and the GUI slider publish to.
        # BEST_EFFORT QoS accepts both the GUI's BEST_EFFORT publisher and
        # the mission's default-RELIABLE publisher.
        self.create_subscription(
            ServoCommand, '/asr/thyra/in/servo_command',
            self._gimbal_cb, qos_profile_sensor_data)

        self.timer = self.create_timer(1.0/30.0, self._tick)
        self.get_logger().info(
            f"Synthetic Camera started ({self.marker_size}m marker, scenario={self.scenario})")

    def _drone_cb(self, msg: DroneState):
        self.pos = list(msg.position)
        self.att = list(msg.orientation)
        self.drone_state_received = True

    def _gimbal_cb(self, msg: ServoCommand):
        # Raw servo command (GUI slider or mission). Map it to camera pitch
        # via the measured endpoints: gimbal_down_cmd → straight down (0 rad),
        # gimbal_45_cmd → 45° (π/4). A raw value beyond gimbal_down_cmd maps
        # to a slightly-backward tilt (negative pitch). Gimbal is AUX1.
        if msg.aux_index != 0:
            return
        span = self.gimbal_45_cmd - self.gimbal_down_cmd
        self.gimbal_pitch_override = (math.pi / 4.0) * (msg.value - self.gimbal_down_cmd) / span

    def _rot(self, r, p, y):
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        return Rz @ Ry @ Rx

    def _project_f(self, x, y, z, R_total, drone_pos):
        rel_world = np.array([x - drone_pos[0], y - drone_pos[1], z - drone_pos[2]])
        cam_vec = R_total.T @ rel_world
        if cam_vec[2] <= 0.1: return None
        u = (cam_vec[1] / cam_vec[2]) * self.f_px + self.width / 2.0
        v = (-cam_vec[0] / cam_vec[2]) * self.f_px + self.height / 2.0
        return (u, v)

    def _tick(self):
        dt = 1.0/30.0
        now = self.get_clock().now().nanoseconds / 1e9
        drone_alt = -self.pos[2]  # AGL altitude

        # 1. Physics — DYNAMIC target movement
        if self.scenario in ('MOVING', 'MOVING_EASY'):
            # MOVING_EASY runs the same motion model 10× slower across the
            # board (forward drift, lateral sinusoids, and the constant
            # north creep). Lateral/forward ratio is preserved, so turn
            # angles match MOVING; only the magnitude shrinks.
            vel_scale = 0.25 if self.scenario == 'MOVING_EASY' else 1.0

            # Check altitude trigger — only after first real telemetry
            if not self.target_started:
                if self.drone_state_received and drone_alt >= 2.0:
                    self.target_started = True
                    self.get_logger().info(
                        f'TARGET TRIGGERED: Drone crossed 2.0m (alt={drone_alt:.2f}m)')

            if self.target_started:
                # North velocity: always forward, never negative
                # (sin + 1.0) * (vx_max / 2.0) → [0, vx_max], plus a small
                # constant so the target never stops entirely.
                self.target_vx = ((math.sin(now * 0.2) + 1.0) * (self.VX_MAX / 2.0) + 0.2) * vel_scale

                # Lateral velocity: three overlapping sinusoids with irrational
                # frequency ratios — smooth but effectively never-repeating
                vy_raw = (0.125 * math.sin(now * math.pi / 7.0)
                        + 0.075 * math.sin(now * math.sqrt(2.0) / 3.0)
                        + 0.05  * math.sin(now * math.e / 11.0)) * vel_scale

                # Clamp lateral to ±0.25 × forward velocity (max ~14° turns)
                vy_limit = 0.25 * self.target_vx
                self.target_vy = max(-vy_limit, min(vy_limit, vy_raw))

                # Integrate position
                self.target_x += self.target_vx * dt
                self.target_y += self.target_vy * dt

                # Update heading from velocity vector
                if abs(self.target_vx) > 0.01 or abs(self.target_vy) > 0.01:
                    self.target_heading = math.atan2(self.target_vy, self.target_vx)
            else:
                # Target stationary until triggered
                self.target_vx = 0.0
                self.target_vy = 0.0
        else:
            # STATIC scenario — target never moves
            self.target_vx = 0.0
            self.target_vy = 0.0

        # 2. Publish truth
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.twist.linear.x, ts.twist.linear.y = self.target_x, self.target_y
        ts.twist.angular.x, ts.twist.angular.y = self.target_vx, self.target_vy
        ts.twist.angular.z = self.target_heading
        self.pub_truth.publish(ts)

        # 3. Render
        img = np.full((self.height, self.width, 3), 45, dtype=np.uint8)
        drone_pos = np.array(self.pos, dtype=np.float64)
        cam_pitch = self.gimbal_pitch_override if self.gimbal_pitch_override is not None else self.mount_pitch
        R_drone = self._rot(self.att[0], self.att[1], self.att[2])
        R_total = R_drone @ self._rot(0.0, cam_pitch, 0.0)

        # Camera sits cam_offset_x forward of drone center (body frame).
        # Rotating through the drone attitude captures every translation
        # effect of body motion: pitch shifts it mostly in Z, yaw traces a
        # horizontal arc in X/Y, roll does nothing (camera is on the roll
        # axis). The gimbal only rotates the optical axis, not the body.
        cam_pos = drone_pos + R_drone @ np.array([self.cam_offset_x, 0.0, 0.0])

        # 3a. Grid
        grid_color = (0, 255, 255)
        ticks = np.arange(-self.grid_extent, self.grid_extent + self.grid_spacing, self.grid_spacing)
        for x in ticks:
            pts = []
            for y in np.linspace(self.pos[1]-30, self.pos[1]+30, 20):
                p = self._project_f(x, y, 0.0, R_total, cam_pos)
                if p: pts.append(p)
            if len(pts) > 1: cv2.polylines(img, [np.array(pts, np.int32)], False, grid_color, 1)
        for y in ticks:
            pts = []
            for x in np.linspace(self.pos[0]-30, self.pos[0]+30, 20):
                p = self._project_f(x, y, 0.0, R_total, cam_pos)
                if p: pts.append(p)
            if len(pts) > 1: cv2.polylines(img, [np.array(pts, np.int32)], False, grid_color, 1)

        # 3b. ArUco Marker — TL/TR edge = FORWARD (velocity direction)
        #
        # Corner ordering (canonical ArUco):
        #   TL=0  TR=1
        #   BL=3  BR=2
        #
        # With heading=0 (velocity → North/+X):
        #   TL = center + (+half_fwd, -half_right)  = forward-left
        #   TR = center + (+half_fwd, +half_right)  = forward-right
        #   BR = center + (-half_fwd, +half_right)  = back-right
        #   BL = center + (-half_fwd, -half_right)  = back-left
        half = self.marker_size / 2.0
        ch = math.cos(self.target_heading)
        sh = math.sin(self.target_heading)

        # local offsets: (along_forward, across_right) → world NED
        local_corners = [
            (+half, -half),  # TL: forward, left
            (+half, +half),  # TR: forward, right
            (-half, +half),  # BR: back, right
            (-half, -half),  # BL: back, left
        ]
        corners_world = []
        for dx_fwd, dy_right in local_corners:
            wx = self.target_x + dx_fwd * ch - dy_right * sh
            wy = self.target_y + dx_fwd * sh + dy_right * ch
            corners_world.append((wx, wy))

        dst_pts = []
        all_vis = True
        for cx, cy in corners_world:
            p = self._project_f(cx, cy, 0.0, R_total, cam_pos)
            if p: dst_pts.append(p)
            else: all_vis = False; break

        if all_vis and len(dst_pts) == 4:
            # Warp the high-resolution reference marker onto the projected
            # quad. BORDER_TRANSPARENT leaves pixels outside the warped
            # region untouched, so the scene background (grid + base color)
            # stays visible without an explicit mask.
            H = cv2.getPerspectiveTransform(
                _SRC_CORNERS, np.array(dst_pts, dtype=np.float32))
            cv2.warpPerspective(
                MARKER_IMG, H, (self.width, self.height),
                dst=img, borderMode=cv2.BORDER_TRANSPARENT,
                flags=cv2.INTER_LINEAR)

        # 4. Whiteout
        if self.whiteout_duration > 0:
            if (now % self.whiteout_interval) < self.whiteout_duration:
                img[:,:] = 255

        # 5. Publish
        try:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))
        except Exception:
            pass


def main():
    rclpy.init()
    node = SyntheticCam()
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
