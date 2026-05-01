#!/usr/bin/env python3
"""
synthetic_cam_comparison.py
---------------------------
Fair Comparison simulation engine with single 0.4m ArUco marker.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from interfaces.msg import DroneState
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time


# ---------------------------------------------------------------------------
# ArUco helpers
# ---------------------------------------------------------------------------
_ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)

def _get_marker_bits(marker_id: int) -> np.ndarray:
    return cv2.aruco.drawMarker(_ARUCO_DICT, marker_id, 8)

MARKER_BITS = _get_marker_bits(0)


class SyntheticComparisonCam(Node):
    def __init__(self):
        super().__init__('synthetic_cam_comparison')

        self.declare_parameter('target_start_x', 20.0)
        self.declare_parameter('target_start_y', 0.0)
        self.declare_parameter('target_vx_max', 0.0)
        self.declare_parameter('target_vy_max', 0.0)
        self.declare_parameter('whiteout_interval_s', 10.0)
        self.declare_parameter('whiteout_duration_s', 0.2)
        self.declare_parameter('camera_pitch_deg', 45.0)
        self.declare_parameter('marker_size_m', 0.5)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('hfov_deg', 85.0)
        self.declare_parameter('scenario', 'MOVING') # Added: STATIC or MOVING

        self.scenario = self.get_parameter('scenario').value.upper()
        self.target_x = self.get_parameter('target_start_x').value
        self.target_y = self.get_parameter('target_start_y').value
        
        if self.scenario == 'STATIC':
            self.vx_max = 0.0
            self.vy_max = 0.0
        else:
            self.vx_max   = self.get_parameter('target_vx_max').value
            self.vy_max   = self.get_parameter('target_vy_max').value
        self.whiteout_interval = self.get_parameter('whiteout_interval_s').value
        self.whiteout_duration = self.get_parameter('whiteout_duration_s').value
        self.mount_pitch = math.radians(self.get_parameter('camera_pitch_deg').value)
        self.marker_size = self.get_parameter('marker_size_m').value
        self.width  = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        hfov_rad    = math.radians(self.get_parameter('hfov_deg').value)
        self.f_px   = (self.width / 2.0) / math.tan(hfov_rad / 2.0)

        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_heading = 0.0
        self.pos = [0.0, 0.0, -5.0]
        self.att = [0.0, 0.0, 0.0]
        self.gimbal_pitch_override = None
        self.grid_spacing = 5.0
        self.grid_extent = 100.0
        self.bridge = CvBridge()

        self.pub_img = self.create_publisher(Image, '/camera/camera/color/image_raw', 10)
        self.pub_truth = self.create_publisher(TwistStamped, '/asr/sim/true_target_state', 10)
        self.create_subscription(DroneState, '/asr/thyra/out/drone_state', self._drone_cb, 10)
        self.create_subscription(Float64, '/gimbal/cmd_pitch', self._gimbal_cb, 10)

        self.timer = self.create_timer(1.0/30.0, self._tick)
        self.get_logger().info(f"Synthetic Camera started (0.4m single marker)")

    def _drone_cb(self, msg: DroneState):
        self.pos = list(msg.position)
        self.att = list(msg.orientation)

    def _gimbal_cb(self, msg: Float64):
        self.gimbal_pitch_override = math.radians(msg.data)

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
        
        # 1. Physics
        if self.vx_max != 0 or self.vy_max != 0:
            self.target_vx = self.vx_max * math.sin(now * 0.2)
            self.target_vy = self.vy_max * math.cos(now * 0.3)
            self.target_x += self.target_vx * dt
            self.target_y += self.target_vy * dt
            if abs(self.target_vx) > 0.01 or abs(self.target_vy) > 0.01:
                self.target_heading = math.atan2(self.target_vy, self.target_vx)

        # 2. Publish truth
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.twist.linear.x, ts.twist.linear.y = self.target_x, self.target_y
        ts.twist.angular.x, ts.twist.angular.y = self.target_vx, self.target_vy
        self.pub_truth.publish(ts)

        # 3. Render
        img = np.full((self.height, self.width, 3), 45, dtype=np.uint8)
        drone_pos = np.array(self.pos, dtype=np.float64)
        cam_pitch = self.gimbal_pitch_override if self.gimbal_pitch_override is not None else self.mount_pitch
        R_total = self._rot(self.att[0], self.att[1], self.att[2]) @ self._rot(0.0, cam_pitch, 0.0)

        # 3a. Grid
        grid_color = (0, 255, 255)
        ticks = np.arange(-self.grid_extent, self.grid_extent + self.grid_spacing, self.grid_spacing)
        for x in ticks:
            pts = []
            for y in np.linspace(self.pos[1]-30, self.pos[1]+30, 20):
                p = self._project_f(x, y, 0.0, R_total, drone_pos)
                if p: pts.append(p)
            if len(pts) > 1: cv2.polylines(img, [np.array(pts, np.int32)], False, grid_color, 1)
        for y in ticks:
            pts = []
            for x in np.linspace(self.pos[0]-30, self.pos[0]+30, 20):
                p = self._project_f(x, y, 0.0, R_total, drone_pos)
                if p: pts.append(p)
            if len(pts) > 1: cv2.polylines(img, [np.array(pts, np.int32)], False, grid_color, 1)

        # 3b. ArUco Marker
        half = self.marker_size / 2.0
        ch, sh = math.cos(self.target_heading), math.sin(self.target_heading)
        corners_world = [
            (self.target_x + (-half)*ch - (-half)*sh, self.target_y + (-half)*sh + (-half)*ch),
            (self.target_x + ( half)*ch - (-half)*sh, self.target_y + ( half)*sh + (-half)*ch),
            (self.target_x + ( half)*ch - ( half)*sh, self.target_y + ( half)*sh + ( half)*ch),
            (self.target_x + (-half)*ch - ( half)*sh, self.target_y + (-half)*sh + ( half)*ch),
        ]
        dst_pts = []
        all_vis = True
        for cx, cy in corners_world:
            p = self._project_f(cx, cy, 0.0, R_total, drone_pos)
            if p: dst_pts.append(p)
            else: all_vis = False; break
        
        if all_vis and len(dst_pts) == 4:
            tl, tr, br, bl = [np.array(p) for p in dst_pts]
            cv2.fillPoly(img, [np.array([tl, tr, br, bl], dtype=np.int32)], (255, 255, 255))
            for row in range(8):
                v0, v1 = row/8.0, (row+1)/8.0
                l0, l1 = tl+(bl-tl)*v0, tl+(bl-tl)*v1
                r0, r1 = tr+(br-tr)*v0, tr+(br-tr)*v1
                for col in range(8):
                    if MARKER_BITS[row, col] == 0:
                        h0, h1 = col/8.0, (col+1)/8.0
                        p0, p1 = l0+(r0-l0)*h0, l0+(r0-l0)*h1
                        p2, p3 = l1+(r1-l1)*h1, l1+(r1-l1)*h0
                        cv2.fillPoly(img, [np.array([p0, p1, p2, p3], dtype=np.int32)], (0, 0, 0))

        # 4. Whiteout
        if self.whiteout_duration > 0:
            if (now % self.whiteout_interval) < self.whiteout_duration:
                img[:,:] = 255

        # 5. Publish
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))

def main():
    rclpy.init()
    node = SyntheticComparisonCam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
