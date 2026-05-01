#!/usr/bin/env python3
"""
aruco_detector_comparison.py
----------------------------
ArUco detection node for the fair comparison scenario.

Responsibility:
  1. Subscribe to the synthetic camera image feed.
  2. Detect ArUco marker (ID 0, DICT_6X6_50).
  3. Compute normalized pixel error ([-1, 1] range).
  4. Estimate relative yaw from marker corner geometry.
  5. Publish detection result as a Vector3Stamped:
       x = normalized horizontal pixel error  (-1 left .. +1 right)
       y = normalized vertical   pixel error  (-1 top  .. +1 bottom)
       z = lock flag (1.0 = locked, 0.0 = no detection)
     header.frame_id encodes the relative yaw as a string (degrees).

Publishers:
  /asr/comparison/aruco_pixel_error  (geometry_msgs/Vector3Stamped)

Subscribers:
  /asr/sim/synthetic_camera/image    (sensor_msgs/Image)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from std_msgs.msg import Float64
from interfaces.msg import DroneState


_ARUCO_DICT   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
_ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
_ARUCO_PARAMS.minMarkerPerimeterRate = 0.01
_ARUCO_PARAMS.perspectiveRemovePixelPerCell = 4


class ArucoDetectorComparison(Node):
    """Detects ArUco ID 0 and publishes normalized pixel error."""

    def __init__(self):
        super().__init__('aruco_detector_comparison')
        self.bridge = CvBridge()

        # State Variables
        self.drone_pos = [0.0, 0.0, 0.0]
        self.drone_att = [0.0, 0.0, 0.0]
        self.mount_pitch = math.radians(45.0)  # Default
        self.gimbal_pitch = None

        # Camera Intrinsics (matched to synthetic_cam_comparison)
        self.w = 640
        self.h = 480
        hfov_rad = math.radians(85.0)
        self.f_px = (self.w / 2.0) / math.tan(hfov_rad / 2.0)

        # Publisher
        self.pub_error = self.create_publisher(
            Vector3Stamped, '/asr/comparison/aruco_pixel_error', 10)

        # Subscribers
        self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            self._image_cb, qos_profile_sensor_data)
        self.create_subscription(
            DroneState, '/asr/thyra/out/drone_state', self._drone_cb, 10)
        self.create_subscription(
            Float64, '/gimbal/cmd_pitch', self._gimbal_cb, 10)

        self.get_logger().info('ArUco detector (comparison) started with GROUND PROJECTION')

    def _drone_cb(self, msg: DroneState):
        self.drone_pos = list(msg.position)
        self.drone_att = list(msg.orientation)

    def _gimbal_cb(self, msg: Float64):
        self.gimbal_pitch = math.radians(msg.data)

    def _to_ground(self, u, v, alt):
        """Project pixel (u,v) to ground plane NED meters relative to drone."""
        # 1. Image to camera-frame unit vector
        x_c = (u - self.w/2.0) / self.f_px
        y_c = (v - self.h/2.0) / self.f_px
        P_c = np.array([y_c, x_c, 1.0])  # Note: Thyra cam has X=down, Y=right? No. 
        # Re-verify project_f in cam: Pc[1]=right, -Pc[0]=up
        # So P_c = [- (v - h/2)/f, (u - w/2)/f, 1.0]
        vec_c = np.array([-(v - self.h/2.0)/self.f_px, (u - self.w/2.0)/self.f_px, 1.0])
        
        # 2. Camera to NED rotation
        pitch_total = self.gimbal_pitch if self.gimbal_pitch is not None else self.mount_pitch
        
        # Effective Angle: Match simulation convention exactly
        R_drone = self._rot(self.drone_att[0], self.drone_att[1], self.drone_att[2])
        R_mount = self._rot(0.0, pitch_total, 0.0)
        R = R_drone @ R_mount
        
        # 3. Target Ground Projection
        vec_n_target = R @ vec_c
        if vec_n_target[2] <= 0: return None
        k_target = alt / vec_n_target[2]
        ground_target = vec_n_target * k_target

        # 4. Center-of-FOV Ground Projection
        vec_c_center = np.array([0.0, 0.0, 1.0])
        vec_n_center = R @ vec_c_center
        if vec_n_center[2] <= 0: return None
        k_center = alt / vec_n_center[2]
        ground_center = vec_n_center * k_center

        # The error for control is (Target - Boresight) in ground meters
        rel_err = ground_target - ground_center
        return rel_err[:2] # [x_m, y_m] relative to drone in NED

    def _rot(self, roll, pitch, yaw):
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        return Rz @ Ry @ Rx

    def _image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        cx, cy = w / 2.0, h / 2.0

        corners, ids, _ = cv2.aruco.detectMarkers(frame, _ARUCO_DICT, parameters=_ARUCO_PARAMS)

        out = Vector3Stamped()
        out.header.stamp = self.get_clock().now().to_msg()

        if ids is not None and 0 in ids.flatten():
            idx = list(ids.flatten()).index(0)
            c = corners[idx][0]   # shape (4, 2)

            # Marker centre
            mc_x = float(np.mean(c[:, 0]))
            mc_y = float(np.mean(c[:, 1]))

            # Metric Error (Ground Projection)
            alt = -self.drone_pos[2]
            g_pos = self._to_ground(mc_x, mc_y, alt)
            
            if g_pos is not None:
                err_x_m = g_pos[0]  # North offset from drone in meters
                err_y_m = g_pos[1]  # East  offset from drone in meters
                
                out.vector.x = err_x_m
                out.vector.y = err_y_m
                out.vector.z = 1.0   # LOCKED
            else:
                out.vector.z = 0.0

            # Header encodes relative yaw (marker heading - drone yaw)
            dx = c[1][0] - c[0][0]
            dy = c[1][1] - c[0][1]
            marker_angle_img = math.degrees(math.atan2(dy, dx))
            out.header.frame_id = f'{marker_angle_img:.2f}'

            # Draw detection overlay
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.circle(frame, (int(mc_x), int(mc_y)), 6, (0, 255, 0), -1)
        else:
            out.vector.x = 0.0
            out.vector.y = 0.0
            out.vector.z = 0.0   # NO LOCK
            out.header.frame_id = '0.0'

        self.pub_error.publish(out)

        # Show annotated feed
        cv2.imshow('ArUco Detector (Comparison)', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorComparison()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
