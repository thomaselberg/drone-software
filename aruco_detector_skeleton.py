#!/usr/bin/env python3
"""
aruco_detector_skeleton.py
--------------------------
Responsibility:
1. Subscribe to raw image feed.
2. Detect ArUco marker (ID 0).
3. Calculate normalized pixel error and lock status.

Publishers:
- /asr/thyra/out/aruco_pixel_error (ArucoPixelError)

Subscribers:
- /asr/sim/synthetic_camera/image (sensor_msgs/Image)
"""

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        self.bridge = CvBridge()

        # --- Publishers ---
        self.pub_error = self.create_publisher(None, '/asr/thyra/out/aruco_pixel_error', 10)

        # --- Subscribers ---
        self.sub_image = self.create_subscription(None, '/asr/sim/synthetic_camera/image', self.image_callback, 10)

    def image_callback(self, msg):
        # 1. Convert ROS Image to OpenCV
        # 2. Run cv2.aruco.detectMarkers
        # 3. Calculate normalized error:
        #    x_error = (marker_center_x - image_center_x) / (image_width / 2)
        # 4. Calculate Relative Yaw if possible
        # 5. Publish ArucoPixelError
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
