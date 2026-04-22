#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class SyntheticCameraNode(Node):
    def __init__(self):
        super().__init__('synthetic_camera_sim')
        
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        
        self.bridge = CvBridge()
        self.pub_image = self.create_publisher(Image, 'out/camera_feed', qos_profile_sensor_data)
        
        # ArUco dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        
        # Create a timer for the video feed (30 FPS)
        self.create_timer(1.0/30.0, self.timer_cb)
        
        self.get_logger().info("Synthetic Camera Sim Started")

    def timer_cb(self):
        # Create a blank grey image
        img = np.full((self.height, self.width, 3), 128, dtype=np.uint8)
        
        # Draw a simulated ArUco marker in the middle
        marker_size = 100
        marker_img = np.zeros((marker_size, marker_size), dtype=np.uint8)
        
        # Use older OpenCV API for compatibility if needed
        try:
            marker_img = cv2.aruco.generateImageMarker(self.aruco_dict, 0, marker_size)
        except AttributeError:
            marker_img = cv2.aruco.drawMarker(self.aruco_dict, 0, marker_size)
            
        # Place marker in center
        y_off = (self.height - marker_size) // 2
        x_off = (self.width - marker_size) // 2
        img[y_off:y_off+marker_size, x_off:x_off+marker_size] = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
        
        # Add some text
        cv2.putText(img, "SYNTHETIC CAMERA FEED", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(img, f"Marker ID: 0", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

        # Show the window
        cv2.imshow("Synthetic Camera Feed", img)
        cv2.waitKey(1)
        
        # Publish
        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.pub_image.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SyntheticCameraNode()
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
