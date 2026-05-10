#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
from cv_bridge import CvBridge
import os

class StaticRGBDTestNode(Node):
    def __init__(self):
        super().__init__('static_rgbd_test_node')

        # QoS setup
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.bridge = CvBridge()

        # Publishers
        self.color_pub = self.create_publisher(
            CompressedImage, '/thyra/out/color_image/compressed', qos)
        self.depth_pub = self.create_publisher(
            CompressedImage, '/thyra/out/depth_image/compressed', qos)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/thyra/out/pose/synced_with_RGBD', qos)

        # Load static RGB image once
        package_share = os.path.join(
            os.getenv('AMENT_PREFIX_PATH').split(':')[0],
            'share', 'sensors'
        )
        image_path = os.path.join(package_share, 'ArUco.jpg')

        if not os.path.exists(image_path):
            self.get_logger().error(f"Static image not found at: {image_path}")
            rclpy.shutdown()
            return

        rgb_bgr = cv2.imread(image_path)
        if rgb_bgr is None:
            self.get_logger().error("Failed to load RGB image.")
            rclpy.shutdown()
            return

        self.color_msg = self.cv2_to_compressed(rgb_bgr, 'jpeg')

        # Create a static null depth image (all zeros, 640x480 16-bit)
        depth_cv = np.zeros((480, 640), dtype=np.uint16)
        self.depth_msg = self.cv2_to_compressed(depth_cv, 'png')

        # Dummy pose
        self.pose_msg = PoseStamped()
        self.pose_msg.pose.position.x = 0.0
        self.pose_msg.pose.position.y = 0.0
        self.pose_msg.pose.position.z = 0.0
        self.pose_msg.pose.orientation.w = 1.0

        # Timer at 5 Hz
        self.timer = self.create_timer(0.2, self.publish_static_data)

        self.get_logger().info("Static RGBD Test Node started, publishing at 5 Hz.")

    def cv2_to_compressed(self, cv_img, fmt):
        """
        Converts a CV2 image to a ROS2 CompressedImage message.
        """
        msg = CompressedImage()
        if fmt == 'jpeg':
            success, buffer = cv2.imencode('.jpg', cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        elif fmt == 'png':
            success, buffer = cv2.imencode('.png', cv_img)
        else:
            raise ValueError(f"Unsupported format: {fmt}")

        if not success:
            raise RuntimeError(f"Failed to encode image as {fmt}")

        msg.format = fmt
        msg.data = buffer.tobytes()
        return msg

    def publish_static_data(self):
        now = self.get_clock().now().to_msg()
        self.color_msg.header.stamp = now
        self.depth_msg.header.stamp = now
        self.pose_msg.header.stamp = now

        self.color_pub.publish(self.color_msg)
        self.depth_pub.publish(self.depth_msg)
        self.pose_pub.publish(self.pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StaticRGBDTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
