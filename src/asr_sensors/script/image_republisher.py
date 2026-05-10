#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
import message_filters
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class CompressedImageRepublisher(Node):
    def __init__(self):
        super().__init__('compressed_image_republisher_best_effort')
        
        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()
        
        # Define Best Effort QoS profile
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # Subscribers for color and depth images
        self.color_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw', qos_profile=qos)
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/depth/image_rect_raw', qos_profile=qos)
        
        # Synchronize color and depth images
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)
        
        # Publishers for compressed topics
        self.color_pub = self.create_publisher(
            CompressedImage, '/thyra/out/color_image/compressed', qos)
        self.depth_pub = self.create_publisher(
            CompressedImage, '/thyra/out/depth_image/compressed', qos)
        
        # Timer for 5 Hz publishing
        self.timer = self.create_timer(1.0 / 5.0, self.timer_callback)
        self.latest_color_msg = None
        self.latest_depth_msg = None
        self.last_published_timestamp = None
        
        self.get_logger().info('Republishing synchronized compressed color (JPEG) and depth (PNG) images with Best Effort QoS at 5 Hz')

    def image_callback(self, color_msg, depth_msg):
        # Log received encodings
        #self.get_logger().info(f'Received color image encoding: {color_msg.encoding}, depth image encoding: {depth_msg.encoding}')
        
        # Store the latest synchronized messages only if they are new
        if self.last_published_timestamp is None or \
           (color_msg.header.stamp.sec > self.last_published_timestamp.sec or \
            (color_msg.header.stamp.sec == self.last_published_timestamp.sec and \
             color_msg.header.stamp.nanosec > self.last_published_timestamp.nanosec)):
            self.latest_color_msg = color_msg
            self.latest_depth_msg = depth_msg

    def timer_callback(self):
        # Publish only if new synchronized messages are available
        if self.latest_color_msg and self.latest_depth_msg:
            # Check if this pair hasn't been published yet
            if self.last_published_timestamp is None or \
               (self.latest_color_msg.header.stamp.sec > self.last_published_timestamp.sec or \
                (self.latest_color_msg.header.stamp.sec == self.last_published_timestamp.sec and \
                 self.latest_color_msg.header.stamp.nanosec > self.last_published_timestamp.nanosec)):
                try:
                    # Convert and compress color image to JPEG
                    color_cv = self.bridge.imgmsg_to_cv2(self.latest_color_msg, desired_encoding='rgb8')
                    # Convert from RGB to BGR for OpenCV JPEG encoding
                    color_cv_bgr = cv2.cvtColor(color_cv, cv2.COLOR_RGB2BGR)
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 20]
                    success, color_buffer = cv2.imencode('.jpg', color_cv_bgr, encode_param)
                    if not success:
                        self.get_logger().error('Failed to compress color image to JPEG')
                        return
                    color_compressed = CompressedImage()
                    color_compressed.header = self.latest_color_msg.header
                    color_compressed.format = 'jpeg'
                    color_compressed.data = color_buffer.tobytes()                    
                    # Convert and compress depth image to PNG
                    depth_cv = self.bridge.imgmsg_to_cv2(self.latest_depth_msg, desired_encoding='16UC1')
                    success, depth_buffer = cv2.imencode('.png', depth_cv)
                    if not success:
                        self.get_logger().error('Failed to compress depth image to PNG')
                        return
                    depth_compressed = CompressedImage()
                    depth_compressed.header = self.latest_depth_msg.header
                    depth_compressed.format = 'png'
                    depth_compressed.data = depth_buffer.tobytes()
                    
                    # Publish compressed images
                    self.color_pub.publish(color_compressed)
                    self.depth_pub.publish(depth_compressed)
                    self.last_published_timestamp = self.latest_color_msg.header.stamp
                    #self.get_logger().info(f'Published new compressed image pair at timestamp {self.last_published_timestamp.sec}.{self.last_published_timestamp.nanosec}')
                    
                    # Clear messages to prevent republishing
                    self.latest_color_msg = None
                    self.latest_depth_msg = None
                    
                except CvBridgeError as e:
                    self.get_logger().error(f'CvBridge error: {str(e)}')
                except Exception as e:
                    self.get_logger().error(f'Unexpected error compressing images: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()