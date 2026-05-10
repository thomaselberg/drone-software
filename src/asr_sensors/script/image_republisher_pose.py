#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
import message_filters
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from geometry_msgs.msg import PoseStamped

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
        
        # Subscribers for color, depth, position, and attitude
        self.color_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw', qos_profile=qos)
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/depth/image_rect_raw', qos_profile=qos)
        self.position_sub = message_filters.Subscriber(
            self, VehicleLocalPosition, '/fmu/out/vehicle_local_position', qos_profile=qos)
        self.attitude_sub = message_filters.Subscriber(
            self, VehicleAttitude, '/fmu/out/vehicle_attitude', qos_profile=qos)
     

        # Synchronize all four messages
        self.ts = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub, self.position_sub, self.attitude_sub],
            queue_size=10,
            slop=0.1,
            allow_headerless=True  # Enable headerless message support
        )
        self.ts.registerCallback(self.synced_callback)
        
        # Publishers for compressed topics and pose
        self.color_pub = self.create_publisher(
            CompressedImage, '/thyra/out/color_image/compressed', qos)
        self.depth_pub = self.create_publisher(
            CompressedImage, '/thyra/out/depth_image/compressed', qos)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/thyra/out/pose/synced_with_RGBD', qos)
        self.origin_offset_sub = self.create_subscription(
            PoseStamped, '/thyra/out/origin_offset', self.origin_offset_callback, 10)

        # Timer for 5 Hz publishing
        self.timer = self.create_timer(1.0 / 5.0, self.timer_callback)
        self.latest_color_msg = None
        self.latest_depth_msg = None
        self.latest_position_msg = None
        self.latest_attitude_msg = None
        self.last_published_timestamp = None
        self.current_origin_offset = (0, 0, 0)



        self.get_logger().info('Republishing synchronized compressed color (JPEG) and depth (PNG) images, and pose with Best Effort QoS at 5 Hz')

    def origin_offset_callback(self, msg):
        # Handle the origin offset message
        self.current_origin_offset = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)


    def synced_callback(self, color_msg, depth_msg, position_msg, attitude_msg):
        # Store the latest synchronized messages only if they are new (using color timestamp as reference)
        if self.last_published_timestamp is None or \
           (color_msg.header.stamp.sec > self.last_published_timestamp.sec or \
            (color_msg.header.stamp.sec == self.last_published_timestamp.sec and \
             color_msg.header.stamp.nanosec > self.last_published_timestamp.nanosec)):
            self.latest_color_msg = color_msg
            self.latest_depth_msg = depth_msg
            self.latest_position_msg = position_msg
            self.latest_attitude_msg = attitude_msg

    def timer_callback(self):
        # Publish only if new synchronized messages are available
        if self.latest_color_msg and self.latest_depth_msg and self.latest_position_msg and self.latest_attitude_msg:
            # Check if this set hasn't been published yet
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
                    
                    # Create PoseStamped message
                    pose_msg = PoseStamped()
                    pose_msg.header = self.latest_color_msg.header  # Use color timestamp for sync
                    pose_msg.pose.position.x = float(self.latest_position_msg.x - self.current_origin_offset[0])
                    pose_msg.pose.position.y = float(self.latest_position_msg.y - self.current_origin_offset[1])
                    pose_msg.pose.position.z = float(self.latest_position_msg.z - self.current_origin_offset[2])
                    pose_msg.pose.orientation.x = float(self.latest_attitude_msg.q[1])
                    pose_msg.pose.orientation.y = float(self.latest_attitude_msg.q[2])
                    pose_msg.pose.orientation.z = float(self.latest_attitude_msg.q[3])
                    pose_msg.pose.orientation.w = float(self.latest_attitude_msg.q[0])
                    
                    # Publish all
                    self.color_pub.publish(color_compressed)
                    self.depth_pub.publish(depth_compressed)
                    self.pose_pub.publish(pose_msg)
                    self.last_published_timestamp = self.latest_color_msg.header.stamp
                    
                    # Clear messages to prevent republishing
                    self.latest_color_msg = None
                    self.latest_depth_msg = None
                    self.latest_position_msg = None
                    self.latest_attitude_msg = None
                    
                except CvBridgeError as e:
                    self.get_logger().error(f'CvBridge error: {str(e)}')
                except Exception as e:
                    self.get_logger().error(f'Unexpected error compressing images or creating pose: {str(e)}')

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