#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from interfaces.msg import ProbeGlobalLocations  # Adjust based on your message package

class ProbeRepublisher(Node):
    def __init__(self):
        super().__init__('probe_republisher')

        # Define QoS profile matching the input topic
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        qos2 = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )


        # Create subscriber for /probe_detector/global_probe_locations_cyclone
        self.subscription = self.create_subscription(
            ProbeGlobalLocations,
            '/probe_detector/global_probe_locations_cyclone',
            self.listener_callback,
            qos2
        )

        # Create publisher for /probe_detector/global_probe_locations
        self.publisher = self.create_publisher(
            ProbeGlobalLocations,
            '/probe_detector/global_probe_locations',
            qos
        )

        self.get_logger().info('Probe Republisher Node started')

    def listener_callback(self, msg):
        # Republish the received message to the new topic
        self.publisher.publish(msg)
        self.get_logger().info('Republishing message to /probe_detector/global_probe_locations')

def main(args=None):
    rclpy.init(args=args)
    node = ProbeRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Probe Republisher Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()