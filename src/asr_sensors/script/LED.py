#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from interfaces.msg import DroneState
import serial
from std_msgs.msg import Int16
led_mode = 0
led_buffer = 0
class DroneLEDNode(Node):
    def __init__(self):
        #qos = QoSProfile(
        #    depth=10,
        #    reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        #)
        super().__init__('drone_led_node')
        self.ser = serial.Serial( '/dev/ttyACM0', 9600, timeout=1)
        self.led_mode = 0
        self.subscription = self.create_subscription(
            DroneState,
            "thyra/out/drone_state",
            self.state_callback,
            10
        )

    def state_callback(self, msg):
        global led_mode, led_buffer
        led_mode = msg.led_mode
        self.led_mode = led_mode
        #self.led_mode = msg.led_mode
        #self.get_logger().info(f"LED mode updated: {self.led_mode}")
        try:
            if led_mode != led_buffer:
                self.ser.write(f"{self.led_mode}\n".encode())
                led_buffer = led_mode
            # Send mode as ASCII string with newline (easy for Arduino to parse)
            
            
            # OR if you want to send just one raw byte (no text):
            # self.ser.write(bytes([self.led_mode & 0xFF]))
        except Exception as e:
            self.get_logger().error(f"Failed to write to serial: {e}")


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # Open serial port (adjust to your device, e.g. '/dev/ttyUSB0' or 'COM3' on Windows)
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)


def main(args=None):
    rclpy.init(args=args)
    node = DroneLEDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print("Drone LED Node stopped")
    print(f"Final LED mode: {node.led_mode}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
