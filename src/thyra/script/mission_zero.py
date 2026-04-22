#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from interfaces.action import DroneCommand
from interfaces.msg import ManualControlInput, GcsHeartbeat, DroneState
from px4_msgs.msg import VehicleLocalPosition

import time
import math

class MissionZero(Node):
    def __init__(self):
        super().__init__('mission_zero')

        # QoS Profiles
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # State Variables
        self.drone_state = DroneState()
        self.local_pos = VehicleLocalPosition()
        self.state = "IDLE"
        self.state_start_time = self.get_clock().now()
        
        # Subscriptions
        self.create_subscription(DroneState, 'out/drone_state', self.state_cb, 10)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.lpos_cb, qos_sensor)

        # Publishers
        self.heartbeat_pub = self.create_publisher(GcsHeartbeat, 'in/gcs_heartbeat', 10)
        self.manual_pub = self.create_publisher(ManualControlInput, 'in/manual_input', 10)

        # Action Client
        self.cmd_client = ActionClient(self, DroneCommand, 'in/drone_command')

        # Timers
        self.create_timer(0.1, self.heartbeat_timer)
        self.create_timer(0.05, self.mission_timer)

        self.get_logger().info("Mission Zero Node Initialized")

    def state_cb(self, msg):
        self.drone_state = msg

    def lpos_cb(self, msg):
        self.local_pos = msg

    def heartbeat_timer(self):
        msg = GcsHeartbeat()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.heartbeat_pub.publish(msg)

    def send_cmd(self, cmd_type, target_pose=None):
        goal_msg = DroneCommand.Goal()
        goal_msg.command_type = cmd_type
        if target_pose:
            goal_msg.target_pose = target_pose
        
        self.get_logger().info(f"Sending Command: {cmd_type}")
        self.cmd_client.wait_for_server()
        return self.cmd_client.send_goal_async(goal_msg)

    def mission_timer(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds / 1e9

        if self.state == "IDLE":
            if elapsed > 5.0:  # Wait for sim to settle
                self.send_cmd("arm")
                self.state = "ARMING"
                self.state_start_time = now

        elif self.state == "ARMING":
            if self.drone_state.arming_state == 2:  # ARMED
                self.get_logger().info("Armed! Sending Takeoff...")
                self.send_cmd("takeoff", target_pose=[0.0, 0.0, -2.5]) # 2.5m altitude
                self.state = "TAKEOFF"
                self.state_start_time = now

        elif self.state == "TAKEOFF":
            alt = -self.local_pos.z
            if alt > 2.0:
                self.get_logger().info(f"Reached Alt: {alt:.2f}m. Switching to Manual Aided...")
                self.send_cmd("manual_aided")
                self.state = "MOVE_NORTH"
                self.state_start_time = now

        elif self.state == "MOVE_NORTH":
            if elapsed < 10.0:
                # Send velocity command via manual_input
                # pitch = 0.15 corresponds to ~0.5m/s North as per mapping
                msg = ManualControlInput()
                msg.roll = 0.0
                msg.pitch = 0.15 
                msg.yaw_velocity = 0.0
                msg.thrust = 0.0 # Hover in vertical axis
                self.manual_pub.publish(msg)
            else:
                self.get_logger().info("Move North Complete. Landing...")
                self.send_cmd("land")
                self.state = "LANDING"
                self.state_start_time = now

        elif self.state == "LANDING":
            if self.drone_state.arming_state == 1:  # DISARMED
                self.get_logger().info("Landed and Disarmed. Mission Complete.")
                self.state = "DONE"

def main(args=None):
    rclpy.init(args=args)
    node = MissionZero()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
