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

        # Matched to asr_autopilot (main.cpp:57-59)
        qos_heartbeat = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Matched to asr_autopilot (main.cpp:362 uses qos which is TransientLocal)
        qos_manual = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Matched to standard sensor data
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
        self.heartbeat_pub = self.create_publisher(GcsHeartbeat, 'in/gcs_heartbeat', qos_heartbeat)
        self.manual_pub = self.create_publisher(ManualControlInput, 'in/manual_input', qos_manual)

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
        msg.timestamp = float(self.get_clock().now().nanoseconds / 1e9)
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
            if self.drone_state.arming_state == 1:  # ARMED (from state_manager.h)
                self.get_logger().info("Armed! Sending Takeoff...")
                self.send_cmd("takeoff", target_pose=[-2.5]) # 2.5m altitude (negative in NED)
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
                msg.pitch = 0.8 # Boosted for ~1.5 m/s forward speed
                msg.yaw_velocity = 0.0
                msg.thrust = 0.0 # Hover in vertical axis
                self.manual_pub.publish(msg)
            else:
                self.get_logger().info("Move North Complete. Landing...")
                self.send_cmd("land")
                self.state = "LANDING"
                self.state_start_time = now

        elif self.state == "LANDING":
            if self.drone_state.arming_state == 0:  # DISARMED (from state_manager.h)
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
