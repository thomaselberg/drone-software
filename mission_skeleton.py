#!/usr/bin/env python3
"""
mission_25_skeleton.py
----------------------
Final Verbose Skeleton for the Mission Controller.
Provides the complete State Machine structure for Mode A/B benchmarking.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time
import math

class MissionState:
    IDLE          = "IDLE"
    TAKEOFF       = "TAKEOFF"       # Immediate ascent to 5m
    SEARCH        = "SEARCH"        # GOTO GPS start location
    STABILIZE_5M  = "STABILIZE_5M"  # 10s tracking at 5m
    DESCEND_TO_1M = "DESCEND_TO_1M" # Moving down to 1m while tracking
    STABILIZE_1M  = "STABILIZE_1M"  # 10s tracking at 1m (Pre-Land)
    TERMINAL_LAND = "TERMINAL_LAND" # Mode A (Blind) or Mode B (Gimbal Slant)
    RTL           = "RTL"           # Failsafe abort

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # --- Configuration ---
        self.declare_parameter('mode', 'GIMBAL') # 'STATIC' or 'GIMBAL'
        self.mission_mode = self.get_parameter('mode').value
        
        # --- Publishers ---
        # Topic C: Control Input
        self.pub_control = self.create_publisher(None, '/asr/thyra/in/manual_control_input', 10)

        # --- Subscribers ---
        # Topic A: Drone State
        self.sub_state = self.create_subscription(None, '/asr/thyra/out/drone_state', self.drone_callback, 10)
        # Topic B: Pixel Error
        self.sub_pixel = self.create_subscription(None, '/asr/thyra/out/aruco_pixel_error', self.pixel_callback, 10)
        # Ground Truth (One-Shot for Static Mode)
        self.sub_truth = self.create_subscription(None, '/asr/sim/true_target_state', self.truth_callback, 10)

        # --- Variables ---
        self.state = MissionState.IDLE
        self.last_lock_time = self.get_clock().now()
        self.state_timer = time.time()
        self.one_shot_data = None
        self.gimbal_angle = 45.0
        
        # --- Main Loop ---
        self.create_timer(0.05, self.mission_loop) # 20Hz

    def drone_callback(self, msg):
        # Store NED position and Yaw
        pass

    def pixel_callback(self, msg):
        # Store x,y error and update last_lock_time if msg.z == 1.0
        pass

    def truth_callback(self, msg):
        # One-Shot Capture Logic for Mode A
        if self.state == MissionState.TERMINAL_LAND and self.mission_mode == 'STATIC':
            if self.one_shot_data is None:
                self.one_shot_data = msg
                self.get_logger().info("STATIC MODE: Captured One-Shot Ground Truth.")

    def mission_loop(self):
        """ Primary State Machine Transitions """
        
        # 1. Failsafe: RTL on 2s lock loss (Except during Static Blind Land)
        if self.state not in [MissionState.IDLE, MissionState.TERMINAL_LAND]:
            if (self.get_clock().now() - self.last_lock_time).nanoseconds > 2e9:
                self.state = MissionState.RTL

        # 2. Transition Logic
        if self.state == MissionState.IDLE:
            # Command Takeoff -> Proceed to TAKEOFF state
            pass

        elif self.state == MissionState.TAKEOFF:
            # Check altitude -> if >= 5m, proceed to SEARCH
            pass

        elif self.state == MissionState.SEARCH:
            # Command GOTO -> if pixel_error.z == 1, proceed to STABILIZE_5M
            pass

        elif self.state == MissionState.STABILIZE_5M:
            # Timer 10s + Visual Centering -> then proceed to DESCEND_TO_1M
            pass

        elif self.state == MissionState.DESCEND_TO_1M:
            # Command Z-velocity while centering -> if altitude <= 1m, proceed to STABILIZE_1M
            pass

        elif self.state == MissionState.STABILIZE_1M:
            # Timer 10s + Visual Centering -> proceed to TERMINAL_LAND
            pass

        elif self.state == MissionState.TERMINAL_LAND:
            if self.mission_mode == 'STATIC':
                self.execute_blind_landing()
            else:
                self.execute_slant_landing()

        elif self.state == MissionState.RTL:
            # Command Return to Launch
            pass

    def execute_blind_landing(self):
        """ Mode A: 1m Hover -> Extrapolate -> 0.5m/s Plunge """
        if self.one_shot_data:
            # 1. Calculate target future position
            # 2. Command horizontal velocity to intercept
            # 3. If within 0.1m, set Vz = 0.5 m/s downwards
            pass

    def execute_slant_landing(self):
        """ Mode B: 5s Gimbal Sweep (45 to 90 deg) with active tracking """
        # 1. Update gimbal_angle from 45 to 90 over 5 seconds
        # 2. Minimize pixel error with horizontal velocity
        # 3. Land on touchdown
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
