#!/usr/bin/env python3
"""
synthetic_cam_and_target_skeleton.py
------------------------------------
Responsibility: 
1. Simulation of Target Physics (Hardcoded start/velocity).
2. Rendering of ArUco marker using Perspective Warp.
3. Injection of Illumination Disturbances (Whiteout).

Publishers:
- /asr/sim/synthetic_camera/image (sensor_msgs/Image)
- /asr/sim/true_target_state (TrueTargetState)

Subscribers:
- /asr/thyra/out/drone_state (DroneState) - To know where the camera is.
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time

class SyntheticSimEngine(Node):
    def __init__(self):
        super().__init__('synthetic_sim_engine')

        # --- Hardcoded Target Config ---
        self.target_x = 20.0
        self.target_y = 0.0
        self.target_vx = 1.0  # m/s
        self.target_vy = 0.0  # m/s

        # --- Illumination Config ---
        self.whiteout_interval = 10.0 # seconds
        self.whiteout_duration = 0.2  # seconds
        self.last_whiteout_time = time.time()

        # --- Publishers ---
        self.pub_image = self.create_publisher(None, '/asr/sim/synthetic_camera/image', 10)
        self.pub_truth = self.create_publisher(None, '/asr/sim/true_target_state', 10)

        # --- Subscribers ---
        self.sub_drone = self.create_subscription(None, '/asr/thyra/out/drone_state', self.drone_callback, 10)

        self.drone_state = None
        self.create_timer(0.033, self.sim_loop) # 30Hz

    def drone_callback(self, msg):
        self.drone_state = msg

    def sim_loop(self):
        # 1. Update Target Physics (Constant Velocity)
        # self.target_x += self.target_vx * dt ...
        
        # 2. Check for Illumination Whiteout
        # if (now - self.last_whiteout_time) > 10s: render white frame
        
        # 3. Render Synthetic Frame
        # - Project 3D Grid on ground
        # - Project ArUco Marker using Perspective Warp
        
        # 4. Publish Image and True State
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SyntheticSimEngine()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
