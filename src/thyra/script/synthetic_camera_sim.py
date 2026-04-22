#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from interfaces.msg import DroneState
from px4_msgs.msg import VehicleLocalPosition
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class SyntheticCameraNode(Node):
    def __init__(self):
        super().__init__('synthetic_camera_sim')
        
        # Parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('hfov', 85.0)
        
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        hfov_rad = math.radians(self.get_parameter('hfov').value)
        self.f_px = (self.width / 2.0) / math.tan(hfov_rad / 2.0)
        
        # Fixed Camera Mounting (45 degrees forward-down)
        self.mount_pitch = math.radians(45.0) 
        
        # Drone State
        self.pos = [0.0, 0.0, 0.0]
        self.att = [0.0, 0.0, 0.0]
        
        # Grid settings - Much larger and static
        self.grid_spacing = 2.0 
        self.grid_extent = 100.0 # Drawing lines from -100 to 100 meters
        
        self.bridge = CvBridge()
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.lpos_cb, qos_profile_sensor_data)
        self.create_subscription(DroneState, 'out/drone_state', self.state_cb, 10)
        self.pub_image = self.create_publisher(Image, 'out/camera_feed', qos_profile_sensor_data)
        
        self.create_timer(1.0/30.0, self.timer_cb)
        self.get_logger().info("Super-Stable Slant Grid Camera Started")

    def lpos_cb(self, msg):
        self.pos = [msg.x, msg.y, msg.z]

    def state_cb(self, msg):
        if len(msg.orientation) >= 3:
            self.att = [msg.orientation[0], msg.orientation[1], msg.orientation[2]]

    def get_rotation_matrix(self, roll, pitch, yaw):
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        return R_z @ R_y @ R_x

    def project_point(self, x_world, y_world, z_world, R_total, drone_pos):
        P_rel = np.array([x_world, y_world, z_world]) - drone_pos
        P_cam = R_total.T @ P_rel
        
        # CV Mapping: Z=Forward (P_cam[2]), X=Right (P_cam[1]), Y=Down (-P_cam[0])
        z_val = P_cam[2]
        if z_val < 0.2: 
            return None
            
        u = (P_cam[1] / z_val) * self.f_px + self.width / 2
        v = (-P_cam[0] / z_val) * self.f_px + self.height / 2
        
        return (int(u), int(v))

    def timer_cb(self):
        # High-contrast background
        img = np.full((self.height, self.width, 3), 45, dtype=np.uint8)
        
        drone_pos = np.array(self.pos)
        R_drone = self.get_rotation_matrix(self.att[0], self.att[1], self.att[2])
        R_mount = self.get_rotation_matrix(0, self.mount_pitch, 0)
        R_total = R_drone @ R_mount
        
        grid_color = (0, 255, 255) # Cyan
        
        # Define static grid points
        ticks = np.arange(-self.grid_extent, self.grid_extent + self.grid_spacing, self.grid_spacing)
        
        # Draw North-South Lines
        for x in ticks:
            pts = []
            # Sample density increases as we get closer to drone for smoothness
            for y in np.linspace(self.pos[1] - 30, self.pos[1] + 30, 40):
                p = self.project_point(x, y, 0.0, R_total, drone_pos)
                if p:
                    # Clip to slightly outside image to prevent wrap-around
                    if -100 < p[0] < self.width + 100 and -100 < p[1] < self.height + 100:
                        pts.append(p)
            if len(pts) > 1:
                cv2.polylines(img, [np.array(pts)], False, grid_color, 1, cv2.LINE_AA)

        # Draw East-West Lines
        for y in ticks:
            pts = []
            for x in np.linspace(self.pos[0] - 30, self.pos[0] + 30, 40):
                p = self.project_point(x, y, 0.0, R_total, drone_pos)
                if p:
                    if -100 < p[0] < self.width + 100 and -100 < p[1] < self.height + 100:
                        pts.append(p)
            if len(pts) > 1:
                cv2.polylines(img, [np.array(pts)], False, grid_color, 1, cv2.LINE_AA)

        # Static Land Target at (5, 0)
        p_target = self.project_point(5.0, 0.0, 0.0, R_total, drone_pos)
        if p_target:
            if 0 <= p_target[0] < self.width and 0 <= p_target[1] < self.height:
                cv2.circle(img, p_target, 20, (0, 0, 255), 3, cv2.LINE_AA) # Red circle
                cv2.putText(img, "H", (p_target[0]-10, p_target[1]+10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # HUD
        cv2.rectangle(img, (0, 0), (180, 80), (20, 20, 20), -1)
        cv2.putText(img, f"ALT: {-self.pos[2]:.2f}m", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(img, f"X: {self.pos[0]:.2f} Y: {self.pos[1]:.2f}", (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        cv2.imshow("Synthetic Camera Feed", img)
        cv2.waitKey(1)
        
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
