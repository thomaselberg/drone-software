import numpy as np

def deg2rad(d):
    return d * np.pi / 180.0

# Known translation in drone frame
t_drone = np.array([0.137751, -0.018467, 0.12126])  # meters

# Step 1: camera optical frame to drone FRD (no tilt yet)
R_perm = np.array([
    [0, 0, 1],  # X_drone <- Z_cam
    [1, 0, 0],  # Y_drone <- X_cam
    [0, 1, 0]   # Z_drone <- Y_cam
])

# Step 2: apply mount tilt about drone Y axis (-45° pitch down)
pitch_mount = deg2rad(-45)
R_mount = np.array([
    [np.cos(pitch_mount), 0, np.sin(pitch_mount)],
    [0, 1, 0],
    [-np.sin(pitch_mount), 0, np.cos(pitch_mount)]
])

# Final rotation: camera -> drone
R_drone_from_cam = R_mount @ R_perm

# Build 4x4 homogeneous transform
T_drone_from_cam = np.eye(4)
T_drone_from_cam[:3, :3] = R_drone_from_cam
T_drone_from_cam[:3, 3] = t_drone

print("=== Camera -> Drone transform ===")
print(T_drone_from_cam)

# Test: point 21.5cm ahead in camera frame
p_cam = np.array([0, 0, 0.215, 1])  # homogeneous
p_drone = T_drone_from_cam @ p_cam
print("\nPoint in drone coords:", p_drone[:3])
