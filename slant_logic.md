# Slanted Approach Logic (Inverted Gimbal Control)

## Concept Overview
The traditional approach to visual tracking involves moving the drone to a static position and then using a gimbal to track a subject natively. This approach **inverts that logic**. 

Instead of the gimbal reacting to track the target, the **gimbal dictates the geometric approach profile**, and the drone reacts to keep the target centered. The gimbal essentially acts as a reference signal that "pushes" the drone forward down a glide slope.

## The Kinematic Mechanism

1. **Velocity Controller (The Reaction):**
   The drone runs a dedicated PID velocity controller. The sole objective of this controller is to minimize the 2D pixel distance between the center of the camera's Field of View (FOV) and the center of the ArUco marker. It achieves this by commanding forward/backward/left/right velocities.

2. **Gimbal Pitch (The Push):**
   By actively pitching the camera servo downwards, the physical angle of the camera changes. This causes the ArUco marker to abruptly appear higher in the camera frame (moving away from the center). 

3. **The Geometric "Glide Slope":**
   To compensate for the marker drifting off-center, the velocity controller is mathematically forced to command forward velocity to "catch up" to the target and recenter it. 
   
   If the gimbal pitch is swept continuously from 45° down to 90° (looking straight down), the drone is continuously forced to approach the target horizontally while dropping altitude, perfectly carving out an approach slope.

## Phases of the Landing Mission

### 1. High Altitude Search & Alignment
* **Pitch:** Fixed at forward/down angle (e.g., 45°).
* **Benefit:** Massively increases the forward ground footprint (Field of View) for faster target acquisition.
* **Alignment:** Once the target is locked, the drone's yaw is aligned to match the target's forward orientation.

### 2. Active Approach (The Slant)
* **Pitch:** Dynamically actuating continuously from 45° towards 90°.
* **Drone Reaction:** The velocity controller pushes the drone forward and coordinates descending altitude to maintain the target dead-center in the FOV against the changing camera pitch.

### 3. Touchdown
* **Pitch:** Reaches exactly 90° (pointing straight down).
* **Condition:** When the gimbal is at 90° and the XY pixel error remains zero, the drone is geometrically guaranteed to be hovering directly and squarely over the landing pad.
* **Action:** The system issues the final vertical `land` command.

## Advantages
* **Simplicity:** Eliminates complex 3D trigonometric math or rigid mathematical glide paths. 
* **Dynamic:** The steepness of the approach is entirely controlled by the *rate* at which the servo node sweeps the pitch angle. 
* **Robustness:** Relies entirely on simple 2D pixel-error minimization from the camera, offloading the physical approach path to the hardware actuation of the Cube.
