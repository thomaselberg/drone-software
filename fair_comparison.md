# Fair Comparison Simulation: Gimbal vs. Static Camera

## Overview
This document outlines the simulation framework for the P6 thesis comparison between a **Dynamic Gimbaling Camera** and a **Static 45° Camera**. The goal is to provide a "Fair Comparison" by giving the existing literature (the static camera approach) the **benefit of the doubt** through a perfect state estimator (Fake EKF).


## 2. System Configuration
- **Control Mode:** `manual_aided` (Topic-based velocity streaming).
- **Vision:** Synthetic ArUco detection (NED Frame, Flat Earth).
- **Visualization:** High-contrast grid lines drawn in the synthetic camera window (OpenCV) for motion reference.

## 3. Mission Workflow (State Machine)
1. **TAKEOFF:** Immediate ascent to 5m altitude
2. **SEARCH:** `GOTO` target GPS start location.
3. **ACQUISITION:** Transition to `Manual Aided` upon ArUco lock.
4. **STABILIZE:** 10s hover at 5m, keeping target centered in FOV.
5. **DESCEND:** Controlled descent to 1m altitude while maintaining center-lock.
6. **PRE-LAND:** 10s tracking at 1m altitude.
7. **TERMINAL_LAND:**
   - **Mode A (Static 45°):** 
     - Trigger point: Start of final descent from 1m altitude.
     - Logic: Maintain 1m altitude, move horizontally to the extrapolated target position using last known `(x, y, vx, vy)`.
     - Touchdown: Once horizontal position is reached, set Z velocity to 0.5 m/s until touchdown.
     - Vision: Lock loss is expected and ignored during this phase.
   - **Mode B (Gimbal Slant):** 
     - Logic: Active sweep from 45° to 90° as per `slant_logic.md`.
     - Timing: Full sweep must take exactly **5 seconds**.
     - Maintain visual lock until physical touchdown.

It shall be possible to select either stationary 45 degree angle or gimbal mode.
The static camera angle will get fed with true target states when visually locked to the target.
The gimbal will not need this as it will use the slant_logic.md -thus never needing to know where the target is - it will just get closer automatically due to the FOV sweep and continuous target centering.


## 4. Scenario & Disturbances

- **Geometry:** Flat earth assumption.
- **Target Altitude:** Fixed at 0m (ground level).
- **Target Motion Model:**
    - **Velocity Vector:** Smooth acceleration curves.
    - **Forward Velocity ($V_f$):** Range $[0, 3]$ m/s.
    - **Lateral Velocity ($V_l$):** Range $[-0.3, 0.3]$ m/s.
    - **Constraints:** 
        - $|V_l| \le 0.1 \cdot V_f$ (Lateral movement never exceeds 10% of forward speed).
        - **No Reversing:** $V_f \ge 0$.
    - **Heading:** The ArUco marker's forward orientation is always perfectly aligned with its current velocity vector.
    - **Start position** of the target must be easily configurable
    - **wind** Stochastic Gust Generator: A standalone Python script that injects randomized 5-10m/s wind vectors into the Gazebo world via real-time service requests. Should be perfectly reproducible for fair comparison. I propose 3 different gust scenarios that always make the same profile when running. The gust scenario should be selected and run within start_sim.sh
    - **Simulated significant change in illumination** the camera will probably get blasted with light from reflections at some point - so at some intervals (every 10 sec) the cam should briefly (max 0.2s) be completely white. This should also be reproducible. I propose this be added to the synthetic camera script and easily configurable. 

## 5. Key Performance Indicators (KPIs)
The following metrics must be recorded at the point of **Touchdown** into a new CSV file for every run.
- **Filename Format:** `[mode]_[wind_scenario]_[HHMM].csv` (e.g., `static_gust2_1445.csv`)

1. **Linear Distance Error:** Euclidean distance (m) between the drone's center and the target center.
2. **Rotation Error:** The angular difference (degrees/radians) between the drone's forward direction and the target's forward direction.
3. **Engagement Duration:** Total time elapsed from **First Visual Lock** to **Touchdown**.

## 6. Safety & Failsafes
- **Lock Loss (<2s):** Maintain last commanded velocity.
- **Lock Loss (>2s):** Engage `RTL` (Abort landing and return to takeoff position).
- **Exception:** Lock loss is ignored during the `Mode A` Terminal Landing phase.


