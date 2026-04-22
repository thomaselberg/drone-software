#!/bin/bash

# Setup colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Ensure display is available for OpenCV windows
export DISPLAY=${DISPLAY:-:1}

echo -e "${BLUE}================================================${NC}"
echo -e "${GREEN}      MISSION 0: SIMULATION BASELINE            ${NC}"
echo -e "${BLUE}================================================${NC}"

# 1. Cleanup all existing processes
echo -e "${YELLOW}>>> Cleaning up previous sessions...${NC}"
pkill -9 -f MicroXRCEAgent
pkill -9 -f px4
pkill -9 -f "gz sim"
pkill -9 -f "ruby"
pkill -9 -f "thyra"
pkill -9 -f "mission"
pkill -9 -f "synthetic"
pkill -9 -f "python3"
sleep 2

# Clear old logs
rm -f /tmp/sim_output_mission0.log

# 2. Source environment
source /opt/ros/jazzy/setup.bash
cd ~/drone-software
source install/setup.bash

# 3. Launch QGroundControl
QGC_APPIMAGE="$HOME/QGroundControl-x86_64.AppImage"
if [ -f "$QGC_APPIMAGE" ]; then
    echo -e "${BLUE}>>> Launching QGroundControl...${NC}"
    chmod +x "$QGC_APPIMAGE"
    "$QGC_APPIMAGE" > /dev/null 2>&1 &
fi

# 4. Launch Simulation (Team Launch)
echo -e "${BLUE}>>> [1/3] Launching official team simulation...${NC}"
ros2 launch thyra thyra_sim.launch.py 2>&1 | tee /tmp/sim_output_mission0.log &
SIM_PID=$!

# 5. Launch Synthetic Camera
echo -e "${BLUE}>>> [2/3] Launching synthetic camera...${NC}"
ros2 run thyra synthetic_camera_sim.py --ros-args -r __ns:=/asr/thyra &
CAM_PID=$!

# 6. Wait for Autopilot Readiness
echo -e "${YELLOW}>>> [3/3] Waiting for THYRA OPERATIONAL...${NC}"
while true; do
    if grep -q "THYRA OPERATIONAL" /tmp/sim_output_mission0.log 2>/dev/null; then
        echo -e "${GREEN}>>> Autopilot Ready! Launching Mission Zero...${NC}"
        break
    fi
    sleep 1
done

# 7. Launch Mission Script
ros2 run thyra mission_zero.py --ros-args -r __ns:=/asr/thyra &
MISSION_PID=$!

# Handle shutdown
echo -e "${GREEN}Mission running. Press Ctrl+C to stop everything.${NC}"
trap "echo 'Shutting down...'; kill $SIM_PID $CAM_PID $MISSION_PID 2>/dev/null; exit" INT TERM
wait
