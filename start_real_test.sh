#!/bin/bash
# start_real_test.sh — Real flight test launcher
#
# Usage:
#   ./start_real_test.sh HANDHELD
#   ./start_real_test.sh HOVER_TRACK 3.0
#   ./start_real_test.sh STATIC_LAND 5.0 0.3 0.5
#   ./start_real_test.sh DYNAMIC_LAND 5.0 0.5 0.8
#
# Args: MODE [TAKEOFF_ALT] [PID_KP] [MAX_VEL]

MODE="${1:-HANDHELD}"
ALT="${2:-3.0}"
KP="${3:-0.3}"
VEL="${4:-0.8}"

source /opt/ros/jazzy/setup.bash
cd ~/drone-software
source install/setup.bash

echo "═══════════════════════════════════════════"
echo "  REAL FLIGHT TEST — Mode: $MODE"
echo "  Alt: ${ALT}m  KP: $KP  MaxVel: ${VEL}m/s"
echo "═══════════════════════════════════════════"

# 1. Launch real drone autopilot
if [ "$MODE" != "HANDHELD" ]; then
    echo ">>> [1/3] Launching autopilot (real drone)..."
    ros2 launch thyra thyra.launch.py 2>&1 | tee /tmp/real_test.log &
    AP_PID=$!
else
    echo ">>> [1/3] Launching autopilot (HANDHELD — servo only)..."
    ros2 launch thyra thyra.launch.py 2>&1 | tee /tmp/real_test.log &
    AP_PID=$!
fi

# 2. Launch ArUco detector (uses real RealSense camera)
echo ">>> [2/3] Launching ArUco detector..."
ros2 run thyra aruco_detector.py &
DET_PID=$!

# 3. Wait for autopilot
echo ">>> Waiting for THYRA OPERATIONAL..."
while true; do
    if grep -q "THYRA OPERATIONAL" /tmp/real_test.log 2>/dev/null; then
        echo ">>> Autopilot Ready!"
        break
    fi
    sleep 1
done

# 4. Launch mission
echo ">>> [3/3] Launching real_flight_test ($MODE)..."
ros2 run thyra real_flight_test.py --ros-args \
    -p mode:="$MODE" \
    -p takeoff_alt:="$ALT" \
    -p pid_kp:="$KP" \
    -p max_vel:="$VEL" &
MISSION_PID=$!

echo ""
echo "╔═══════════════════════════════════════╗"
echo "║  Press Ctrl+C to abort at any time    ║"
echo "╚═══════════════════════════════════════╝"

trap "echo 'Shutting down...'; kill $AP_PID $DET_PID $MISSION_PID 2>/dev/null; exit" INT TERM
wait
