#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════
# start_real_flight.sh
# ═══════════════════════════════════════════════════════════════════════
# Real flight launcher for the unified vision_landing_mission.
# Runs on the Raspberry Pi (no display). GUI runs separately on the SSH PC
# and can take over control at any time via /asr/thyra/in/manual_input
# and the GUI's Land button (DroneCommand action 'land').
#
# Mode is implicitly GIMBAL (only mode supported on real hardware).
# Wind generator is not started (sim-only).
#
# Usage:
#   ./start_real_flight.sh STATIC          # known target, fly toward, then land
#   ./start_real_flight.sh DYNAMIC         # hover at takeoff_alt, wait for moving target
#   ./start_real_flight.sh DYNAMIC 1.5     # override takeoff_alt (default 1.5 m)
#
# Telemetry from the SSH PC (same ROS 2 network):
#   ros2 topic echo /asr/mission/state                     # state machine + key vars
#   rqt_image_view /camera/camera/color/image_raw          # raw RealSense
#   rqt_image_view /asr/aruco/detector_image    # annotated detector overlay
# ═══════════════════════════════════════════════════════════════════════

# ── Arguments ─────────────────────────────────────────────────────────
SCENARIO_RAW="${1:-DYNAMIC}"
SCENARIO=$(echo "$SCENARIO_RAW" | tr '[:lower:]' '[:upper:]')
TAKEOFF_ALT="${2:-1.5}"

if [ "$SCENARIO" != "STATIC" ] && [ "$SCENARIO" != "DYNAMIC" ]; then
    echo "Error: scenario must be STATIC or DYNAMIC (got '$SCENARIO_RAW')"
    exit 1
fi

# Mode is fixed for real flight
MODE="GIMBAL"

# Known marker location for STATIC scenario (place the marker accordingly).
# Override with TARGET_X / TARGET_Y env vars, e.g.:
#   TARGET_X=2.0 TARGET_Y=0.5 ./start_real_flight.sh STATIC
TARGET_X="${TARGET_X:-1.0}"
TARGET_Y="${TARGET_Y:-0.0}"

# ── Colors ────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"
echo -e "${CYAN}   REAL FLIGHT — vision_landing_mission           ${NC}"
echo -e "${CYAN}   Mode: ${GREEN}${MODE}${CYAN}   Scenario: ${GREEN}${SCENARIO}${NC}"
echo -e "${CYAN}   Takeoff alt: ${GREEN}${TAKEOFF_ALT} m${NC}"
if [ "$SCENARIO" == "STATIC" ]; then
    echo -e "${CYAN}   Target (known): ${GREEN}(${TARGET_X}, ${TARGET_Y}) m${NC}"
fi
echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"

# ── 1. Cleanup previous sessions ──────────────────────────────────────
echo -e "${YELLOW}>>> Cleaning up previous sessions...${NC}"
pkill -9 -f "vision_landing_mission"  2>/dev/null
pkill -9 -f "kpi_logger_real"         2>/dev/null
pkill -9 -f "kpi_logger_sim"          2>/dev/null
pkill -9 -f "real_flight_test"        2>/dev/null
pkill -9 -f "aruco_detector" 2>/dev/null
pkill -9 -f "thyra.launch"            2>/dev/null
pkill -9 -f MicroXRCEAgent            2>/dev/null
sleep 2

rm -f /tmp/real_flight.log

# ── 2. Source environment ─────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
cd ~/drone-software
source install/setup.bash

# ── 3. Launch real-drone autopilot ────────────────────────────────────
# thyra.launch.py brings up:
#   - MicroXRCEAgent on serial /dev/ttyAMA0
#   - LED node
#   - RealSense + image_republisher (via thyra_cam.launch.py)
#   - asr_autopilot (after 15s delay)
echo -e "${BLUE}>>> [1/4] Launching real autopilot + RealSense...${NC}"
ros2 launch thyra thyra.launch.py 2>&1 | tee /tmp/real_flight.log &
AP_PID=$!

# ── 4. Launch ArUco Detector (C++, headless) ──────────────────────────
echo -e "${BLUE}>>> [2/4] Launching ArUco detector (C++, headless)...${NC}"
ros2 run thyra aruco_detector --ros-args -p show_window:=false &
DET_PID=$!

# ── 5. Wait for autopilot readiness ───────────────────────────────────
echo -e "${YELLOW}>>> Waiting for THYRA OPERATIONAL...${NC}"
WAIT_COUNT=0
while true; do
    if grep -q "THYRA OPERATIONAL" /tmp/real_flight.log 2>/dev/null; then
        echo -e "${GREEN}>>> Autopilot Ready!${NC}"
        break
    fi
    WAIT_COUNT=$((WAIT_COUNT + 1))
    if [ $WAIT_COUNT -ge 120 ]; then
        echo -e "${RED}>>> Autopilot did not start in 120s — aborting${NC}"
        kill $AP_PID $DET_PID 2>/dev/null
        exit 1
    fi
    sleep 1
done

# ── 6. Launch KPI Logger (real) ───────────────────────────────────────
echo -e "${BLUE}>>> [3/4] Launching KPI logger (real)...${NC}"
ros2 run thyra kpi_logger_real.py --ros-args \
    -p scenario:="${SCENARIO}" &
KPI_PID=$!

# ── 7. Launch Mission Controller ──────────────────────────────────────
echo -e "${GREEN}>>> [4/4] Launching vision_landing_mission (${MODE}, ${SCENARIO})...${NC}"
ros2 run thyra vision_landing_mission.py --ros-args \
    -p mode:="${MODE}" \
    -p scenario:="${SCENARIO}" \
    -p takeoff_alt:="${TAKEOFF_ALT}" \
    -p target_start_x:="${TARGET_X}" \
    -p target_start_y:="${TARGET_Y}" &
MISSION_PID=$!

# ── Shutdown handler ──────────────────────────────────────────────────
echo ""
echo -e "${GREEN}╔═══════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  All nodes launched. Press Ctrl+C to stop.   ║${NC}"
echo -e "${GREEN}║  Use the GUI on the SSH PC to take over or   ║${NC}"
echo -e "${GREEN}║  click Land at any time.                     ║${NC}"
echo -e "${GREEN}║  CSV results → ~/drone-software/results/      ║${NC}"
echo -e "${GREEN}╚═══════════════════════════════════════════════╝${NC}"

trap "echo -e '${RED}Shutting down all nodes...${NC}'; \
      kill $AP_PID $DET_PID $KPI_PID $MISSION_PID 2>/dev/null; \
      sleep 1; \
      pkill -9 -f MicroXRCEAgent 2>/dev/null; \
      exit" INT TERM

wait
