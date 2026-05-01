#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════
# start_comparison_sim.sh
# ═══════════════════════════════════════════════════════════════════════
# Launches the Fair Comparison scenario (separate from Mission 0).
#
# Usage:
#   ./start_comparison_sim.sh              # defaults: GIMBAL, gust1
#   ./start_comparison_sim.sh STATIC gust2
#   ./start_comparison_sim.sh GIMBAL gust3
# ═══════════════════════════════════════════════════════════════════════

# ── Arguments ─────────────────────────────────────────────────────────
# ── Arguments ─────────────────────────────────────────────────────────
MODE_RAW="${1:-GIMBAL}"
MODE=$(echo "$MODE_RAW" | tr '[:lower:]' '[:upper:]')
SCENARIO_RAW="${2:-DYNAMIC}" # STATIC or DYNAMIC
SCENARIO=$(echo "$SCENARIO_RAW" | tr '[:lower:]' '[:upper:]')
WIND="${3:-none}"
TARGET_DIST="10.0"   # Meters North

# Map DYNAMIC (mission) to MOVING (camera)
if [ "$SCENARIO" == "DYNAMIC" ]; then
    CAM_SCENARIO="MOVING"
else
    CAM_SCENARIO="STATIC"
fi

# ── Colors ────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m'

# ── Display ───────────────────────────────────────────────────────────
export DISPLAY=${DISPLAY:-:1}

echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"
echo -e "${CYAN}   FAIR COMPARISON SIMULATION                     ${NC}"
echo -e "${CYAN}   Mode: ${GREEN}${MODE}${CYAN}   Wind: ${GREEN}${WIND}${NC}"
echo -e "${CYAN}   Target Distance: ${GREEN}${TARGET_DIST}m${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"

# ── 1. Cleanup ────────────────────────────────────────────────────────
echo -e "${YELLOW}>>> Cleaning up previous sessions...${NC}"
pkill -9 -f MicroXRCEAgent   2>/dev/null
pkill -9 -f px4              2>/dev/null
pkill -9 -f "gz sim"         2>/dev/null
pkill -9 -f "ruby"           2>/dev/null
pkill -9 -f "thyra"          2>/dev/null
pkill -9 -f "mission"        2>/dev/null
pkill -9 -f "synthetic"      2>/dev/null
pkill -9 -f "aruco_detector" 2>/dev/null
pkill -9 -f "wind_gust"      2>/dev/null
pkill -9 -f "python3"        2>/dev/null
sleep 2

rm -f /tmp/sim_output_comparison.log

# ── 2. Source environment ─────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
cd ~/drone-software
source install/setup.bash

# ── 3. QGroundControl (optional) ──────────────────────────────────────
QGC_APPIMAGE="$HOME/QGroundControl-x86_64.AppImage"
if [ -f "$QGC_APPIMAGE" ]; then
    echo -e "${BLUE}>>> Launching QGroundControl...${NC}"
    chmod +x "$QGC_APPIMAGE"
    "$QGC_APPIMAGE" > /dev/null 2>&1 &
fi

# ── 4. Launch base simulation (team launch) ──────────────────────────
echo -e "${BLUE}>>> [1/5] Launching official team simulation...${NC}"
ros2 launch thyra thyra_sim.launch.py 2>&1 | tee /tmp/sim_output_comparison.log &
SIM_PID=$!

# ── 5. Launch Synthetic Camera + Target Engine ────────────────────────
echo -e "${BLUE}>>> [2/5] Launching comparison synthetic camera...${NC}"
ros2 run thyra synthetic_cam_comparison.py --ros-args \
    -p target_start_x:="${TARGET_DIST}" \
    -p target_start_y:=0.0 \
    -p scenario:="${CAM_SCENARIO}" \
    -p whiteout_interval_s:=10.0 \
    -p whiteout_duration_s:=0.2 \
    -p camera_pitch_deg:=45.0 &
CAM_PID=$!

# ── 6. Launch ArUco Detector ─────────────────────────────────────────
echo -e "${BLUE}>>> [3/5] Launching ArUco detector...${NC}"
ros2 run thyra aruco_detector_comparison.py &
DET_PID=$!

# ── 7. Launch Wind Gust Generator ────────────────────────────────────
if [ "$WIND" != "none" ]; then
    echo -e "${BLUE}>>> [4/5] Launching wind gust generator (${WIND})...${NC}"
    ros2 run thyra wind_gust_generator.py --ros-args \
        -p scenario:="${WIND}" &
    WIND_PID=$!
else
    echo -e "${YELLOW}>>> [4/5] Skipping wind gust generator (none)...${NC}"
    WIND_PID=""
fi

# ── 8. Wait for autopilot readiness ──────────────────────────────────
echo -e "${YELLOW}>>> Waiting for THYRA OPERATIONAL...${NC}"
while true; do
    if grep -q "THYRA OPERATIONAL" /tmp/sim_output_comparison.log 2>/dev/null; then
        echo -e "${GREEN}>>> Autopilot Ready!${NC}"
        break
    fi
    sleep 1
done

# ── 9. Launch Mission Controller ─────────────────────────────────────
echo -e "${GREEN}>>> [5/5] Launching mission_comparison (${MODE}, ${WIND})...${NC}"
ros2 run thyra mission_comparison.py --ros-args \
    -p mode:="${MODE}" \
    -p scenario:="${SCENARIO}" \
    -p wind_scenario:="${WIND}" \
    -p target_start_x:="${TARGET_DIST}" \
    -p target_start_y:=0.0 &
MISSION_PID=$!

# ── Shutdown handler ──────────────────────────────────────────────────
echo ""
echo -e "${GREEN}╔═══════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  All nodes launched. Press Ctrl+C to stop.   ║${NC}"
echo -e "${GREEN}║  Results → ~/drone-software/results/          ║${NC}"
echo -e "${GREEN}╚═══════════════════════════════════════════════╝${NC}"

trap "echo -e '${RED}Shutting down all nodes...${NC}'; \
      kill $SIM_PID $CAM_PID $DET_PID $WIND_PID $MISSION_PID 2>/dev/null; \
      exit" INT TERM

wait
