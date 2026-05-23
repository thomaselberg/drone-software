#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════
# start_sim_base.sh
# ═══════════════════════════════════════════════════════════════════════
# Long-lived sim base stack. Mirrors start_base.sh (real flight) for the
# sim environment. Launches Gazebo + PX4 SITL + autopilot + synthetic
# camera + ArUco detector + KPI logger, then waits. The mission is
# started SEPARATELY in another terminal.
#
# Two-terminal workflow (identical shape to real flight):
#
#   Terminal A:  ./start_sim_base.sh GIMBAL DYNAMIC
#   Terminal B:  ros2 run thyra vision_landing_mission.py --ros-args \
#                    -p mode:=GIMBAL -p scenario:=DYNAMIC \
#                    -p takeoff_alt:=1.5
#
# Use this for iterative tuning: kill the mission with Ctrl+C, edit
# gains, run it again — Gazebo and PX4 stay up the whole time.
#
# For one-shot end-to-end runs (and the batch runner), use
# start_sim.sh instead — it does its own teardown.
#
# Usage:
#   ./start_sim_base.sh                            # defaults: GIMBAL DYNAMIC
#   ./start_sim_base.sh STATIC STATIC
#   ./start_sim_base.sh GIMBAL DYNAMIC_EASY
# ═══════════════════════════════════════════════════════════════════════

# ── Arguments ─────────────────────────────────────────────────────────
MODE_RAW="${1:-GIMBAL}"
MODE=$(echo "$MODE_RAW" | tr '[:lower:]' '[:upper:]')
SCENARIO_RAW="${2:-STATIC}"
SCENARIO=$(echo "$SCENARIO_RAW" | tr '[:lower:]' '[:upper:]')

# ── Scenario-dependent target spawn + camera scenario ────────────────
case "$SCENARIO" in
    DYNAMIC)
        TARGET_DIST="-1.0"
        CAM_SCENARIO="MOVING"
        ;;
    DYNAMIC_EASY)
        TARGET_DIST="1.0"
        CAM_SCENARIO="MOVING_EASY"
        ;;
    *)
        TARGET_DIST="10.0"
        CAM_SCENARIO="STATIC"
        ;;
esac

# ── Colors ────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
BOLD='\033[1m'
NC='\033[0m'

# ── Display ───────────────────────────────────────────────────────────
export DISPLAY=${DISPLAY:-:1}

echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"
echo -e "${CYAN}${BOLD}   SIM BASE STACK (mission runs in a second terminal)${NC}"
echo -e "${CYAN}   Mode: ${GREEN}${MODE}${CYAN}   Scenario: ${GREEN}${SCENARIO}${NC}"
echo -e "${CYAN}   Target spawn: ${GREEN}${TARGET_DIST} m${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"

# ── 1. Cleanup previous sessions ──────────────────────────────────────
echo -e "${YELLOW}>>> Cleaning up previous sessions...${NC}"
pkill -9 -f "vision_landing_mission" 2>/dev/null
pkill -9 -f "bench_dry_test"         2>/dev/null
pkill -9 -f "kpi_logger_sim"         2>/dev/null
pkill -9 -f "kpi_logger_real"        2>/dev/null
pkill -9 -f "synthetic_cam" 2>/dev/null
pkill -9 -f "aruco_detector" 2>/dev/null
pkill -9 -f "thyra_sim.launch"       2>/dev/null
pkill -9 -f MicroXRCEAgent           2>/dev/null
pkill -9 -f "gz sim"                 2>/dev/null
pkill -9 -f "ruby"                   2>/dev/null
pkill -9 -f px4                      2>/dev/null
sleep 2

rm -f /tmp/sim_base.log

# ── 2. Source environment ─────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
cd ~/drone-software
source install/setup.bash

# ── 3. QGroundControl (optional) ──────────────────────────────────────
SKIP_QGC=false
for arg in "$@"; do
    if [ "$arg" == "--no-qgc" ]; then
        SKIP_QGC=true
    fi
done

QGC_APPIMAGE="$HOME/QGroundControl-x86_64.AppImage"
if [ "$SKIP_QGC" == "false" ] && [ -f "$QGC_APPIMAGE" ]; then
    if ! pgrep -f "QGroundControl" > /dev/null 2>&1; then
        echo -e "${BLUE}>>> Launching QGroundControl...${NC}"
        chmod +x "$QGC_APPIMAGE"
        "$QGC_APPIMAGE" > /dev/null 2>&1 &
    else
        echo -e "${YELLOW}>>> QGroundControl already running — reusing${NC}"
    fi
elif [ "$SKIP_QGC" == "true" ]; then
    echo -e "${YELLOW}>>> QGroundControl skipped (--no-qgc)${NC}"
fi

# ── 4. Launch base simulation (Gazebo + PX4 SITL + autopilot) ────────
echo -e "${BLUE}>>> [1/4] Launching thyra_sim.launch.py...${NC}"
ros2 launch thyra thyra_sim.launch.py 2>&1 | tee /tmp/sim_base.log &
SIM_PID=$!

# ── 5. Wait for autopilot readiness ──────────────────────────────────
echo -e "${YELLOW}>>> Waiting for THYRA OPERATIONAL...${NC}"
WAIT_COUNT=0
while true; do
    if grep -q "THYRA OPERATIONAL" /tmp/sim_base.log 2>/dev/null; then
        echo -e "${GREEN}>>> Autopilot Ready!${NC}"
        break
    fi
    WAIT_COUNT=$((WAIT_COUNT + 1))
    if [ $WAIT_COUNT -ge 120 ]; then
        echo -e "${RED}>>> Autopilot did not start in 120s — aborting${NC}"
        kill $SIM_PID 2>/dev/null
        exit 1
    fi
    sleep 1
done

# ── 6. Launch Synthetic Camera + Target Engine ───────────────────────
echo -e "${BLUE}>>> [2/4] Launching synthetic camera (${CAM_SCENARIO})...${NC}"
ros2 run thyra synthetic_cam.py --ros-args \
    -p target_start_x:="${TARGET_DIST}" \
    -p target_start_y:=0.0 \
    -p scenario:="${CAM_SCENARIO}" \
    -p marker_size_m:=0.3 \
    -p whiteout_interval_s:=10.0 \
    -p whiteout_duration_s:=0.2 \
    -p camera_pitch_deg:=45.0 &
CAM_PID=$!

# ── 7. Launch ArUco Detector (C++) ───────────────────────────────────
echo -e "${BLUE}>>> [3/4] Launching ArUco detector (C++)...${NC}"
ros2 run thyra aruco_detector --ros-args -p show_window:=true &
DET_PID=$!

# ── 8. Launch KPI Logger (sim) ───────────────────────────────────────
echo -e "${BLUE}>>> [4/4] Launching KPI logger (sim)...${NC}"
ros2 run thyra kpi_logger_sim.py --ros-args \
    -p mode:="${MODE}" \
    -p scenario:="${SCENARIO}" &
KPI_PID=$!

# ── Banner ────────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}╔═══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  Sim base up. In another terminal:                            ║${NC}"
echo -e "${GREEN}║                                                               ║${NC}"
echo -e "${GREEN}║    cd ~/drone-software && source install/setup.bash           ║${NC}"
echo -e "${GREEN}║    ros2 run thyra vision_landing_mission.py --ros-args \\      ║${NC}"
echo -e "${GREEN}║      -p mode:=${MODE} -p scenario:=${SCENARIO} \\                   ║${NC}"
echo -e "${GREEN}║      -p takeoff_alt:=1.5 \\                                    ║${NC}"
echo -e "${GREEN}║      -p target_start_x:=${TARGET_DIST} -p target_start_y:=0.0       ║${NC}"
echo -e "${GREEN}║                                                               ║${NC}"
echo -e "${GREEN}║  Ctrl+C the mission to re-run with new params; the base       ║${NC}"
echo -e "${GREEN}║  stays up. Ctrl+C here when fully done.                       ║${NC}"
echo -e "${GREEN}║  CSV results → ~/drone-software/results/                      ║${NC}"
echo -e "${GREEN}╚═══════════════════════════════════════════════════════════════╝${NC}"
echo ""

# ── Shutdown handler ──────────────────────────────────────────────────
trap "echo -e '${RED}Shutting down sim base...${NC}'; \
      kill \$SIM_PID \$CAM_PID \$DET_PID \$KPI_PID 2>/dev/null; \
      sleep 1; \
      pkill -9 -f vision_landing_mission 2>/dev/null; \
      pkill -9 -f bench_dry_test 2>/dev/null; \
      pkill -9 -f kpi_logger 2>/dev/null; \
      pkill -9 -f synthetic_cam 2>/dev/null; \
      pkill -9 -f aruco_detector 2>/dev/null; \
      pkill -9 -f thyra_sim.launch 2>/dev/null; \
      pkill -9 -f MicroXRCEAgent 2>/dev/null; \
      pkill -9 -f 'gz sim' 2>/dev/null; \
      pkill -9 -f px4 2>/dev/null; \
      exit" INT TERM

wait
