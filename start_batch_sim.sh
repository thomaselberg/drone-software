#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════
# start_batch_sim.sh
# ═══════════════════════════════════════════════════════════════════════
# Runs multiple comparison simulations back-to-back.
# All simulation logic is consolidated here (no background sub-scripts).
#
# Usage:
#   ./start_batch_sim.sh
# ═══════════════════════════════════════════════════════════════════════

# ── Configuration ─────────────────────────────────────────────────────
# Each entry: "MODE SCENARIO WIND"
BATCH=(
    "STATIC STATIC none"
    "GIMBAL STATIC none"
    "STATIC DYNAMIC none"
    "GIMBAL DYNAMIC none"
)

# Timeout per run (seconds) — force-kill if mission doesn't finish
RUN_TIMEOUT=300

# ── Colors ────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
BOLD='\033[1m'
NC='\033[0m'

# ── Setup ─────────────────────────────────────────────────────────────
export DISPLAY=${DISPLAY:-:1}
RESULTS_DIR="$HOME/drone-software/results"
mkdir -p "$RESULTS_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SUMMARY_FILE="${RESULTS_DIR}/master_summary_${TIMESTAMP}.csv"

# Write CSV header
echo "run,mode,scenario,wind,linear_error_m,rotation_error_deg,engagement_s,csv_file" \
    > "$SUMMARY_FILE"

echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}${BOLD}   BATCH COMPARISON RUNNER                            ${NC}"
echo -e "${CYAN}   ${#BATCH[@]} runs queued                                  ${NC}"
echo -e "${CYAN}   Summary → ${GREEN}${SUMMARY_FILE}${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"

# ── Source ROS environment once ───────────────────────────────────────
source /opt/ros/jazzy/setup.bash
cd ~/drone-software
source install/setup.bash

# ── QGroundControl — launch once, keep open for all runs ──────────────
QGC_APPIMAGE="$HOME/QGroundControl-x86_64.AppImage"
if [ -f "$QGC_APPIMAGE" ]; then
    if ! pgrep -f "QGroundControl" > /dev/null 2>&1; then
        echo -e "${BLUE}>>> Launching QGroundControl (stays open for all runs)...${NC}"
        chmod +x "$QGC_APPIMAGE"
        "$QGC_APPIMAGE" > /dev/null 2>&1 &
        sleep 2
    else
        echo -e "${YELLOW}>>> QGroundControl already running — reusing${NC}"
    fi
fi

# ── Batch loop ────────────────────────────────────────────────────────
RUN_NUM=0
TOTAL=${#BATCH[@]}

for ENTRY in "${BATCH[@]}"; do
    RUN_NUM=$((RUN_NUM + 1))
    read -r MODE SCENARIO WIND <<< "$ENTRY"

    # ── Scenario-dependent target spawn ──────────────────────────────
    if [ "$SCENARIO" == "DYNAMIC" ]; then
        TARGET_DIST="-1.0"
        CAM_SCENARIO="MOVING"
    else
        TARGET_DIST="10.0"
        CAM_SCENARIO="STATIC"
    fi

    echo ""
    echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}${BOLD}   RUN ${RUN_NUM}/${TOTAL}: ${MODE} | ${SCENARIO} | ${WIND}${NC}"
    echo -e "${CYAN}   Target Spawn: ${TARGET_DIST}m${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"

    # ── 1. Cleanup previous run (surgical — keep QGC alive) ──────────
    echo -e "${YELLOW}>>> Cleaning up previous sessions...${NC}"
    pkill -f "vision_landing_mission"    2>/dev/null
    pkill -f "kpi_logger_sim"            2>/dev/null
    pkill -f "mission_comparison"        2>/dev/null
    pkill -f "synthetic_cam_comparison"  2>/dev/null
    pkill -f "aruco_detector_comparison" 2>/dev/null
    pkill -f "wind_gust_generator"       2>/dev/null
    pkill -f "thyra_sim.launch"          2>/dev/null
    sleep 1
    pkill -f MicroXRCEAgent   2>/dev/null
    pkill -f "gz sim"         2>/dev/null
    pkill -f "ruby"           2>/dev/null
    pkill -f px4              2>/dev/null
    sleep 1
    pkill -9 -f px4           2>/dev/null
    sleep 2

    rm -f /tmp/sim_output_comparison.log

    # ── 2. Launch base simulation ────────────────────────────────────
    echo -e "${BLUE}>>> [1/5] Launching simulation...${NC}"
    ros2 launch thyra thyra_sim.launch.py 2>&1 | tee /tmp/sim_output_comparison.log &
    SIM_PID=$!

    # ── 3. Launch Synthetic Camera ───────────────────────────────────
    echo -e "${BLUE}>>> [2/5] Launching synthetic camera...${NC}"
    ros2 run thyra synthetic_cam_comparison.py --ros-args \
        -p target_start_x:="${TARGET_DIST}" \
        -p target_start_y:=0.0 \
        -p scenario:="${CAM_SCENARIO}" \
        -p marker_size_m:=0.6 \
        -p whiteout_interval_s:=10.0 \
        -p whiteout_duration_s:=0.2 \
        -p camera_pitch_deg:=45.0 &
    CAM_PID=$!

    # ── 4. Launch ArUco Detector (C++) ───────────────────────────────
    echo -e "${BLUE}>>> [3/5] Launching ArUco detector (C++)...${NC}"
    ros2 run thyra aruco_detector_comparison --ros-args -p show_window:=true &
    DET_PID=$!

    # ── 5. Launch Wind Gust Generator ────────────────────────────────
    WIND_PID=""
    if [ "$WIND" != "none" ]; then
        echo -e "${BLUE}>>> [4/5] Launching wind gust generator (${WIND})...${NC}"
        ros2 run thyra wind_gust_generator.py --ros-args \
            -p scenario:="${WIND}" &
        WIND_PID=$!
    else
        echo -e "${YELLOW}>>> [4/5] Skipping wind gust generator (none)...${NC}"
    fi

    # ── 6. Wait for autopilot readiness ──────────────────────────────
    echo -e "${YELLOW}>>> Waiting for THYRA OPERATIONAL...${NC}"
    WAIT_COUNT=0
    while true; do
        if grep -q "THYRA OPERATIONAL" /tmp/sim_output_comparison.log 2>/dev/null; then
            echo -e "${GREEN}>>> Autopilot Ready!${NC}"
            break
        fi
        WAIT_COUNT=$((WAIT_COUNT + 1))
        if [ $WAIT_COUNT -ge 120 ]; then
            echo -e "${RED}>>> Autopilot did not start in 120s — skipping run${NC}"
            break
        fi
        sleep 1
    done

    if [ $WAIT_COUNT -ge 120 ]; then
        echo "${RUN_NUM},${MODE},${SCENARIO},${WIND},STARTUP_FAIL,STARTUP_FAIL,STARTUP_FAIL,N/A" \
            >> "$SUMMARY_FILE"
        continue
    fi

    # ── 7. Launch KPI Logger (sim) ───────────────────────────────────
    # Snapshot existing CSV files BEFORE launching logger + mission
    BEFORE_CSVS=$(ls -1 "$RESULTS_DIR"/*.csv 2>/dev/null | sort)

    echo -e "${BLUE}>>> [5a/5] Launching KPI logger (sim)...${NC}"
    ros2 run thyra kpi_logger_sim.py --ros-args \
        -p mode:="${MODE}" \
        -p scenario:="${SCENARIO}" \
        -p wind_scenario:="${WIND}" &
    KPI_PID=$!

    # ── 8. Launch Mission Controller ─────────────────────────────────
    echo -e "${GREEN}>>> [5b/5] Launching vision_landing_mission (${MODE}, ${SCENARIO}, ${WIND})...${NC}"
    ros2 run thyra vision_landing_mission.py --ros-args \
        -p mode:="${MODE}" \
        -p scenario:="${SCENARIO}" \
        -p wind_scenario:="${WIND}" \
        -p takeoff_alt:=1.5 \
        -p target_start_x:="${TARGET_DIST}" \
        -p target_start_y:=0.0 &
    MISSION_PID=$!

    # ── 9. Wait for mission node to appear ───────────────────────────
    echo -e "${YELLOW}>>> Waiting for mission node to start...${NC}"
    for i in $(seq 1 30); do
        if pgrep -f "vision_landing_mission" > /dev/null 2>&1; then
            echo -e "${GREEN}>>> Mission node running (PID: $(pgrep -f vision_landing_mission | head -1))${NC}"
            break
        fi
        sleep 1
    done

    # ── 10. Monitor mission until CSV or timeout ─────────────────────
    ELAPSED=0
    NEW_CSV=""
    while [ $ELAPSED -lt $RUN_TIMEOUT ]; do
        sleep 5
        ELAPSED=$((ELAPSED + 5))

        # Check for new CSV files
        AFTER_CSVS=$(ls -1 "$RESULTS_DIR"/*.csv 2>/dev/null | sort)
        NEW_CSV=$(comm -13 <(echo "$BEFORE_CSVS") <(echo "$AFTER_CSVS") | grep -v "master_summary" | head -1)

        if [ -n "$NEW_CSV" ]; then
            echo -e "${GREEN}>>> CSV detected: $(basename "$NEW_CSV")${NC}"
            sleep 3  # Grace period
            break
        fi

        # Check if mission node has exited (only after it was confirmed running)
        if ! pgrep -f "vision_landing_mission" > /dev/null 2>&1; then
            echo -e "${YELLOW}>>> Mission node exited${NC}"
            sleep 2
            AFTER_CSVS=$(ls -1 "$RESULTS_DIR"/*.csv 2>/dev/null | sort)
            NEW_CSV=$(comm -13 <(echo "$BEFORE_CSVS") <(echo "$AFTER_CSVS") | grep -v "master_summary" | head -1)
            break
        fi
    done

    if [ $ELAPSED -ge $RUN_TIMEOUT ]; then
        echo -e "${RED}>>> TIMEOUT after ${RUN_TIMEOUT}s — force killing${NC}"
    fi

    # ── 10. Extract KPIs from CSV and append to summary ──────────────
    if [ -n "$NEW_CSV" ] && [ -f "$NEW_CSV" ]; then
        DATA_LINE=$(tail -1 "$NEW_CSV")
        LINEAR_ERR=$(echo "$DATA_LINE" | cut -d',' -f5)
        ROT_ERR=$(echo "$DATA_LINE" | cut -d',' -f6)
        ENGAGE=$(echo "$DATA_LINE" | cut -d',' -f7)
        echo "${RUN_NUM},${MODE},${SCENARIO},${WIND},${LINEAR_ERR},${ROT_ERR},${ENGAGE},$(basename "$NEW_CSV")" \
            >> "$SUMMARY_FILE"
        echo -e "${GREEN}  Linear Error  : ${LINEAR_ERR} m${NC}"
        echo -e "${GREEN}  Rotation Error: ${ROT_ERR}°${NC}"
        echo -e "${GREEN}  Engagement    : ${ENGAGE} s${NC}"
    else
        echo "${RUN_NUM},${MODE},${SCENARIO},${WIND},TIMEOUT,TIMEOUT,TIMEOUT,N/A" \
            >> "$SUMMARY_FILE"
        echo -e "${RED}  No CSV produced — logged as TIMEOUT${NC}"
    fi
done

# ── Final cleanup ─────────────────────────────────────────────────────
echo -e "${YELLOW}>>> Final cleanup...${NC}"
pkill -f "vision_landing_mission"    2>/dev/null
pkill -f "kpi_logger_sim"            2>/dev/null
pkill -f "mission_comparison"        2>/dev/null
pkill -f "synthetic_cam_comparison"  2>/dev/null
pkill -f "aruco_detector_comparison" 2>/dev/null
pkill -f "wind_gust_generator"       2>/dev/null
pkill -f "thyra_sim.launch"          2>/dev/null
sleep 1
pkill -f MicroXRCEAgent   2>/dev/null
pkill -f "gz sim"         2>/dev/null
pkill -f "ruby"           2>/dev/null
pkill -f px4              2>/dev/null
sleep 1
pkill -9 -f px4           2>/dev/null

# ── Final summary ────────────────────────────────────────────────────
echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}${BOLD}   BATCH COMPLETE — ${TOTAL} runs finished${NC}"
echo -e "${CYAN}   Summary → ${GREEN}${SUMMARY_FILE}${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${YELLOW}Results:${NC}"
column -t -s',' "$SUMMARY_FILE"
