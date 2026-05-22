#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════
# start_batch_sim.sh
# ═══════════════════════════════════════════════════════════════════════
# Runs the sim end-to-end for every (MODE, SCENARIO) combination in
# BATCH, REPEATS times each. Each run is fully self-contained: cleanup
# → launch sim/cam/detector/mission/logger → wait for CSV → kill.
#
# Usage:
#   ./start_batch_sim.sh
# ═══════════════════════════════════════════════════════════════════════

# ── Configuration ─────────────────────────────────────────────────────
# Each entry: "MODE SCENARIO"
BATCH=(
    "STATIC STATIC"
    "GIMBAL STATIC"
    "STATIC DYNAMIC"
    "GIMBAL DYNAMIC"
    "STATIC DYNAMIC_EASY"
    "GIMBAL DYNAMIC_EASY"
)

# Number of repeats per combination — total runs = ${#BATCH[@]} × REPEATS
REPEATS=5

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

# Write CSV header — `repeat` column added so aggregate_batch.py can group
echo "run,repeat,mode,scenario,linear_error_m,rotation_error_deg,engagement_s,csv_file" \
    > "$SUMMARY_FILE"

TOTAL_RUNS=$((${#BATCH[@]} * REPEATS))

echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}${BOLD}   BATCH COMPARISON RUNNER                            ${NC}"
echo -e "${CYAN}   ${#BATCH[@]} combinations × ${REPEATS} repeats = ${TOTAL_RUNS} runs${NC}"
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

# ── Batch loop (combinations × repeats) ──────────────────────────────
RUN_NUM=0

for ENTRY in "${BATCH[@]}"; do
    read -r MODE SCENARIO <<< "$ENTRY"
    MODE=$(echo "$MODE"         | tr '[:lower:]' '[:upper:]')
    SCENARIO=$(echo "$SCENARIO" | tr '[:lower:]' '[:upper:]')

    # ── Scenario-dependent target spawn + camera scenario ───────────
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

    for REP in $(seq 1 $REPEATS); do
    RUN_NUM=$((RUN_NUM + 1))

    echo ""
    echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}${BOLD}   RUN ${RUN_NUM}/${TOTAL_RUNS}: ${MODE} | ${SCENARIO}  (repeat ${REP}/${REPEATS})${NC}"
    echo -e "${CYAN}   Target Spawn: ${TARGET_DIST}m${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"

    # ── 1. Cleanup previous run (surgical — keep QGC alive) ──────────
    echo -e "${YELLOW}>>> Cleaning up previous sessions...${NC}"
    pkill -f "vision_landing_mission"    2>/dev/null
    pkill -f "kpi_logger_sim"            2>/dev/null
    pkill -f "synthetic_cam"  2>/dev/null
    pkill -f "aruco_detector" 2>/dev/null
    pkill -f "thyra_sim.launch"          2>/dev/null
    sleep 1
    pkill -f MicroXRCEAgent   2>/dev/null
    pkill -f "gz sim"         2>/dev/null
    pkill -f "ruby"           2>/dev/null
    pkill -f px4              2>/dev/null
    sleep 1
    pkill -9 -f px4           2>/dev/null
    sleep 2

    rm -f /tmp/sim_output.log

    # ── 2. Launch base simulation ────────────────────────────────────
    echo -e "${BLUE}>>> [1/4] Launching simulation...${NC}"
    ros2 launch thyra thyra_sim.launch.py 2>&1 | tee /tmp/sim_output.log &
    SIM_PID=$!

    # ── 3. Launch Synthetic Camera ───────────────────────────────────
    echo -e "${BLUE}>>> [2/4] Launching synthetic camera...${NC}"
    ros2 run thyra synthetic_cam.py --ros-args \
        -p target_start_x:="${TARGET_DIST}" \
        -p target_start_y:=0.0 \
        -p scenario:="${CAM_SCENARIO}" \
        -p marker_size_m:=0.3 \
        -p whiteout_interval_s:=10.0 \
        -p whiteout_duration_s:=0.2 \
        -p camera_pitch_deg:=45.0 &
    CAM_PID=$!

    # ── 4. Launch ArUco Detector (C++) ───────────────────────────────
    echo -e "${BLUE}>>> [3/4] Launching ArUco detector (C++)...${NC}"
    ros2 run thyra aruco_detector --ros-args -p show_window:=true &
    DET_PID=$!

    # ── 5. Wait for autopilot readiness ──────────────────────────────
    echo -e "${YELLOW}>>> Waiting for THYRA OPERATIONAL...${NC}"
    WAIT_COUNT=0
    while true; do
        if grep -q "THYRA OPERATIONAL" /tmp/sim_output.log 2>/dev/null; then
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
        echo "${RUN_NUM},${REP},${MODE},${SCENARIO},STARTUP_FAIL,STARTUP_FAIL,STARTUP_FAIL,N/A" \
            >> "$SUMMARY_FILE"
        continue
    fi

    # ── 6. Launch KPI Logger (sim) ───────────────────────────────────
    # Snapshot existing CSV files BEFORE launching logger + mission
    BEFORE_CSVS=$(ls -1 "$RESULTS_DIR"/*.csv 2>/dev/null | sort)

    echo -e "${BLUE}>>> [4a/4] Launching KPI logger (sim)...${NC}"
    ros2 run thyra kpi_logger_sim.py --ros-args \
        -p mode:="${MODE}" \
        -p scenario:="${SCENARIO}" &
    KPI_PID=$!

    # ── 7. Launch Mission Controller ─────────────────────────────────
    echo -e "${GREEN}>>> [4b/4] Launching vision_landing_mission (${MODE}, ${SCENARIO})...${NC}"
    ros2 run thyra vision_landing_mission.py --ros-args \
        -p mode:="${MODE}" \
        -p scenario:="${SCENARIO}" \
        -p takeoff_alt:=3.0 \
        -p target_start_x:="${TARGET_DIST}" \
        -p target_start_y:=0.0 &
    MISSION_PID=$!

    # ── 8. Wait for mission node to appear ───────────────────────────
    echo -e "${YELLOW}>>> Waiting for mission node to start...${NC}"
    for i in $(seq 1 30); do
        if pgrep -f "vision_landing_mission" > /dev/null 2>&1; then
            echo -e "${GREEN}>>> Mission node running (PID: $(pgrep -f vision_landing_mission | head -1))${NC}"
            break
        fi
        sleep 1
    done

    # ── 9. Monitor mission until CSV or timeout ──────────────────────
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
        LINEAR_ERR=$(echo "$DATA_LINE" | cut -d',' -f4)
        ROT_ERR=$(echo "$DATA_LINE" | cut -d',' -f5)
        ENGAGE=$(echo "$DATA_LINE" | cut -d',' -f6)
        echo "${RUN_NUM},${REP},${MODE},${SCENARIO},${LINEAR_ERR},${ROT_ERR},${ENGAGE},$(basename "$NEW_CSV")" \
            >> "$SUMMARY_FILE"
        echo -e "${GREEN}  Linear Error  : ${LINEAR_ERR} m${NC}"
        echo -e "${GREEN}  Rotation Error: ${ROT_ERR}°${NC}"
        echo -e "${GREEN}  Engagement    : ${ENGAGE} s${NC}"
    else
        echo "${RUN_NUM},${REP},${MODE},${SCENARIO},TIMEOUT,TIMEOUT,TIMEOUT,N/A" \
            >> "$SUMMARY_FILE"
        echo -e "${RED}  No CSV produced — logged as TIMEOUT${NC}"
    fi
    done   # end REP loop
done       # end BATCH loop

# ── Final cleanup ─────────────────────────────────────────────────────
echo -e "${YELLOW}>>> Final cleanup...${NC}"
pkill -f "vision_landing_mission"    2>/dev/null
pkill -f "kpi_logger_sim"            2>/dev/null
pkill -f "synthetic_cam"  2>/dev/null
pkill -f "aruco_detector" 2>/dev/null
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
echo -e "${CYAN}${BOLD}   BATCH COMPLETE — ${RUN_NUM}/${TOTAL_RUNS} runs finished${NC}"
echo -e "${CYAN}   Summary  → ${GREEN}${SUMMARY_FILE}${NC}"

# ── Aggregate per-combination statistics ─────────────────────────────
AGGREGATE_FILE="${RESULTS_DIR}/aggregate_batch_${TIMESTAMP}.csv"
echo -e "${BLUE}>>> Aggregating per-combination statistics...${NC}"
if ros2 run thyra aggregate_batch.py "$SUMMARY_FILE" "$AGGREGATE_FILE" 2>&1; then
    echo -e "${CYAN}   Aggregate → ${GREEN}${AGGREGATE_FILE}${NC}"
else
    echo -e "${RED}>>> Aggregation failed — master summary is still available${NC}"
fi
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${YELLOW}Raw rows:${NC}"
column -t -s',' "$SUMMARY_FILE"
