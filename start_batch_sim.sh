#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════
# start_batch_sim.sh
# ═══════════════════════════════════════════════════════════════════════
# Runs multiple comparison simulations back-to-back with optional
# full-screen ffmpeg recording and a master summary CSV.
#
# Usage:
#   ./start_batch_sim.sh              # run all combos, with recording
#   ./start_batch_sim.sh --no-record  # skip ffmpeg recording
#
#    "STATIC DYNAMIC none"
#    "GIMBAL DYNAMIC none"
#    "STATIC STATIC gust2"
#    "GIMBAL STATIC gust2"
#    "STATIC DYNAMIC gust2"
#    "GIMBAL DYNAMIC gust2"
═══════════════════════════════════════════════════════════════════════

# ── Configuration ─────────────────────────────────────────────────────
# Each entry: "MODE SCENARIO WIND"
BATCH=(
    "STATIC STATIC none"
    "GIMBAL STATIC none"
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
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
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

# ── ffmpeg screen recording ───────────────────────────────────────────
RECORD=true
if [ "$1" == "--no-record" ]; then
    RECORD=false
fi

FFMPEG_PID=""
if $RECORD; then
    RECORDING_FILE="${RESULTS_DIR}/batch_recording_${TIMESTAMP}.mp4"
    echo -e "${BLUE}>>> Starting screen recording → ${RECORDING_FILE}${NC}"
    ffmpeg -video_size 1920x1080 -framerate 10 -f x11grab -i :1 \
           -c:v libx264 -preset ultrafast -crf 30 \
           -y "$RECORDING_FILE" > /dev/null 2>&1 &
    FFMPEG_PID=$!
    sleep 1
fi

# ── Batch loop ────────────────────────────────────────────────────────
RUN_NUM=0
TOTAL=${#BATCH[@]}

for ENTRY in "${BATCH[@]}"; do
    RUN_NUM=$((RUN_NUM + 1))
    read -r MODE SCENARIO WIND <<< "$ENTRY"

    echo ""
    echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}${BOLD}   RUN ${RUN_NUM}/${TOTAL}: ${MODE} | ${SCENARIO} | ${WIND}${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"

    # Snapshot existing CSV files before this run
    BEFORE_CSVS=$(ls -1 "$RESULTS_DIR"/*.csv 2>/dev/null | sort)

    # Launch the comparison sim
    bash "${SCRIPT_DIR}/start_comparison_sim.sh" "$MODE" "$SCENARIO" "$WIND" &
    SIM_MASTER_PID=$!

    # Wait for mission to finish (new CSV appears) or timeout
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
            sleep 5  # Grace period for clean shutdown
            break
        fi

        # Also check if mission node exited
        if ! pgrep -f "mission_comparison" > /dev/null 2>&1; then
            echo -e "${YELLOW}>>> Mission node exited${NC}"
            sleep 2
            # Re-check for CSV
            AFTER_CSVS=$(ls -1 "$RESULTS_DIR"/*.csv 2>/dev/null | sort)
            NEW_CSV=$(comm -13 <(echo "$BEFORE_CSVS") <(echo "$AFTER_CSVS") | grep -v "master_summary" | head -1)
            break
        fi
    done

    if [ $ELAPSED -ge $RUN_TIMEOUT ]; then
        echo -e "${RED}>>> TIMEOUT after ${RUN_TIMEOUT}s — force killing${NC}"
    fi

    # Kill all sim processes
    echo -e "${YELLOW}>>> Cleaning up run ${RUN_NUM}...${NC}"
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
    kill $SIM_MASTER_PID 2>/dev/null
    wait $SIM_MASTER_PID 2>/dev/null
    sleep 3

    # Extract KPIs from CSV and append to summary
    if [ -n "$NEW_CSV" ] && [ -f "$NEW_CSV" ]; then
        # CSV format: mode,scenario,wind_scenario,linear_error_m,rotation_error_deg,engagement_duration_s,...
        DATA_LINE=$(tail -1 "$NEW_CSV")
        LINEAR_ERR=$(echo "$DATA_LINE" | cut -d',' -f4)
        ROT_ERR=$(echo "$DATA_LINE" | cut -d',' -f5)
        ENGAGE=$(echo "$DATA_LINE" | cut -d',' -f6)
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

# ── Stop ffmpeg ───────────────────────────────────────────────────────
if [ -n "$FFMPEG_PID" ]; then
    echo -e "${BLUE}>>> Stopping screen recording...${NC}"
    kill -INT $FFMPEG_PID 2>/dev/null
    wait $FFMPEG_PID 2>/dev/null
    echo -e "${GREEN}>>> Recording saved → ${RECORDING_FILE}${NC}"
fi

# ── Final summary ────────────────────────────────────────────────────
echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}${BOLD}   BATCH COMPLETE — ${TOTAL} runs finished${NC}"
echo -e "${CYAN}   Summary → ${GREEN}${SUMMARY_FILE}${NC}"
if $RECORD; then
    echo -e "${CYAN}   Video   → ${GREEN}${RECORDING_FILE}${NC}"
fi
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${YELLOW}Results:${NC}"
column -t -s',' "$SUMMARY_FILE"
