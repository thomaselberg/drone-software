#!/bin/bash
# Base stack for real flight: autopilot + RealSense + ArUco detector + KPI logger.
# The mission is started separately in another terminal (template printed below).
#
# Usage:  ./start_base.sh [static|dynamic] [target_x_m]
#           static    — fixed marker at target_x_m north (default 5 m)
#           dynamic   — moving marker (default; target_x ignored)

SCENARIO="${1:-dynamic}"
SCENARIO_UC=$(echo "$SCENARIO" | tr '[:lower:]' '[:upper:]')
TARGET_X="${2:-5.0}"

case "$SCENARIO_UC" in
  STATIC|DYNAMIC) ;;
  *) echo "Usage: $0 [static|dynamic] [target_x_m]"; exit 1 ;;
esac

pkill -9 -f "vision_landing_mission|kpi_logger_real|kpi_logger_sim|aruco_detector|thyra.launch|MicroXRCEAgent" 2>/dev/null
sleep 2
rm -f /tmp/real_flight.log

source /opt/ros/jazzy/setup.bash
cd ~/drone-software
source install/setup.bash

ros2 launch thyra thyra.launch.py 2>&1 | tee /tmp/real_flight.log &

while ! grep -q "THYRA OPERATIONAL" /tmp/real_flight.log 2>/dev/null; do sleep 1; done
echo ">>> Autopilot ready"

ros2 run thyra aruco_detector --ros-args -p show_window:=false &
ros2 run thyra kpi_logger_real.py --ros-args -p scenario:="${SCENARIO_UC}" &

echo ""
echo "==============================================================="
echo "  Base stack up. Open GUI + rqt on the SSH PC."
echo "  Scenario: ${SCENARIO_UC}   Target X: ${TARGET_X} m (STATIC only)"
echo ""
echo "  When ready, in another terminal run the mission:"
echo "    cd ~/drone-software && source install/setup.bash"
if [ "$SCENARIO_UC" = "STATIC" ]; then
    echo "    ros2 run thyra vision_landing_mission.py --ros-args \\"
    echo "      -p mode:=GIMBAL \\"
    echo "      -p scenario:=STATIC \\"
    echo "      -p target_start_x:=${TARGET_X} \\"
    echo "      -p target_start_y:=0.0 \\"
    echo "      -p search_kp:=0.1 \\"
    echo "      -p takeoff_alt:=2.0 \\"
    echo "      -p descend_alt:=1.5 \\"
    echo "      -p terminal_alt_trigger:=1.0"
else
    echo "    ros2 run thyra vision_landing_mission.py --ros-args \\"
    echo "      -p mode:=GIMBAL \\"
    echo "      -p scenario:=DYNAMIC \\"
    echo "      -p search_kp:=0.1 \\"
    echo "      -p takeoff_alt:=2.0 \\"
    echo "      -p descend_alt:=1.5 \\"
    echo "      -p terminal_alt_trigger:=1.0"
fi
echo ""
echo "  Abort: click Land in GUI, wait for touchdown, then Ctrl+C mission."
echo "  Ctrl+C here when fully done."
echo "==============================================================="

trap "echo 'Shutting down...'; pkill -9 -f vision_landing_mission 2>/dev/null; pkill -9 -f kpi_logger_real 2>/dev/null; pkill -9 -f aruco_detector 2>/dev/null; pkill -9 -f thyra.launch 2>/dev/null; pkill -9 -f MicroXRCEAgent 2>/dev/null; exit" INT TERM

wait
