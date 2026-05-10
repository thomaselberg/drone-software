#!/bin/bash
# Base stack for real flight: autopilot + RealSense + ArUco detector + KPI logger.
# The mission is started separately in another terminal:
#   ros2 run thyra vision_landing_mission.py --ros-args -p mode:=GIMBAL -p scenario:=DYNAMIC ...
#
# Usage:  ./start_base.sh [STATIC|DYNAMIC]   (default: DYNAMIC — only affects KPI logger label)

SCENARIO="${1:-DYNAMIC}"
SCENARIO=$(echo "$SCENARIO" | tr '[:lower:]' '[:upper:]')

pkill -9 -f "vision_landing_mission|kpi_logger_real|kpi_logger_sim|aruco_detector_comparison|thyra.launch|MicroXRCEAgent" 2>/dev/null
sleep 2
rm -f /tmp/real_flight.log

source /opt/ros/jazzy/setup.bash
cd ~/drone-software
source install/setup.bash

ros2 launch thyra thyra.launch.py 2>&1 | tee /tmp/real_flight.log &

while ! grep -q "THYRA OPERATIONAL" /tmp/real_flight.log 2>/dev/null; do sleep 1; done
echo ">>> Autopilot ready"

ros2 run thyra aruco_detector_comparison --ros-args -p show_window:=false &
ros2 run thyra kpi_logger_real.py --ros-args -p scenario:="${SCENARIO}" &

echo ""
echo "==============================================================="
echo "  Base stack up. Open GUI + rqt on the SSH PC."
echo "  When ready, in another terminal run the mission:"
echo "    cd ~/drone-software && source install/setup.bash"
echo "    ros2 run thyra vision_landing_mission.py --ros-args \\"
echo "      -p mode:=GIMBAL -p scenario:=${SCENARIO} -p wind_scenario:=none \\"
echo "      -p takeoff_alt:=1.5 -p target_start_x:=1.0 -p target_start_y:=0.0"
echo ""
echo "  Abort: click Land in GUI, wait for touchdown, then Ctrl+C mission."
echo "  Ctrl+C here when fully done."
echo "==============================================================="

trap "echo 'Shutting down...'; pkill -9 -f vision_landing_mission 2>/dev/null; pkill -9 -f kpi_logger_real 2>/dev/null; pkill -9 -f aruco_detector_comparison 2>/dev/null; pkill -9 -f thyra.launch 2>/dev/null; pkill -9 -f MicroXRCEAgent 2>/dev/null; exit" INT TERM

wait
