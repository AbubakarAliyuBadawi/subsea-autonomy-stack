#!/bin/bash
# Blueye Real-Hardware Docking Risk Assessment Launcher
#
# Terminals opened:
#   1. ArUco detection  (pose_estimation_aruco_node)
#   2. Fish detection   (YOLO clearance assessment)
#   3. Docking BN + sensors (blueye_docking_risk_assessment.launch.py)
#
# Prerequisite: blueye_telemetry.py must already be running
#   (started via: ros2 launch blueye_bt_real blueye_bt_real.launch.py
#    OR:          ros2 run blueye_visualization_real blueye_telemetry.py)

echo "=================================================="
echo " Blueye Docking Risk Assessment System"
echo "=================================================="
echo ""

# --- Workspace checks ---
if [ ! -d "$HOME/Desktop/mundus_mir_simulator/install" ]; then
    echo "ERROR: mundus_mir_simulator workspace not built."
    echo "  cd ~/Desktop/mundus_mir_simulator && colcon build"
    exit 1
fi

if [ ! -d "$HOME/Desktop/PhD_Autumn_2025/subsea-autonomy-stack/install" ]; then
    echo "ERROR: subsea-autonomy-stack workspace not built."
    echo "  cd ~/Desktop/PhD_Autumn_2025/subsea-autonomy-stack && colcon build"
    exit 1
fi

if [ ! -f "$HOME/Desktop/PhD_Autumn_2025/subsea-autonomy-stack/models/fish_detection/weights/merge_yolov4.weights" ]; then
    echo "ERROR: fish detection YOLOv4 weights not found."
    exit 1
fi

if [ ! -f "$HOME/Desktop/PhD_Autumn_2025/subsea-autonomy-stack/models/fish_detection/configs/yolov4.cfg" ]; then
    echo "ERROR: fish detection YOLOv4 config not found."
    exit 1
fi

echo "All prerequisites found."
echo ""
echo "NOTE: Make sure blueye_telemetry.py is already running!"
echo "      (provides /blueye/battery, /blueye/altitude, /blueye/speed, /blueye/gps)"
echo ""
read -p "Press Enter to launch, or Ctrl+C to cancel..."
echo ""

# ============================================================
# Terminal 1: ArUco Detection
# ============================================================
gnome-terminal --tab --title="ArUco Detection" -- bash -c "
echo '================================================'
echo ' ArUco Detection Node'
echo '================================================'
cd ~/Desktop/mundus_mir_simulator
source install/setup.bash
echo 'Starting ArUco pose estimation...'
ros2 run mundus_mir_docking_controller pose_estimation_aruco_node
exec bash"

echo "Terminal 1: ArUco detection launched"
sleep 2

# ============================================================
# Terminal 2: Fish Detection
# ============================================================
gnome-terminal --tab --title="Fish Detection" -- bash -c "
echo '================================================'
echo ' Fish Detection Node (Clearance Assessment)'
echo '================================================'
cd ~/Desktop/PhD_Autumn_2025/subsea-autonomy-stack
source install/setup.bash
echo 'Starting YOLO fish detector...'
ros2 run fish_detection fish_detection --ros-args \
    -p weights_path:=models/fish_detection/weights/merge_yolov4.weights \
    -p config_path:=models/fish_detection/configs/yolov4.cfg \
    -p confidence_threshold:=0.55 \
    -p display_window:=false
exec bash"

echo "Terminal 2: Fish detection launched"
sleep 2

# ============================================================
# Terminal 3: Docking BN System
# ============================================================
gnome-terminal --tab --title="Docking Risk Assessment" -- bash -c "
echo '================================================'
echo ' Docking Bayesian Network'
echo '================================================'
cd ~/Desktop/PhD_Autumn_2025/subsea-autonomy-stack
source install/setup.bash
echo 'Waiting for ArUco and fish detection to initialise...'
sleep 3
echo 'Starting docking risk assessment...'
ros2 launch blueye_risk_assessment blueye_docking_risk_assessment.launch.py
exec bash"

echo "Terminal 3: Docking BN launched"
echo ""
echo "=================================================="
echo " All systems launched!"
echo "=================================================="
echo ""
echo "Monitor outputs:"
echo "  ros2 topic echo /docking/mode_recommendation"
echo "  ros2 topic echo /docking/reliability"
echo "  ros2 topic echo /docking/approach_feasibility"
echo ""
echo "Stop everything:"
echo "  pkill -f 'ros2|python3 fish'"
