#!/bin/bash
# Docking System Startup Script
# 
# Prerequisites: Gazebo simulation must be running
#
# This script opens 3 terminals:
#   Terminal 1: ArUco detection (mundus_mir_docking_controller)
#   Terminal 2: Fish detection (YOLO-based clearance)
#   Terminal 3: Docking BN + sensors (rov_risk_assessment)

echo "=========================================="
echo "🚀 Docking Risk Assessment System Launcher"
echo "=========================================="
echo ""
echo "⚠️  PREREQUISITE: Make sure Gazebo is running!"
echo ""
read -p "Press Enter when Gazebo is ready, or Ctrl+C to cancel..."
echo ""

# Check if required workspaces exist
if [ ! -d "$HOME/Desktop/mundus_mir_simulator/install" ]; then
    echo "❌ Error: mundus_mir_simulator workspace not built"
    echo "   Please build it first: cd ~/Desktop/mundus_mir_simulator && colcon build"
    exit 1
fi

if [ ! -d "$HOME/Desktop/PhD_Autumn_2025/subsea-autonomy-stack/install" ]; then
    echo "❌ Error: subsea-autonomy-stack workspace not built"
    echo "   Please build it first: cd ~/Desktop/PhD_Autumn_2025/subsea-autonomy-stack && colcon build"
    exit 1
fi

if [ ! -f "$HOME/Desktop/PhD_Autumn_2025/subsea-autonomy-stack/models/fish_detection/weights/merge_yolov4.weights" ]; then
    echo "❌ Error: fish detection YOLOv4 weights not found"
    exit 1
fi

if [ ! -f "$HOME/Desktop/PhD_Autumn_2025/subsea-autonomy-stack/models/fish_detection/configs/yolov4.cfg" ]; then
    echo "❌ Error: fish detection YOLOv4 config not found"
    exit 1
fi

echo "✅ All prerequisites found"
echo ""
echo "Launching system in 3 seconds..."
sleep 3

# ============================================
# Terminal 1: ArUco Detection
# ============================================
gnome-terminal --tab --title="ArUco Detection" -- bash -c "
echo '================================================'
echo '📷 ARUCO DETECTION NODE'
echo '================================================'
cd ~/Desktop/mundus_mir_simulator
source install/setup.bash
echo '✓ Workspace sourced'
echo '✓ Starting ArUco pose estimation...'
echo ''
ros2 run mundus_mir_docking_controller pose_estimation_aruco_node
exec bash"

echo "✓ Terminal 1: ArUco detection launched"
sleep 2

# ============================================
# Terminal 2: Fish Detection
# ============================================
gnome-terminal --tab --title="Fish Detection" -- bash -c "
echo '================================================'
echo '🐟 FISH DETECTION NODE (Clearance Assessment)'
echo '================================================'
cd ~/Desktop/PhD_Autumn_2025/subsea-autonomy-stack
source install/setup.bash
echo '✓ ROS2 environment sourced'
echo '✓ Starting YOLO fish detector...'
echo ''
ros2 run fish_detection fish_detection --ros-args \\
    -p weights_path:=models/fish_detection/weights/merge_yolov4.weights \\
    -p config_path:=models/fish_detection/configs/yolov4.cfg \\
    -p confidence_threshold:=0.55 \\
    -p display_window:=false
exec bash"

echo "✓ Terminal 2: Fish detection launched"
sleep 2

# ============================================
# Terminal 3: Docking BN System
# ============================================
gnome-terminal --tab --title="Docking BN System" -- bash -c "
echo '================================================'
echo '🧠 DOCKING BAYESIAN NETWORK'
echo '================================================'
cd ~/Desktop/PhD_Autumn_2025/subsea-autonomy-stack
source ~/Desktop/mundus_mir_simulator/install/setup.bash
source install/setup.bash
echo '✓ Workspaces sourced'
echo '✓ Waiting for ArUco and Fish detection...'
sleep 3
echo '✓ Starting docking risk assessment system...'
echo ''
ros2 launch rov_risk_assessment docking_risk_assessment.launch.py
exec bash"

echo "✓ Terminal 3: Docking BN system launched"
echo ""
echo "=========================================="
echo "✅ ALL SYSTEMS LAUNCHED!"
echo "=========================================="
echo ""
echo "Monitor system status:"
echo "  ros2 topic list | grep blueye"
echo "  ros2 topic echo /docking/mode_recommendation"
echo ""
echo "To stop all nodes:"
echo "  pkill -f 'ros2|python3 fish'"
echo ""
