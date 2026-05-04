# Docking-Specialized Bayesian Network - ROS2 Integration Guide

## 📋 Overview

This package implements the **docking-specialized Bayesian Network** from your PhD research paper. It demonstrates the systematic mapping from CoTA tasks to probabilistic risk assessment.

### Key Framework Contribution
- **24-node docking-specialized BN** (reduced from 32-node full mission network)
- **100% traceability** from CoTA tasks to BN nodes
- **Real sensor integration**: ArUco detection, fish-based clearance, USBL positioning
- **Human-autonomy collaboration**: Models operator cognitive state and system reliability

---

## 🗂️ File Structure

```
rov_risk_assessment/
├── rov_risk_assessment/
│   ├── mission_control_bn_node.py          # Full mission (existing)
│   ├── docking_bn_node.py                  # Docking specialized (NEW)
│   ├── operator_state_publisher.py         # Human state (NEW)
│   ├── altitude.py                         # Reused
│   ├── battey_level.py                     # Reused
│   ├── camera_quality.py                   # Reused
│   ├── speed.py                            # Reused
│   ├── weather_api.py                      # Reused (Current only)
│   └── fish_detection.py                   # Your existing YOLO node
│
├── launch/
│   ├── risk_assessment.launch.py           # Full mission (existing)
│   └── docking_risk_assessment.launch.py   # Docking only (NEW)
│
└── config/
    ├── integrated_mission_control.xdsl     # Full mission BN (existing)
    └── docking_specialized_bn.xdsl         # Docking BN (NEW - you have this)
```

---

## 🔧 Installation Steps

### 1. Add New Files to Your Package

```bash
cd ~/Desktop/PhD_Autumn_2025/subsea-autonomy-stack/rov_risk_assessment

# Add new Python nodes
cp docking_bn_node.py rov_risk_assessment/
cp operator_state_publisher.py rov_risk_assessment/

# Add new launch file
cp docking_risk_assessment.launch.py launch/

# Ensure docking BN is in config
cp docking_specialized_bn.xdsl config/
```

### 2. Update setup.py

Add the new executables to your `setup.py`:

```python
entry_points={
    'console_scripts': [
        # Existing executables
        'mission_control_bn = rov_risk_assessment.mission_control_bn_node:main',
        'weather_api = rov_risk_assessment.weather_api:main',
        'battery_level = rov_risk_assessment.battey_level:main',
        'speed = rov_risk_assessment.speed:main',
        'altitude = rov_risk_assessment.altitude:main',
        'camera_quality = rov_risk_assessment.camera_quality:main',
        'turbidity = rov_risk_assessment.turbidity:main',
        'trust = rov_risk_assessment.trust:main',
        
        # NEW: Docking-specialized nodes
        'docking_bn = rov_risk_assessment.docking_bn_node:main',
        'operator_state = rov_risk_assessment.operator_state_publisher:main',
        'fish_detection = rov_risk_assessment.fish_detection:main',  # If not already added
    ],
},
```

### 3. Update data_files in setup.py

Ensure launch files and config are installed:

```python
data_files=[
    # ... existing entries ...
    (os.path.join('share', package_name, 'launch'), 
     glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), 
     glob('config/*.xdsl')),
],
```

### 4. Build the Package

```bash
cd ~/Desktop/PhD_Autumn_2025/subsea-autonomy-stack
colcon build --packages-select rov_risk_assessment
source install/setup.bash
```

---

## 🚀 Running the Docking System

### Launch Docking Risk Assessment

```bash
ros2 launch rov_risk_assessment docking_risk_assessment.launch.py
```

### What Gets Launched:

1. **Environmental Monitor** (Current only)
2. **Vehicle Sensors** (Battery, Speed, Altitude, Camera)
3. **ArUco Detector** (from `mundus_mir_docking_controller` package)
4. **Fish Detector** (YOLO-based clearance assessment)
5. **Operator State Publisher** (Fatigue/Stress)
6. **Docking Bayesian Network** (Main risk assessment)

---

## 📊 Topic Mapping: CoTA Tasks → ROS2 Topics → BN Nodes

### Evidence Inputs (Observable Nodes)

| **CoTA Task** | **ROS2 Topic** | **BN Node** | **Source** |
|---------------|----------------|-------------|------------|
| Environmental | `/blueye/current` | Current | weather_api |
| A-P1.1 | `/blueye/battery_level` | BatteryLevel | battery_level |
| A-P1.1 | `/blueye/altitude` | Altitude | altitude |
| A1.1 | `/blueye/usbl_strength` | USBLStrength | weather_api |
| A1.3 | `/blueye/speed` | Speed | speed |
| A2.2 | `/blueye/camera_quality` | CameraQuality | camera_quality |
| **A3.1** | `/blueye/aruco_visibility` | **ArUcoMarkersVisible** | **pose_estimation_aruco** |
| **A3.2 (D2)** | `/blueye/docking_station_detected` | **DockingStationDetection** | **pose_estimation_aruco** |
| A4.1, A4.2 | `/blueye/pose_estimated_board_stamped` | PoseEstimationQuality, PositionError | pose_estimation_aruco |
| **A5.1** | `/fish_detection/count` | **DockingClearance** | **fish_detection** |
| H1 | `/blueye/human/fatigue` | Fatigue | operator_state |
| H1 | `/blueye/human/stress` | Stress | operator_state |

### Outputs (Decision Nodes)

| **BN Node** | **ROS2 Topic** | **Description** |
|-------------|----------------|-----------------|
| DockingModeRecommendation | `/docking/mode_recommendation` | Autonomous/Human/Shared |
| DockingReliability | `/docking/reliability` | Success probability |
| VisualGuidanceQuality | `/docking/visual_guidance_quality` | Excellent/Good/Failed |
| ApproachFeasibility | `/docking/approach_feasibility` | Safe/Marginal/Unsafe |

---

## 🧪 Testing Scenarios

### Scenario 1: Nominal Docking (All Systems Good)

```bash
# Expected output: Autonomous mode recommended
# - ArUco: All markers visible
# - Clearance: No fish detected
# - Camera: Excellent quality
# - USBL: Strong signal
```

### Scenario 2: Degraded Vision

Simulate poor camera quality:
```bash
ros2 topic pub /blueye/camera_quality std_msgs/String "data: 'Poor'" --once
```

**Expected:** Mode shifts toward Human/Shared control

### Scenario 3: Obstructed Docking

Fish detection triggers clearance issue:
```bash
# Fish detector will publish count > 0
# Expected: ApproachFeasibility = Unsafe, recommend Human control
```

### Scenario 4: Operator Fatigue

Increase fatigue over time (automatic) or manually:
```bash
ros2 param set /operator_state_publisher fatigue_value 0.8
```

**Expected:** SituationalAwareness degrades, may shift toward Autonomous

---

## 📝 Monitoring the System

### View All Topics

```bash
ros2 topic list | grep blueye
```

### Monitor BN Output

```bash
# Watch mode recommendation
ros2 topic echo /docking/mode_recommendation

# Watch docking reliability
ros2 topic echo /docking/reliability

# Watch ArUco detection
ros2 topic echo /blueye/aruco_visibility

# Watch fish count
ros2 topic echo /fish_detection/count
```

### Check Node Status

```bash
ros2 node list
ros2 node info /docking_risk_assessment
```

---

## 🔍 Debugging

### Check if BN Loaded Correctly

The docking BN node should log:
```
✓ Loaded Docking BN: /path/to/docking_specialized_bn.xdsl
✓ Network contains 24 nodes (24 expected)
```

### Common Issues

**Issue:** ArUco node not publishing
```bash
# Check if mundus_mir_docking_controller is built
ros2 pkg list | grep mundus

# Verify camera topic
ros2 topic echo /blueye/camera_1/image_raw --once
```

**Issue:** Fish detection not working
```bash
# Check YOLO weights path
ros2 param get /fish_detector weights_path

# Verify camera feed
ros2 run rqt_image_view rqt_image_view
```

**Issue:** BN not updating
```bash
# Check evidence is being set
ros2 topic hz /blueye/current
ros2 topic hz /blueye/aruco_visibility
```

---

## 📖 Paper Validation: Traceability

### Demonstrating Framework Contribution

The docking system proves these key claims from your paper:

1. **Systematic Task-to-Node Mapping** (Table 1 in paper)
   - Every BN node maps to a CoTA task
   - ArUco detection (A3) → ArUcoMarkersVisible, DockingStationDetection
   - Fish detection (A5.1) → DockingClearance
   - Operator monitoring (H1) → Fatigue, Stress

2. **Task Dependency-to-Arc Mapping** (Table 2 in paper)
   - ArUco → PoseEstimation (A3 → A4 task dependency)
   - Camera → ArUco (A2 → A3 task dependency)
   - Fatigue → Attention (H1 cognitive process)

3. **100% Coverage** (Table 3 in paper)
   - All 24 nodes traced to tasks
   - All 30 arcs traced to task dependencies
   - All ESD decision points represented

### Running Validation Experiments

```bash
# Record a bag file during docking
ros2 bag record -a -o docking_validation

# Play back for analysis
ros2 bag play docking_validation

# Export BN state for paper figures
# (The docking_bn_node logs comprehensive state every 1 second)
```

---

## 🎯 Integration with Your ArUco Node

Your `pose_estimation_aruco` node already publishes everything needed:

✅ `/blueye/aruco_visibility` (String: "All"/"Some"/"None")
✅ `/blueye/docking_station_detected` (Bool)
✅ `/blueye/pose_estimated_board_stamped` (PoseWithCovarianceStamped)

**No modifications needed** to your ArUco node!

The docking BN subscribes to these topics and maps them to:
- `ArUcoMarkersVisible` (from aruco_visibility)
- `DockingStationDetection` (from docking_station_detected)
- `PoseEstimationQuality` (calculated from pose variance)

---

## 🐟 Integration with Your Fish Detection

Your `fish_detection` node publishes:

✅ `/fish_detection/count` (Int32)
✅ `/fish_detection/detections` (String - JSON)

The docking BN maps fish count to `DockingClearance`:
- **0 fish** → Clear (state 0)
- **≥1 fish** → Obstructed (state 1)

---

## 📊 Example Output

When running, you should see:

```
============================================================
DOCKING BAYESIAN NETWORK - RISK ASSESSMENT
============================================================

📡 DOCKING SENSORS:
  ArUco Markers: Some
  Station Detected: True
  Fish Count: 0
  Camera Quality: Good
  USBL Strength: Strong

🎯 DOCKING ASSESSMENT:
  Visual Guidance: Good
  Approach Feasibility: Safe
  Docking Reliability: 0.825

🤖 SYSTEM STATE:
  Autonomous Reliability: 0.870
  Operator Situational Awareness: 0.750

✨ DOCKING MODE RECOMMENDATION:
  Autonomous: 65.40%  ███████████████████
  Human:      15.20%  ████
  Shared:     19.40%  █████

  ➜ RECOMMENDED: AUTONOMOUS (Confidence: 65.4%)
============================================================
```

---

## 🎓 For Your PhD Paper

### Figures to Generate

1. **System Architecture Diagram**
   - Show ROS2 nodes → BN evidence flow
   - Highlight traceability: CoTA → Topics → BN Nodes

2. **Real-time Risk Assessment Screenshot**
   - Terminal showing BN state output
   - Demonstrate mode recommendation changes

3. **Scenario Comparison Table**
   - Nominal vs. Degraded conditions
   - Mode recommendation evolution

### Experimental Protocol

1. **Baseline:** Run nominal docking (all sensors good)
2. **Scenario 1:** Degrade camera quality mid-mission
3. **Scenario 2:** Introduce fish obstruction
4. **Scenario 3:** Increase operator fatigue over time
5. **Record:** Mode recommendations and confidence levels

---

## 📞 Contact & Support

For questions about this integration:
- PhD Candidate: Badawi
- Institution: NTNU Marine Technology
- Project: BREACH - Human-Autonomy Collaboration

**Framework Paper:** "Risk Modeling for Human-System Interaction of Autonomous Systems: A Systematic Framework Integrating ESDs, CoTA, and Dynamic Bayesian Networks"
