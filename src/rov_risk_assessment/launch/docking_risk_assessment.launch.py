# #!/usr/bin/env python3
# """
# Docking Risk Assessment Launch File

# Launches only the nodes needed for docking-specialized Bayesian Network.

# Nodes Launched:
#     1. Environmental data (current only - no wind/waves)
#     2. Battery level monitor
#     3. Speed monitor (DVL-based)
#     4. Altitude monitor
#     5. Camera quality monitor
#     6. ArUco detection (your existing node)
#     7. Fish detection (for clearance)
#     8. Docking Bayesian Network (specialized 24-node network)
#     9. Operator state publisher (optional - time-based fatigue)

# Nodes NOT Launched (removed from docking BN):
#     - Wind/Waves monitoring
#     - Turbidity
#     - Trust model
#     - Full mission control BN

# For Paper Validation:
#     This launch configuration demonstrates the docking-specialized framework
#     with explicit traceability to CoTA tasks (A1-A9, H1-H6).
# """

# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
        
#         # ============================================
#         # ENVIRONMENTAL DATA (Current only)
#         # Maps to: Environmental root node
#         # ============================================
#         Node(
#             package='rov_risk_assessment',
#             executable='weather_api',
#             name='environmental_data',
#             output='screen',
#             parameters=[
#                 {'publish_wind': False},      # Not in docking BN
#                 {'publish_waves': False},     # Not in docking BN
#                 {'publish_current': True},    # Keep for docking
#             ]
#         ),
        
#         # ============================================
#         # VEHICLE STATE SENSORS
#         # Maps to: BatteryLevel, Speed, Altitude
#         # ============================================
#         Node(
#             package='rov_risk_assessment',
#             executable='battery_level',
#             name='battery_monitor',
#             output='screen'
#         ),
        
#         Node(
#             package='rov_risk_assessment',
#             executable='speed',
#             name='speed_monitor',
#             output='screen'
#         ),
        
#         Node(
#             package='rov_risk_assessment',
#             executable='altitude',
#             name='altitude_monitor',
#             output='screen'
#         ),
        
#         Node(
#             package='rov_risk_assessment',
#             executable='camera_quality',
#             name='camera_monitor',
#             output='screen'
#         ),
        
#         # ============================================
#         # DOCKING-SPECIFIC: ArUco Detection
#         # Maps to: ArUcoMarkersVisible, DockingStationDetection
#         # CoTA Tasks: A3.1, A3.2 (D2)
#         # ============================================
#         Node(
#             package='mundus_mir_docking_controller',  # Your ArUco package
#             executable='pose_estimation_aruco',       # Your ArUco node
#             name='aruco_detector',
#             output='screen',
#             remappings=[
#                 ('/blueye/camera_1/image_raw', '/blueye/camera_1/image_raw'),
#                 ('/blueye/aruco_visibility', '/blueye/aruco_visibility'),
#                 ('/blueye/docking_station_detected', '/blueye/docking_station_detected'),
#             ]
#         ),
        
#         # ============================================
#         # DOCKING-SPECIFIC: Fish Detection (Clearance)
#         # Maps to: DockingClearance
#         # CoTA Task: A5.1 (Assess station clearance)
#         # ============================================
#         Node(
#             package='rov_risk_assessment',
#             executable='fish_detection',
#             name='fish_detector',
#             output='screen',
#             parameters=[
#                 {'display_window': False},         # Disable GUI for headless operation
#                 {'confidence_threshold': 0.55},
#                 {'publish_annotated': False},      # Save bandwidth
#             ]
#         ),
        
#         # ============================================
#         # HUMAN OPERATOR STATE (Optional)
#         # Maps to: Fatigue, Stress
#         # CoTA: H1 (Monitor Pre-Docking - continuous)
#         # ============================================
#         Node(
#             package='rov_risk_assessment',
#             executable='operator_state',
#             name='operator_state_publisher',
#             output='screen',
#             parameters=[
#                 {'fatigue_mode': 'time_based'},   # Options: 'time_based', 'manual', 'fixed'
#                 {'stress_mode': 'fixed'},         # Options: 'time_based', 'manual', 'fixed'
#                 {'initial_fatigue': 0.2},         # Low fatigue at start
#                 {'initial_stress': 0.3},          # Low-medium stress for docking
#                 {'fatigue_rate': 0.1},            # Increases 0.1 per 10 minutes
#             ]
#         ),
        
#         # ============================================
#         # DOCKING BAYESIAN NETWORK (Main Node)
#         # Implements the 24-node docking-specialized BN
#         # Outputs: DockingModeRecommendation
#         # ============================================
#         Node(
#             package='rov_risk_assessment',
#             executable='docking_bn',
#             name='docking_risk_assessment',
#             output='screen'
#         ),
#     ])


#!/usr/bin/env python3
"""
Docking Risk Assessment Launch File (Without ArUco/Fish)

Prerequisites (run these BEFORE launching):
    1. Start Gazebo simulation manually
    2. Terminal 1: ros2 run mundus_mir_docking_controller pose_estimation_aruco_node
    3. Terminal 2: ros2 run fish_detection fish_detection

Then launch this file:
    ros2 launch rov_risk_assessment docking_risk_assessment.launch.py

This will start:
    - Environmental data publisher (Current)
    - Battery/Speed/Altitude/Camera monitors
    - Operator state publisher (Fatigue/Stress)
    - Docking Bayesian Network (subscribes to ArUco and Fish topics)
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # ============================================
        # ENVIRONMENTAL DATA (Current only)
        # ============================================
        Node(
            package='rov_risk_assessment',
            executable='weather_api',
            name='environmental_data',
            output='screen'
        ),
        
        # ============================================
        # VEHICLE STATE SENSORS
        # ============================================
        Node(
            package='rov_risk_assessment',
            executable='battery_level',
            name='battery_monitor',
            output='screen'
        ),
        
        Node(
            package='rov_risk_assessment',
            executable='speed',
            name='speed_monitor',
            output='screen'
        ),
        
        Node(
            package='rov_risk_assessment',
            executable='altitude',
            name='altitude_monitor',
            output='screen'
        ),
        
        Node(
            package='rov_risk_assessment',
            executable='camera_quality',
            name='camera_monitor',
            output='screen'
        ),
        
        # ============================================
        # HUMAN OPERATOR STATE
        # ============================================
        Node(
            package='rov_risk_assessment',
            executable='operator_state',
            name='operator_state_publisher',
            output='screen',
            parameters=[
                {'fatigue_mode': 'time_based'},
                {'stress_mode': 'fixed'},
                {'initial_fatigue': 0.2},
                {'initial_stress': 0.3},
                {'fatigue_rate': 0.1},
            ]
        ),
        
        # ============================================
        # DOCKING BAYESIAN NETWORK
        # Subscribes to:
        #   - /blueye/aruco_visibility (from pose_estimation_aruco_node)
        #   - /blueye/docking_station_detected (from pose_estimation_aruco_node)
        #   - /fish_detection/count (from fish_detection package)
        # ============================================
        Node(
            package='rov_risk_assessment',
            executable='docking_bn',
            name='docking_risk_assessment',
            output='screen'
        ),
    ])
