#!/usr/bin/env python3
"""
Blueye Real-Hardware Docking Risk Assessment Launch File

Prerequisites (start these before launching):
    1. ros2 run blueye_visualization_real blueye_telemetry.py
           — provides /blueye/battery, /blueye/altitude, /blueye/speed, /blueye/gps
    2. ros2 run mundus_mir_docking_controller pose_estimation_aruco_node
           — provides /blueye/aruco_visibility, /blueye/docking_station_detected,
                      /blueye/pose_estimated_board_stamped
    3. ros2 run fish_detection fish_detection
           — provides /fish_detection/count

Nodes launched here:
    - weather_api       : Stormglass environmental data + USBL strength from GPS
    - battery_level     : /blueye/battery (Pose) → /blueye/battery_level (String)
    - camera_quality    : /blueye/camera_1/image_raw → /blueye/camera_quality
    - operator_state    : time-based fatigue/stress publisher
    - docking_bn        : 24-node Bayesian Network → docking mode recommendation

Nodes NOT launched (provided by blueye_telemetry.py):
    - speed   : /blueye/speed published directly by telemetry node
    - altitude: /blueye/altitude published directly by telemetry node
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ----------------------------------------------------------------
        # Environmental data (Stormglass API + USBL strength from GPS)
        # ----------------------------------------------------------------
        Node(
            package='blueye_risk_assessment',
            executable='weather_api',
            name='environmental_data',
            output='screen'
        ),

        # ----------------------------------------------------------------
        # Battery level: reads /blueye/battery (Pose), publishes String
        # ----------------------------------------------------------------
        Node(
            package='blueye_risk_assessment',
            executable='battery_level',
            name='battery_monitor',
            output='screen'
        ),

        # ----------------------------------------------------------------
        # Camera quality: reads /blueye/camera_1/image_raw
        # ----------------------------------------------------------------
        Node(
            package='blueye_risk_assessment',
            executable='camera_quality',
            name='camera_monitor',
            output='screen'
        ),

        # ----------------------------------------------------------------
        # Human operator state (time-based fatigue, fixed stress)
        # ----------------------------------------------------------------
        Node(
            package='blueye_risk_assessment',
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

        # ----------------------------------------------------------------
        # Docking Bayesian Network (24-node, docking-specialised)
        # Subscribes to all preprocessed topics above plus:
        #   /blueye/speed, /blueye/altitude  (from blueye_telemetry.py)
        #   /blueye/aruco_visibility, /blueye/docking_station_detected,
        #   /blueye/pose_estimated_board_stamped  (from aruco node)
        #   /fish_detection/count  (from fish detection node)
        # ----------------------------------------------------------------
        Node(
            package='blueye_risk_assessment',
            executable='docking_bn',
            name='docking_risk_assessment',
            output='screen'
        ),
    ])
