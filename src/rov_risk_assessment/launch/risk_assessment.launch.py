#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rov_risk_assessment',
            executable='weather_api',
            name='environmental_data',
            output='screen'
        ),
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
        Node(
            package='rov_risk_assessment',
            executable='turbidity',
            name='turbidity_monitor',
            output='screen'
        ),
        Node(
            package='rov_risk_assessment',
            executable='trust',
            name='trust_model',
            output='screen'
        ),
        Node(
            package='rov_risk_assessment',
            executable='mission_control_bn',
            name='risk_assessment_bn',
            output='screen'
        ),
    ])