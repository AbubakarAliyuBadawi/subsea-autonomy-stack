#!/usr/bin/env python3
"""
Operator State Publisher

Publishes human operator cognitive state (Fatigue, Stress) for docking BN.

Modes:
    - time_based: Fatigue increases over mission duration
    - manual: Publish values from ROS2 parameters (can update via CLI)
    - fixed: Constant values for testing

Maps to CoTA Tasks:
    - H1: Monitor Pre-Docking Approach (continuous monitoring causes fatigue)
    - Fatigue/Stress are root nodes in the human cognitive chain

Usage Examples:
    # Time-based fatigue (increases over mission)
    ros2 run rov_risk_assessment operator_state --ros-args -p fatigue_mode:=time_based
    
    # Manual control
    ros2 run rov_risk_assessment operator_state --ros-args -p fatigue_mode:=manual -p fatigue_value:=0.7
    
    # Fixed for testing
    ros2 run rov_risk_assessment operator_state --ros-args -p fatigue_mode:=fixed -p fatigue_value:=0.2
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from datetime import datetime


class OperatorStatePublisher(Node):
    def __init__(self):
        super().__init__('operator_state_publisher')
        
        # Declare parameters
        self.declare_parameter('fatigue_mode', 'time_based')  # time_based, manual, fixed
        self.declare_parameter('stress_mode', 'fixed')        # time_based, manual, fixed
        self.declare_parameter('initial_fatigue', 0.2)        # Starting fatigue (0-1)
        self.declare_parameter('initial_stress', 0.3)         # Starting stress (0-1)
        self.declare_parameter('fatigue_rate', 0.1)           # Increase per 10 minutes
        self.declare_parameter('stress_rate', 0.05)           # Increase per 10 minutes
        self.declare_parameter('fatigue_value', 0.2)          # Manual/fixed value
        self.declare_parameter('stress_value', 0.3)           # Manual/fixed value
        self.declare_parameter('update_rate', 1.0)            # Hz
        
        # Get parameters
        self.fatigue_mode = self.get_parameter('fatigue_mode').value
        self.stress_mode = self.get_parameter('stress_mode').value
        self.initial_fatigue = self.get_parameter('initial_fatigue').value
        self.initial_stress = self.get_parameter('initial_stress').value
        self.fatigue_rate = self.get_parameter('fatigue_rate').value
        self.stress_rate = self.get_parameter('stress_rate').value
        self.fatigue_value = self.get_parameter('fatigue_value').value
        self.stress_value = self.get_parameter('stress_value').value
        update_rate = self.get_parameter('update_rate').value
        
        # State
        self.mission_start_time = self.get_clock().now()
        self.current_fatigue = self.initial_fatigue
        self.current_stress = self.initial_stress
        
        # Publishers
        self.fatigue_pub = self.create_publisher(Float32, '/blueye/human/fatigue', 10)
        self.stress_pub = self.create_publisher(Float32, '/blueye/human/stress', 10)
        
        # Timer
        self.create_timer(1.0 / update_rate, self.publish_state)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('✓ Operator State Publisher started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Fatigue mode: {self.fatigue_mode}')
        self.get_logger().info(f'  Stress mode: {self.stress_mode}')
        if self.fatigue_mode == 'time_based':
            self.get_logger().info(f'  Fatigue: starts at {self.initial_fatigue:.2f}, +{self.fatigue_rate:.2f}/10min')
        if self.stress_mode == 'time_based':
            self.get_logger().info(f'  Stress: starts at {self.initial_stress:.2f}, +{self.stress_rate:.2f}/10min')
        self.get_logger().info('=' * 60)
    
    def publish_state(self):
        """Calculate and publish operator state"""
        
        # Calculate mission elapsed time (seconds)
        elapsed = (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9
        
        # ============================================
        # FATIGUE CALCULATION
        # ============================================
        if self.fatigue_mode == 'time_based':
            # Fatigue increases linearly with time
            # fatigue_rate is per 10 minutes (600 seconds)
            time_factor = elapsed / 600.0
            self.current_fatigue = min(0.95, self.initial_fatigue + (self.fatigue_rate * time_factor))
            
        elif self.fatigue_mode == 'manual':
            # Use parameter value (can be updated via ros2 param set)
            self.current_fatigue = self.get_parameter('fatigue_value').value
            
        elif self.fatigue_mode == 'fixed':
            # Fixed value
            self.current_fatigue = self.fatigue_value
        
        # ============================================
        # STRESS CALCULATION
        # ============================================
        if self.stress_mode == 'time_based':
            # Stress increases with mission duration
            time_factor = elapsed / 600.0
            self.current_stress = min(0.95, self.initial_stress + (self.stress_rate * time_factor))
            
        elif self.stress_mode == 'manual':
            # Use parameter value
            self.current_stress = self.get_parameter('stress_value').value
            
        elif self.stress_mode == 'fixed':
            # Fixed value
            self.current_stress = self.stress_value
        
        # ============================================
        # PUBLISH
        # ============================================
        fatigue_msg = Float32()
        fatigue_msg.data = float(self.current_fatigue)
        self.fatigue_pub.publish(fatigue_msg)
        
        stress_msg = Float32()
        stress_msg.data = float(self.current_stress)
        self.stress_pub.publish(stress_msg)
        
        # Log periodically (every 30 seconds)
        if int(elapsed) % 30 == 0 and int(elapsed) > 0:
            self.get_logger().info(
                f'Operator State | '
                f'Fatigue: {self.current_fatigue:.3f} ({self.get_fatigue_category()}) | '
                f'Stress: {self.current_stress:.3f} ({self.get_stress_category()}) | '
                f'Mission time: {int(elapsed/60)}:{int(elapsed%60):02d}'
            )
    
    def get_fatigue_category(self):
        """Convert fatigue to category for logging"""
        if self.current_fatigue <= 0.3:
            return "Low"
        elif self.current_fatigue <= 0.6:
            return "Medium"
        else:
            return "High"
    
    def get_stress_category(self):
        """Convert stress to category for logging"""
        if self.current_stress <= 0.3:
            return "Low"
        elif self.current_stress <= 0.6:
            return "Medium"
        else:
            return "High"


def main(args=None):
    rclpy.init(args=args)
    
    node = OperatorStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
