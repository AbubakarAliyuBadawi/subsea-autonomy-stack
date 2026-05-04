#!/usr/bin/env python3
"""
Turbidity Publisher
Estimates water turbidity from environmental conditions and fog settings
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32


class TurbidityPublisher(Node):
    def __init__(self):
        super().__init__('turbidity_publisher')
        
        # Declare parameters (match your Gazebo fog settings)
        self.declare_parameter('base_turbidity', 0.08)  # Your fog density from SDF
        self.declare_parameter('update_rate', 1.0)      # Hz
        
        self.base_turbidity = self.get_parameter('base_turbidity').value
        update_rate = self.get_parameter('update_rate').value
        
        # Subscribe to environmental conditions (affect turbidity)
        self.current_sub = self.create_subscription(
            Float32, '/blueye/current', self.current_callback, 10)
        self.waves_sub = self.create_subscription(
            Float32, '/blueye/waves', self.waves_callback, 10)
        
        # Publish turbidity
        self.turbidity_pub = self.create_publisher(
            String, '/blueye/turbidity', 10)
        
        # Environmental state
        self.current = 0.0
        self.waves = 0.0
        
        # Timer for periodic publishing
        self.create_timer(1.0 / update_rate, self.publish_turbidity)
        
        self.get_logger().info(f'✓ Turbidity Publisher started (base: {self.base_turbidity})')
    
    def current_callback(self, msg: Float32):
        self.current = msg.data
    
    def waves_callback(self, msg: Float32):
        self.waves = msg.data
    
    def publish_turbidity(self):
        """
        Calculate turbidity based on:
        1. Base turbidity (from Gazebo fog density)
        2. Current speed (stirs up sediment)
        3. Wave height (surface disturbance)
        """
        
        # Base turbidity from simulation
        turbidity = self.base_turbidity
        
        # Current increases turbidity (stirs sediment)
        # High current (>0.5 m/s) significantly increases turbidity
        turbidity += self.current * 0.15
        
        # Waves increase turbidity (surface mixing, sediment suspension)
        turbidity += self.waves * 0.1
        
        # Clamp to [0, 1]
        turbidity = max(0.0, min(1.0, turbidity))
        
        # Categorize
        category = self.categorize_turbidity(turbidity)
        
        turbidity_msg = String()
        turbidity_msg.data = category
        self.turbidity_pub.publish(turbidity_msg)
    
    def categorize_turbidity(self, turbidity):
        """
        Categorize turbidity based on visibility impact
        Matches your Gazebo fog density ranges
        """
        if turbidity < 0.05:
            return "Clear"       # Crystal clear (fog_density < 0.05)
        elif turbidity < 0.15:
            return "Moderate"    # Some turbidity (fog_density 0.05-0.15)
        else:
            return "High"        # Very turbid (fog_density > 0.15)


def main():
    rclpy.init()
    node = TurbidityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()