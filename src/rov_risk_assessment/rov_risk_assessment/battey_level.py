#!/usr/bin/env python3
"""
Battery Level Categorizer
Converts battery percentage to categorical states: High/Medium/Low/Critical
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String


class BatteryLevelNode(Node):
    def __init__(self):
        super().__init__('battery_level_node')
        
        # Subscribe to battery percentage
        self.battery_sub = self.create_subscription(
            Float64,
            '/blueye/battery_percentage',
            self.battery_callback,
            10
        )
        
        # Publish battery level category
        self.level_pub = self.create_publisher(
            String,
            '/blueye/battery_level',
            10
        )
        
        self.get_logger().info('✓ Battery Level Categorizer started')
    
    def battery_callback(self, msg: Float64):
        """Categorize battery percentage"""
        
        percentage = msg.data
        
        level_msg = String()
        
        # Categorize based on percentage
        if percentage > 75.0:
            level_msg.data = "High"
        elif percentage > 50.0:
            level_msg.data = "Medium"
        elif percentage > 25.0:
            level_msg.data = "Low"
        else:
            level_msg.data = "Critical"
        
        self.level_pub.publish(level_msg)


def main():
    rclpy.init()
    node = BatteryLevelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()