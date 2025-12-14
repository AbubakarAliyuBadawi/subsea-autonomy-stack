#!/usr/bin/env python3
"""
Speed Publisher (DVL-based)
Calculates speed from DVL velocity measurements
"""

import rclpy
from rclpy.node import Node
from marine_acoustic_msgs.msg import Dvl
from std_msgs.msg import String
import math
from collections import deque


class SpeedPublisher(Node):
    def __init__(self):
        super().__init__('speed_publisher')
        
        # Parameters
        self.declare_parameter('buffer_size', 5)
        self.declare_parameter('alpha', 0.2)  # Low-pass filter
        
        buffer_size = self.get_parameter('buffer_size').value
        self.alpha = self.get_parameter('alpha').value
        
        # Speed filtering
        self.speed_buffer = deque(maxlen=buffer_size)
        self.filtered_speed = None
        self.filter_initialized = False
        
        # Subscribe to DVL
        self.dvl_sub = self.create_subscription(
            Dvl, '/blueye/dvl', self.dvl_callback, 10)
        
        # Publish speed category
        self.speed_pub = self.create_publisher(
            String, '/blueye/speed_category', 10)
        
        self.get_logger().info('✓ Speed Publisher (DVL-based) started')
    
    def dvl_callback(self, msg: Dvl):
        """Calculate speed from DVL velocity"""
        
        # Extract velocity from DVL
        # DVL provides velocity in instrument frame
        vx = msg.velocity.x
        vy = msg.velocity.y
        vz = msg.velocity.z
        
        # Calculate 3D speed magnitude
        raw_speed = math.sqrt(vx**2 + vy**2 + vz**2)
        
        # Initialize filter
        if not self.filter_initialized:
            self.filtered_speed = raw_speed
            for _ in range(self.speed_buffer.maxlen):
                self.speed_buffer.append(raw_speed)
            self.filter_initialized = True
        else:
            # Add to buffer
            self.speed_buffer.append(raw_speed)
            
            # Calculate median
            sorted_speeds = sorted(self.speed_buffer)
            median_speed = sorted_speeds[len(sorted_speeds) // 2]
            
            # Low-pass filter
            self.filtered_speed = (
                self.alpha * median_speed + 
                (1.0 - self.alpha) * self.filtered_speed
            )
        
        # Categorize and publish
        category = self.categorize_speed(self.filtered_speed)
        
        speed_msg = String()
        speed_msg.data = category
        self.speed_pub.publish(speed_msg)
    
    def categorize_speed(self, speed):
        """Categorize speed: Safe/Moderate/High"""
        if speed < 0.3:
            return "Safe"        # Slow, safe maneuvering
        elif speed < 0.6:
            return "Moderate"    # Medium speed
        else:
            return "High"        # High speed, less control


def main():
    rclpy.init()
    node = SpeedPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()