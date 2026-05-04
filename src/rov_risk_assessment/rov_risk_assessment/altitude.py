#!/usr/bin/env python3
"""
Altitude Publisher
Extracts altitude from DVL and categorizes it for Bayesian Network
"""

import rclpy
from rclpy.node import Node
from marine_acoustic_msgs.msg import Dvl
from std_msgs.msg import String
from collections import deque


class AltitudePublisher(Node):
    def __init__(self):
        super().__init__('altitude_publisher')
        
        # Parameters
        self.declare_parameter('buffer_size', 5)
        self.declare_parameter('alpha', 0.1)  # Low-pass filter
        
        buffer_size = self.get_parameter('buffer_size').value
        self.alpha = self.get_parameter('alpha').value
        
        # Altitude filtering
        self.altitude_buffer = deque(maxlen=buffer_size)
        self.filtered_altitude = None
        self.filter_initialized = False
        
        # Subscribe to DVL
        self.dvl_sub = self.create_subscription(
            Dvl, '/blueye/dvl', self.dvl_callback, 10)
        
        # Publish altitude category
        self.altitude_pub = self.create_publisher(
            String, '/blueye/altitude_category', 10)
        
        self.get_logger().info('✓ Altitude Publisher started')
    
    def dvl_callback(self, msg: Dvl):
        """Process DVL altitude measurement"""
        
        raw_altitude = msg.altitude
        
        # Initialize filter on first valid measurement
        if not self.filter_initialized:
            self.filtered_altitude = raw_altitude
            for _ in range(self.altitude_buffer.maxlen):
                self.altitude_buffer.append(raw_altitude)
            self.filter_initialized = True
        else:
            # Add to buffer
            self.altitude_buffer.append(raw_altitude)
            
            # Calculate median (robust against outliers)
            sorted_altitudes = sorted(self.altitude_buffer)
            median_altitude = sorted_altitudes[len(sorted_altitudes) // 2]
            
            # Low-pass filter on median
            self.filtered_altitude = (
                self.alpha * median_altitude + 
                (1.0 - self.alpha) * self.filtered_altitude
            )
        
        # Categorize and publish
        category = self.categorize_altitude(self.filtered_altitude)
        
        altitude_msg = String()
        altitude_msg.data = category
        self.altitude_pub.publish(altitude_msg)
    
    def categorize_altitude(self, altitude):
        """
        Categorize altitude: Safe/Marginal/Unsafe
        Based on typical ROV operating ranges
        """
        if altitude > 3.0:
            return "Safe"        # High altitude, safe clearance
        elif altitude > 1.5:
            return "Marginal"    # Medium altitude, caution needed
        else:
            return "Unsafe"      # Low altitude, collision risk


def main():
    rclpy.init()
    node = AltitudePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()