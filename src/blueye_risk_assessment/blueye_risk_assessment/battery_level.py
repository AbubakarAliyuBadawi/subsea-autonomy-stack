#!/usr/bin/env python3
"""
Battery Level Categorizer — Real Blueye hardware version

Subscribes to /blueye/battery (geometry_msgs/Pose) published by blueye_telemetry.py.
Field mapping:
  position.x = charging_current (A)
  position.y = relative_state_of_charge (0-100 %)
  position.z = current draw (A)
  orientation.x = runtime_to_empty (s)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String


class BatteryLevelNode(Node):
    def __init__(self):
        super().__init__('battery_level_node')

        self.battery_sub = self.create_subscription(
            Pose, '/blueye/battery', self.battery_callback, 10)

        self.level_pub = self.create_publisher(String, '/blueye/battery_level', 10)

        self.get_logger().info('Battery Level Categorizer started (real Blueye)')

    def battery_callback(self, msg: Pose):
        # position.y is relative_state_of_charge (0–100 %)
        percentage = msg.position.y

        level_msg = String()
        if percentage > 75.0:
            level_msg.data = 'High'
        elif percentage > 50.0:
            level_msg.data = 'Medium'
        elif percentage > 25.0:
            level_msg.data = 'Low'
        else:
            level_msg.data = 'Critical'

        self.level_pub.publish(level_msg)


def main():
    rclpy.init()
    node = BatteryLevelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
