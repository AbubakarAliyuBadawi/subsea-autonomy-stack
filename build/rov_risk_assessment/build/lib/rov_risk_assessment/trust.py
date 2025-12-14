#!/usr/bin/env python3
"""
Operator Trust Model (ECT-based)
Trust depends ONLY on Autonomous Control Reliability
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math


class OperatorTrustNode(Node):
    def __init__(self):
        super().__init__('operator_trust_node')
        
        # Trust state
        self.trust = 0.7
        self.expected_reliability = 0.7
        
        # ECT parameters
        self.alpha = 0.05  # Learning rate
        self.beta = 1.0    # Gap scaling
        self.gamma = 0.02  # Decay rate
        
        # History for consistency
        self.reliability_history = []
        self.max_history = 10
        
        # Subscribe to Autonomous Control Reliability
        self.auto_reliability_sub = self.create_subscription(
            String, '/blueye/autonomous_control_reliability', 
            self.reliability_callback, 10)
        
        # Publish trust
        self.trust_pub = self.create_publisher(String, '/blueye/operator_trust', 10)
        
        self.auto_reliability = 0.7
        
        # Update every 5 seconds
        self.create_timer(5.0, self.update_trust)
        
        self.get_logger().info('✓ Operator Trust Model started')
    
    def reliability_callback(self, msg: String):
        """Autonomous Control Reliability: VeryHigh/High/Medium/Low/VeryLow"""
        reliability_map = {
            'veryhigh': 1.0,
            'high': 0.8,
            'medium': 0.6,
            'low': 0.4,
            'verylow': 0.2
        }
        self.auto_reliability = reliability_map.get(msg.data.lower(), 0.6)
    
    def update_trust(self):
        """ECT Trust Update"""
        
        # 1. Performance gap
        gap = self.auto_reliability - self.expected_reliability
        
        # 2. Update history
        self.reliability_history.append(self.auto_reliability)
        if len(self.reliability_history) > self.max_history:
            self.reliability_history.pop(0)
        
        # 3. Consistency
        consistency = self.calculate_consistency()
        
        # 4. Trust change (ECT formula)
        trust_change = self.alpha * math.tanh(self.beta * gap)
        self.trust += trust_change
        
        # 5. Decay
        decay = self.gamma * (0.5 if gap >= 0 else 1.0) * (1.0 - consistency)
        self.trust *= (1.0 - decay)
        
        # 6. Clamp [0, 1]
        self.trust = max(0.0, min(1.0, self.trust))
        
        # 7. Update expectation
        self.expected_reliability = 0.9 * self.expected_reliability + 0.1 * self.auto_reliability
        
        # 8. Publish
        trust_msg = String()
        trust_msg.data = self.categorize_trust(self.trust)
        self.trust_pub.publish(trust_msg)
        
        self.get_logger().info(
            f'Trust: {trust_msg.data} ({self.trust:.2f}) | '
            f'Auto Reliability: {self.auto_reliability:.2f} | Gap: {gap:+.2f}'
        )
    
    def calculate_consistency(self):
        """Measure reliability consistency"""
        if len(self.reliability_history) < 3:
            return 0.5
        mean = sum(self.reliability_history) / len(self.reliability_history)
        variance = sum((r - mean)**2 for r in self.reliability_history) / len(self.reliability_history)
        std_dev = math.sqrt(variance)
        return 1.0 - min(std_dev / 0.3, 1.0)
    
    def categorize_trust(self, trust_value):
        """Convert to BN states"""
        if trust_value > 0.75:
            return "High"
        elif trust_value > 0.5:
            return "Medium"
        elif trust_value > 0.25:
            return "Low"
        else:
            return "VeryLow"


def main():
    rclpy.init()
    node = OperatorTrustNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()