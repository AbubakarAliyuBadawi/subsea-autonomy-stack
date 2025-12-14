#!/usr/bin/env python3
"""
Test Data Publisher for Mission Control Bayesian Network
Publishes simulated sensor data to test the BN inference system
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Int32
from geometry_msgs.msg import PoseStamped
import math
import random
from datetime import datetime


class TestDataPublisher(Node):
    def __init__(self):
        super().__init__('test_data_publisher')

        # Create publishers for all sensor topics
        self.speed_pub = self.create_publisher(Float32, '/blueye/speed', 10)
        self.usbl_pub = self.create_publisher(String, '/blueye/usbl_strength', 10)
        self.camera_pub = self.create_publisher(String, '/blueye/camera_quality', 10)
        self.battery_pub = self.create_publisher(Float32, '/blueye/battery_level', 10)
        self.altitude_pub = self.create_publisher(Float32, '/blueye/altitude', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/blueye/pose', 10)
        
        self.current_pub = self.create_publisher(Float32, '/blueye/current', 10)
        self.wind_pub = self.create_publisher(Float32, '/blueye/wind', 10)
        self.waves_pub = self.create_publisher(Float32, '/blueye/waves', 10)
        
        self.mission_phase_pub = self.create_publisher(String, '/blueye/mission_phase', 10)
        
        # Optional: Human operator states
        self.fatigue_pub = self.create_publisher(Float32, '/blueye/human/fatigue', 10)
        self.stress_pub = self.create_publisher(Float32, '/blueye/human/stress', 10)
        self.attention_pub = self.create_publisher(Float32, '/blueye/human/attention', 10)

        # Simulation state
        self.time = 0.0
        self.scenario = 'normal'  # Can be: 'normal', 'degrading', 'critical', 'recovery'
        self.scenario_start_time = 0.0
        
        # Initial values
        self.battery_level = 100.0
        self.position = [0.0, 0.0, -5.0]  # x, y, z (depth)
        self.velocity = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 1.0]  # quaternion
        
        self.fatigue_val = 0.1
        self.stress_val = 0.2
        self.attention_val = 0.9

        # Timer for publishing (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_all_data)

        self.get_logger().info('✓ Test Data Publisher initialized')
        self.get_logger().info('✓ Publishing to all /blueye/* topics')
        self.get_logger().info('✓ Starting with NORMAL scenario')

    def publish_all_data(self):
        """Publish all sensor data"""
        
        # Update simulation time
        self.time += 0.1
        
        # Check if we should change scenario
        self.update_scenario()
        
        # Generate and publish data based on current scenario
        self.publish_sensor_data()
        self.publish_environmental_data()
        self.publish_mission_context()
        self.publish_human_operator_data()
        
        # Battery depletes over time
        self.battery_level = max(0.0, self.battery_level - 0.01)  # ~1% per 10 seconds

    def update_scenario(self):
        """Update scenario based on time"""
        scenario_duration = self.time - self.scenario_start_time
        
        if self.scenario == 'normal' and scenario_duration > 30.0:
            self.scenario = 'degrading'
            self.scenario_start_time = self.time
            self.get_logger().info('⚠️  Scenario changed to: DEGRADING')
        
        elif self.scenario == 'degrading' and scenario_duration > 20.0:
            self.scenario = 'critical'
            self.scenario_start_time = self.time
            self.get_logger().info('🚨 Scenario changed to: CRITICAL')
        
        elif self.scenario == 'critical' and scenario_duration > 15.0:
            self.scenario = 'recovery'
            self.scenario_start_time = self.time
            self.get_logger().info('✅ Scenario changed to: RECOVERY')
        
        elif self.scenario == 'recovery' and scenario_duration > 25.0:
            self.scenario = 'normal'
            self.scenario_start_time = self.time
            self.get_logger().info('✓ Scenario changed to: NORMAL')

    def publish_sensor_data(self):
        """Publish autonomous control sensor data"""
        
        # Speed - varies by scenario
        if self.scenario == 'normal':
            speed = 0.3 + random.uniform(-0.1, 0.1)
        elif self.scenario == 'degrading':
            speed = 0.6 + random.uniform(-0.15, 0.15)
        elif self.scenario == 'critical':
            speed = 1.2 + random.uniform(-0.2, 0.2)
        else:  # recovery
            speed = 0.4 + random.uniform(-0.1, 0.1)
        
        speed_msg = Float32()
        speed_msg.data = max(0.0, speed)
        self.speed_pub.publish(speed_msg)
        
        # USBL Strength - degrades in critical scenario
        if self.scenario == 'normal' or self.scenario == 'recovery':
            usbl_states = ['Strong'] * 7 + ['Moderate'] * 2 + ['Weak'] * 1
        elif self.scenario == 'degrading':
            usbl_states = ['Strong'] * 4 + ['Moderate'] * 4 + ['Weak'] * 2
        else:  # critical
            usbl_states = ['Moderate'] * 3 + ['Weak'] * 5 + ['Lost'] * 2
        
        usbl_msg = String()
        usbl_msg.data = random.choice(usbl_states)
        self.usbl_pub.publish(usbl_msg)
        
        # Camera Quality - affected by environmental conditions
        if self.scenario == 'normal' or self.scenario == 'recovery':
            camera_states = ['Excellent'] * 5 + ['Good'] * 4 + ['Poor'] * 1
        elif self.scenario == 'degrading':
            camera_states = ['Good'] * 5 + ['Poor'] * 4 + ['Failed'] * 1
        else:  # critical
            camera_states = ['Poor'] * 6 + ['Failed'] * 4
        
        camera_msg = String()
        camera_msg.data = random.choice(camera_states)
        self.camera_pub.publish(camera_msg)
        
        # Battery Level
        battery_msg = Float32()
        battery_msg.data = self.battery_level
        self.battery_pub.publish(battery_msg)
        
        # Altitude - oscillates with some noise
        base_altitude = 2.0 + 0.5 * math.sin(self.time * 0.5)
        if self.scenario == 'critical':
            base_altitude = 0.8 + random.uniform(-0.3, 0.3)
        elif self.scenario == 'degrading':
            base_altitude = 1.5 + random.uniform(-0.4, 0.4)
        
        altitude_msg = Float32()
        altitude_msg.data = max(0.1, base_altitude + random.uniform(-0.2, 0.2))
        self.altitude_pub.publish(altitude_msg)
        
        # Pose - simulate movement with some drift in critical scenarios
        self.update_pose()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        
        pose_msg.pose.position.x = self.position[0]
        pose_msg.pose.position.y = self.position[1]
        pose_msg.pose.position.z = self.position[2]
        
        pose_msg.pose.orientation.x = self.orientation[0]
        pose_msg.pose.orientation.y = self.orientation[1]
        pose_msg.pose.orientation.z = self.orientation[2]
        pose_msg.pose.orientation.w = self.orientation[3]
        
        self.pose_pub.publish(pose_msg)

    def update_pose(self):
        """Update vehicle pose with simulated movement"""
        # Simple motion model
        dt = 0.1
        
        # Update position
        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt
        self.position[2] += self.velocity[2] * dt
        
        # Add noise based on scenario
        if self.scenario == 'critical':
            noise_scale = 0.1
        elif self.scenario == 'degrading':
            noise_scale = 0.05
        else:
            noise_scale = 0.01
        
        self.position[0] += random.uniform(-noise_scale, noise_scale)
        self.position[1] += random.uniform(-noise_scale, noise_scale)
        self.position[2] += random.uniform(-noise_scale * 0.5, noise_scale * 0.5)
        
        # Update orientation (small roll/pitch variations)
        roll_noise = random.uniform(-0.02, 0.02) * (3 if self.scenario == 'critical' else 1)
        pitch_noise = random.uniform(-0.02, 0.02) * (3 if self.scenario == 'critical' else 1)
        
        roll = roll_noise
        pitch = pitch_noise
        yaw = 0.0
        
        # Convert to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        self.orientation[0] = sr * cp * cy - cr * sp * sy  # x
        self.orientation[1] = cr * sp * cy + sr * cp * sy  # y
        self.orientation[2] = cr * cp * sy - sr * sp * cy  # z
        self.orientation[3] = cr * cp * cy + sr * sp * sy  # w

    def publish_environmental_data(self):
        """Publish environmental conditions"""
        
        # Current - increases in degrading/critical scenarios
        if self.scenario == 'normal' or self.scenario == 'recovery':
            current = 0.2 + random.uniform(-0.1, 0.1)
        elif self.scenario == 'degrading':
            current = 0.5 + random.uniform(-0.15, 0.15)
        else:  # critical
            current = 0.9 + random.uniform(-0.2, 0.2)
        
        current_msg = Float32()
        current_msg.data = max(0.0, current)
        self.current_pub.publish(current_msg)
        
        # Wind - surface conditions
        if self.scenario == 'normal' or self.scenario == 'recovery':
            wind = 3.0 + random.uniform(-1.0, 1.0)
        elif self.scenario == 'degrading':
            wind = 7.0 + random.uniform(-2.0, 2.0)
        else:  # critical
            wind = 12.0 + random.uniform(-2.0, 2.0)
        
        wind_msg = Float32()
        wind_msg.data = max(0.0, wind)
        self.wind_pub.publish(wind_msg)
        
        # Waves
        if self.scenario == 'normal' or self.scenario == 'recovery':
            waves = 0.3 + random.uniform(-0.1, 0.1)
        elif self.scenario == 'degrading':
            waves = 1.0 + random.uniform(-0.3, 0.3)
        else:  # critical
            waves = 2.0 + random.uniform(-0.4, 0.4)
        
        waves_msg = Float32()
        waves_msg.data = max(0.0, waves)
        self.waves_pub.publish(waves_msg)

    def publish_mission_context(self):
        """Publish mission phase"""
        
        # Cycle through mission phases based on time
        phase_cycle_time = self.time % 120.0  # 2-minute cycle
        
        if phase_cycle_time < 20:
            phase = 'Transit'
        elif phase_cycle_time < 50:
            phase = 'Inspection'
        elif phase_cycle_time < 70:
            phase = 'DockingApproach'
        elif phase_cycle_time < 90:
            phase = 'Docking'
        elif phase_cycle_time < 100:
            phase = 'Charging'
        else:
            phase = 'Undocking'
        
        phase_msg = String()
        phase_msg.data = phase
        self.mission_phase_pub.publish(phase_msg)

    def publish_human_operator_data(self):
        """Publish simulated human operator states"""
        
        # Fatigue accumulates over time
        self.fatigue_val += 0.0001  # Slow accumulation
        if self.scenario == 'critical':
            self.fatigue_val += 0.001  # Faster in critical situations
        self.fatigue_val = min(1.0, self.fatigue_val)
        
        # Stress responds to scenario
        if self.scenario == 'normal':
            target_stress = 0.2
        elif self.scenario == 'degrading':
            target_stress = 0.5
        elif self.scenario == 'critical':
            target_stress = 0.8
        else:  # recovery
            target_stress = 0.3
        
        # Smooth transition to target stress
        self.stress_val += (target_stress - self.stress_val) * 0.05
        self.stress_val = max(0.0, min(1.0, self.stress_val))
        
        # Attention degrades with fatigue and stress
        attention_degradation = (0.3 * self.fatigue_val + 0.2 * self.stress_val) * 0.01
        self.attention_val -= attention_degradation
        
        # Recovery during normal/recovery scenarios
        if self.scenario in ['normal', 'recovery']:
            self.attention_val += 0.002
        
        self.attention_val = max(0.2, min(1.0, self.attention_val))
        
        # Publish
        fatigue_msg = Float32()
        fatigue_msg.data = self.fatigue_val
        self.fatigue_pub.publish(fatigue_msg)
        
        stress_msg = Float32()
        stress_msg.data = self.stress_val
        self.stress_pub.publish(stress_msg)
        
        attention_msg = Float32()
        attention_msg.data = self.attention_val
        self.attention_pub.publish(attention_msg)


def main(args=None):
    rclpy.init(args=args)

    publisher = TestDataPublisher()

    try:
        publisher.get_logger().info('=' * 70)
        publisher.get_logger().info('🚀 Test Data Publisher Started')
        publisher.get_logger().info('=' * 70)
        publisher.get_logger().info('\nPublishing to topics:')
        publisher.get_logger().info('  • /blueye/speed')
        publisher.get_logger().info('  • /blueye/usbl_strength')
        publisher.get_logger().info('  • /blueye/camera_quality')
        publisher.get_logger().info('  • /blueye/battery_level')
        publisher.get_logger().info('  • /blueye/altitude')
        publisher.get_logger().info('  • /blueye/pose')
        publisher.get_logger().info('  • /blueye/current')
        publisher.get_logger().info('  • /blueye/wind')
        publisher.get_logger().info('  • /blueye/waves')
        publisher.get_logger().info('  • /blueye/mission_phase')
        publisher.get_logger().info('  • /blueye/human/* (fatigue, stress, attention)')
        publisher.get_logger().info('\n📊 Scenario Sequence:')
        publisher.get_logger().info('  1. NORMAL (30s) - Good conditions')
        publisher.get_logger().info('  2. DEGRADING (20s) - Conditions worsen')
        publisher.get_logger().info('  3. CRITICAL (15s) - Poor conditions')
        publisher.get_logger().info('  4. RECOVERY (25s) - Conditions improve')
        publisher.get_logger().info('  5. Loop back to NORMAL')
        publisher.get_logger().info('\n✓ Publishing at 10 Hz...')
        publisher.get_logger().info('=' * 70 + '\n')

        rclpy.spin(publisher)

    except KeyboardInterrupt:
        publisher.get_logger().info('\nShutting down Test Data Publisher...')

    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()