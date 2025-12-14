#!/usr/bin/env python3
"""
Mission Control Bayesian Network Node - Real-time Autonomous/Human Control Arbitration

This ROS2 node acts as an intelligent mission mode advisor for the Blueye ROV system.
It continuously monitors sensor data, environmental conditions, and human operator state,
then uses Bayesian inference to recommend the safest control mode (Autonomous, Human, or Shared).

Main Functions:
    - Subscribes to 15+ ROV sensor topics (speed, battery, camera, USBL, pose, etc.)
    - Discretizes continuous sensor data into categorical states
    - Updates integrated Bayesian Network with real-time evidence
    - Performs probabilistic inference every 1 second
    - Compares human vs autonomous reliability under current conditions
    - Recommends optimal control mode with confidence percentage
    - Logs comprehensive mission status dashboard

Control Mode Logic:
    - AUTONOMOUS: When autonomous systems are more reliable (good sensors, fatigued operator)
    - HUMAN: When human is more reliable (degraded sensors, critical phase, harsh environment)
    - SHARED: When reliabilities are similar or confidence is low

Key Decisions Influenced By:
    ✓ Environmental conditions (current, wind, waves)
    ✓ Sensor health (USBL, camera, pose estimation quality)
    ✓ Vehicle state (battery, speed, altitude)
    ✓ Mission phase (transit, inspection, docking)
    ✓ Human operator state (fatigue, stress, attention) [optional]
    ✓ Phase-specific factors (e.g., ArUco detection during docking)

Use Case Example:
    During docking approach with poor camera quality and high current, the network might
    recommend switching from Autonomous to Human control due to degraded visual guidance
    and increased environmental disturbances, even if the operator is moderately fatigued.

Author: Badawi - PhD Research on Human-Autonomy Collaboration
Network: integrated_mission_control.xdsl (270-node Bayesian Network)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Int32
from geometry_msgs.msg import PoseStamped
import pysmile
import math
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
import os

# License activation
pysmile.License((
    b"SMILE LICENSE ad8f9ea4 fd10eb55 67388448 "
    b"THIS IS AN ACADEMIC LICENSE AND CAN BE USED "
    b"SOLELY FOR ACADEMIC RESEARCH AND TEACHING, "
    b"AS DEFINED IN THE BAYESFUSION ACADEMIC "
    b"SOFTWARE LICENSING AGREEMENT. "
    b"Serial #: bb6p59p7s2dnilegmhuwlzrgi "
    b"Issued for: Abubakar Aliyu Badawi (abubakaraliyubadawi@gmail.com) "
    b"Academic institution: Norwegian University of Science and Technology "
    b"Valid until: 2026-04-18 "
    b"Issued by BayesFusion activation server"
),[
    0xcc,0x17,0xa5,0x36,0x16,0x16,0xab,0x31,0x9f,0x45,0xab,0xbd,0x7d,0xe4,0x20,0xed,
    0xe8,0xc7,0xba,0xae,0x34,0x5a,0xf5,0x7d,0xf6,0xf9,0x13,0x67,0xb2,0x1c,0xc5,0xef,
    0xe3,0xcc,0x46,0xb7,0xc7,0x23,0x81,0x0d,0x98,0x39,0x18,0x08,0xc1,0x1e,0x67,0x25,
    0xbb,0xfe,0x38,0x5b,0xb6,0x4a,0xee,0xb9,0xbd,0xd0,0xf1,0x3b,0x40,0xbc,0x2c,0xd8
])


class MissionControlBayesianNetwork(Node):
    def __init__(self):
        super().__init__('mission_control_bayesian_network')
        try:
            package_share = get_package_share_directory('rov_risk_assessment')
            self.network_path = os.path.join(package_share, 'config', 'integrated_mission_control.xdsl')
            self.get_logger().info(f'Network path: {self.network_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to find package: {e}')
            raise
    
        # Load the integrated mission control Bayesian Network
        self.net = pysmile.Network()
        # self.network_path = "integrated_mission_control.xdsl"
        
        try:
            self.net.read_file(self.network_path)
            self.get_logger().info(f'✓ Loaded Bayesian Network: {self.network_path}')
            
            # Display network info
            all_nodes = self.net.get_all_node_ids()
            self.get_logger().info(f'✓ Network contains {len(all_nodes)} nodes')
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load network: {e}')
            raise

        # Store latest sensor data
        self.sensor_data = {
            'speed': None,
            'usbl_strength': None,
            'camera_quality': None,
            'battery_level': None,
            'altitude': None,
            'current': None,
            'wind': None,
            'waves': None,
            'mission_phase': None,
            'pose_x': None,
            'pose_y': None,
            'pose_z': None,
            'roll': None,
            'pitch': None,
            'yaw': None,
        }

        # Historical data for stability and quality calculations
        self.pose_history = []
        self.max_history = 20

        # Setup ROS2 subscribers
        self.setup_subscribers()

        # Timer for periodic network updates and logging (1 Hz)
        self.create_timer(1.0, self.periodic_update)

        self.get_logger().info('✓ Mission Control Bayesian Network initialized')
        self.get_logger().info('✓ Waiting for sensor data from ROS2 topics...')

    def setup_subscribers(self):
        """Setup ROS2 topic subscribers for all sensor inputs"""
        
        # Autonomous Control sensors
        self.speed_sub = self.create_subscription(
            Float32, '/blueye/speed', self.speed_callback, 10)
        
        self.usbl_sub = self.create_subscription(
            String, '/blueye/usbl_strength', self.usbl_callback, 10)
        
        self.camera_sub = self.create_subscription(
            String, '/blueye/camera_quality', self.camera_callback, 10)
        
        self.battery_sub = self.create_subscription(
            Float32, '/blueye/battery_level', self.battery_callback, 10)
        
        self.altitude_sub = self.create_subscription(
            Float32, '/blueye/altitude', self.altitude_callback, 10)
        
        self.pose_sub = self.create_subscription(
            PoseStamped, '/blueye/pose', self.pose_callback, 10)
        
        # Environmental conditions
        self.current_sub = self.create_subscription(
            Float32, '/blueye/current', self.current_callback, 10)
        
        self.wind_sub = self.create_subscription(
            Float32, '/blueye/wind', self.wind_callback, 10)
        
        self.waves_sub = self.create_subscription(
            Float32, '/blueye/waves', self.waves_callback, 10)
        
        # Mission context
        self.mission_phase_sub = self.create_subscription(
            String, '/blueye/mission_phase', self.mission_phase_callback, 10)
        
        # Human operator states (if available - optional)
        self.fatigue_sub = self.create_subscription(
            Float32, '/blueye/human/fatigue', self.fatigue_callback, 10)
        
        self.stress_sub = self.create_subscription(
            Float32, '/blueye/human/stress', self.stress_callback, 10)
        
        self.attention_sub = self.create_subscription(
            Float32, '/blueye/human/attention', self.attention_callback, 10)

        self.get_logger().info('✓ All ROS2 subscribers initialized')

    # ============================================
    # ROS2 CALLBACK FUNCTIONS - SENSORS
    # ============================================

    def speed_callback(self, msg: Float32):
        """Process speed data and categorize for Speed node"""
        speed = msg.data
        self.sensor_data['speed'] = speed
        
        # Categorize speed based on current conditions
        # For now, use simple thresholds (can be made adaptive)
        # Speed states: Safe (0), Moderate (1), High (2)
        if speed < 0.5:
            state = 0  # Safe
        elif speed < 1.0:
            state = 1  # Moderate
        else:
            state = 2  # High
        
        try:
            self.net.set_evidence("Speed", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set Speed evidence: {e}')

    def usbl_callback(self, msg: String):
        """Process USBL strength data"""
        usbl_str = msg.data.lower()
        self.sensor_data['usbl_strength'] = usbl_str
        
        # USBLStrength states: Strong (0), Moderate (1), Weak (2), Lost (3)
        state_map = {
            'strong': 0,
            'moderate': 1,
            'weak': 2,
            'lost': 3
        }
        
        state = state_map.get(usbl_str, 2)  # Default to Weak
        
        try:
            self.net.set_evidence("USBLStrength", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set USBLStrength evidence: {e}')

    def camera_callback(self, msg: String):
        """Process camera quality data"""
        camera_str = msg.data.lower()
        self.sensor_data['camera_quality'] = camera_str
        
        # CameraQuality states: Excellent (0), Good (1), Poor (2), Failed (3)
        state_map = {
            'excellent': 0,
            'good': 1,
            'poor': 2,
            'failed': 3
        }
        
        state = state_map.get(camera_str, 2)  # Default to Poor
        
        try:
            self.net.set_evidence("CameraQuality", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set CameraQuality evidence: {e}')

    def battery_callback(self, msg: Float32):
        """Process battery level data (percentage 0-100)"""
        battery_pct = msg.data
        self.sensor_data['battery_level'] = battery_pct
        
        # BatteryLevel states: High (0), Medium (1), Low (2), Critical (3)
        if battery_pct > 60:
            state = 0  # High
        elif battery_pct > 30:
            state = 1  # Medium
        elif battery_pct > 15:
            state = 2  # Low
        else:
            state = 3  # Critical
        
        try:
            self.net.set_evidence("BatteryLevel", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set BatteryLevel evidence: {e}')

    def altitude_callback(self, msg: Float32):
        """Process altitude data"""
        altitude = msg.data
        self.sensor_data['altitude'] = altitude
        
        # Altitude states: Safe (0), Marginal (1), Unsafe (2)
        if altitude > 2.0:
            state = 0  # Safe (> 2m from obstacles)
        elif altitude > 1.0:
            state = 1  # Marginal (1-2m)
        else:
            state = 2  # Unsafe (< 1m)
        
        try:
            self.net.set_evidence("Altitude", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set Altitude evidence: {e}')

    def pose_callback(self, msg: PoseStamped):
        """Process pose data and calculate pose estimation quality"""
        # Store position
        self.sensor_data['pose_x'] = msg.pose.position.x
        self.sensor_data['pose_y'] = msg.pose.position.y
        self.sensor_data['pose_z'] = msg.pose.position.z
        
        # Convert quaternion to Euler angles
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
        self.sensor_data['roll'] = roll
        self.sensor_data['pitch'] = pitch
        self.sensor_data['yaw'] = yaw
        
        # Add to history for stability calculation
        self.pose_history.append({
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'timestamp': self.get_clock().now()
        })
        
        if len(self.pose_history) > self.max_history:
            self.pose_history.pop(0)
        
        # Calculate pose estimation quality based on stability
        # PoseEstimationQuality states: High (0), Medium (1), Low (2)
        if len(self.pose_history) >= 10:
            variance = self.calculate_position_variance()
            
            if variance < 0.05:
                state = 0  # High quality (error < 0.1m)
            elif variance < 0.15:
                state = 1  # Medium quality (error 0.1-0.3m)
            else:
                state = 2  # Low quality (error > 0.3m)
        else:
            state = 1  # Default to Medium until we have enough history
        
        try:
            self.net.set_evidence("PoseEstimationQuality", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set PoseEstimationQuality evidence: {e}')

    # ============================================
    # ROS2 CALLBACK FUNCTIONS - ENVIRONMENT
    # ============================================

    def current_callback(self, msg: Float32):
        """Process current data (m/s)"""
        current = msg.data
        self.sensor_data['current'] = current
        
        # Current states: Low (0), Medium (1), High (2)
        if current < 0.3:
            state = 0  # Low
        elif current < 0.7:
            state = 1  # Medium
        else:
            state = 2  # High
        
        try:
            self.net.set_evidence("Current", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set Current evidence: {e}')

    def wind_callback(self, msg: Float32):
        """Process wind data (m/s)"""
        wind = msg.data
        self.sensor_data['wind'] = wind
        
        # Wind states: Low (0), Medium (1), High (2)
        if wind < 5.0:
            state = 0  # Low
        elif wind < 10.0:
            state = 1  # Medium
        else:
            state = 2  # High
        
        try:
            self.net.set_evidence("Wind", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set Wind evidence: {e}')

    def waves_callback(self, msg: Float32):
        """Process wave height data (m)"""
        waves = msg.data
        self.sensor_data['waves'] = waves
        
        # Waves states: Low (0), Medium (1), High (2)
        if waves < 0.5:
            state = 0  # Low
        elif waves < 1.5:
            state = 1  # Medium
        else:
            state = 2  # High
        
        try:
            self.net.set_evidence("Waves", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set Waves evidence: {e}')

    # ============================================
    # ROS2 CALLBACK FUNCTIONS - MISSION CONTEXT
    # ============================================

    def mission_phase_callback(self, msg: String):
        """Process mission phase"""
        phase_str = msg.data.lower()
        self.sensor_data['mission_phase'] = phase_str
        
        # MissionPhase states: Undocking (0), Transit (1), Inspection (2), 
        #                       DockingApproach (3), Docking (4), Charging (5)
        phase_map = {
            'undocking': 0,
            'transit': 1,
            'inspection': 2,
            'dockingapproach': 3,
            'docking': 4,
            'charging': 5
        }
        
        state = phase_map.get(phase_str, 1)  # Default to Transit
        
        try:
            self.net.set_evidence("MissionPhase", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set MissionPhase evidence: {e}')

    # ============================================
    # ROS2 CALLBACK FUNCTIONS - HUMAN OPERATOR (Optional)
    # ============================================

    def fatigue_callback(self, msg: Float32):
        """Process fatigue data (continuous 0-1)"""
        fatigue = msg.data
        
        # Discretize: Fatigue states: Low (0), Medium (1), High (2)
        state = self.discretize(fatigue, [0.3, 0.6])
        
        try:
            self.net.set_evidence("Fatigue", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set Fatigue evidence: {e}')

    def stress_callback(self, msg: Float32):
        """Process stress data (continuous 0-1)"""
        stress = msg.data
        
        # Discretize: Stress states: Low (0), Medium (1), High (2)
        state = self.discretize(stress, [0.3, 0.6])
        
        try:
            self.net.set_evidence("Stress", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set Stress evidence: {e}')

    def attention_callback(self, msg: Float32):
        """Process attention data (continuous 0-1)"""
        attention = msg.data
        
        # Discretize: Attention states: Low (0), Medium (1), High (2)
        state = self.discretize(attention, [0.4, 0.7])
        
        try:
            self.net.set_evidence("Attention", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set Attention evidence: {e}')

    # ============================================
    # NETWORK UPDATE AND INFERENCE
    # ============================================

    def periodic_update(self):
        """Periodic network inference and logging"""
        try:
            # Run inference
            self.net.update_beliefs()
            
            # Get key results
            human_rel_probs = self.net.get_node_value("HumanDecisionReliability")
            auto_rel_probs = self.net.get_node_value("AutonomousControlReliability")
            mode_probs = self.net.get_node_value("MissionModeRecommendation")
            mode_states = self.net.get_outcome_ids("MissionModeRecommendation")
            
            # Calculate expected reliabilities
            reliability_values = [0.975, 0.90, 0.775, 0.60, 0.375]
            expected_human_rel = sum(p * v for p, v in zip(human_rel_probs, reliability_values))
            expected_auto_rel = sum(p * v for p, v in zip(auto_rel_probs, reliability_values))
            
            # Find recommended mode
            best_mode_idx = mode_probs.index(max(mode_probs))
            recommended_mode = mode_states[best_mode_idx]
            confidence = mode_probs[best_mode_idx]
            
            # Log results
            self.get_logger().info('=' * 70)
            self.get_logger().info('MISSION CONTROL BAYESIAN NETWORK STATE')
            self.get_logger().info('=' * 70)
            
            # Sensor data
            self.get_logger().info('\n📡 SENSOR DATA:')
            self.get_logger().info(f'  Speed: {self.sensor_data.get("speed", "N/A")} m/s')
            self.get_logger().info(f'  USBL Strength: {self.sensor_data.get("usbl_strength", "N/A")}')
            self.get_logger().info(f'  Camera Quality: {self.sensor_data.get("camera_quality", "N/A")}')
            self.get_logger().info(f'  Battery: {self.sensor_data.get("battery_level", "N/A")}%')
            self.get_logger().info(f'  Altitude: {self.sensor_data.get("altitude", "N/A")} m')
            
            # Environmental conditions
            self.get_logger().info('\n🌊 ENVIRONMENTAL CONDITIONS:')
            self.get_logger().info(f'  Current: {self.sensor_data.get("current", "N/A")} m/s')
            self.get_logger().info(f'  Wind: {self.sensor_data.get("wind", "N/A")} m/s')
            self.get_logger().info(f'  Waves: {self.sensor_data.get("waves", "N/A")} m')
            
            # Mission context
            self.get_logger().info('\n🎯 MISSION CONTEXT:')
            self.get_logger().info(f'  Phase: {self.sensor_data.get("mission_phase", "N/A")}')
            
            # Pose data
            if self.sensor_data.get('roll') is not None:
                self.get_logger().info('\n📐 POSE:')
                self.get_logger().info(f'  Position: ({self.sensor_data.get("pose_x", 0):.2f}, '
                                      f'{self.sensor_data.get("pose_y", 0):.2f}, '
                                      f'{self.sensor_data.get("pose_z", 0):.2f})')
                self.get_logger().info(f'  Roll: {math.degrees(self.sensor_data["roll"]):.1f}°')
                self.get_logger().info(f'  Pitch: {math.degrees(self.sensor_data["pitch"]):.1f}°')
                self.get_logger().info(f'  Yaw: {math.degrees(self.sensor_data["yaw"]):.1f}°')
            
            # Reliability assessments
            self.get_logger().info('\n🤖 RELIABILITY ASSESSMENT:')
            self.get_logger().info(f'  Human Operator Reliability: {expected_human_rel:.3f}')
            self.get_logger().info(f'  Autonomous Control Reliability: {expected_auto_rel:.3f}')
            
            # Mode recommendation
            self.get_logger().info('\n✨ MISSION MODE RECOMMENDATION:')
            self.get_logger().info(f'  Autonomous: {mode_probs[0]:6.2%}  {"█" * int(mode_probs[0] * 30)}')
            self.get_logger().info(f'  Human:      {mode_probs[1]:6.2%}  {"█" * int(mode_probs[1] * 30)}')
            self.get_logger().info(f'  Shared:     {mode_probs[2]:6.2%}  {"█" * int(mode_probs[2] * 30)}')
            self.get_logger().info(f'\n  ➜ RECOMMENDED: {recommended_mode.upper()} (Confidence: {confidence:.1%})')
            
            if confidence < 0.5:
                self.get_logger().warn('  ⚠️  WARNING: Low confidence - consider shared control')
            
            self.get_logger().info('=' * 70 + '\n')
            
        except Exception as e:
            self.get_logger().warning(f'Could not update network: {e}')

    # ============================================
    # UTILITY FUNCTIONS
    # ============================================

    def discretize(self, value: float, thresholds: list) -> int:
        """
        Convert continuous value to discrete state index
        
        Args:
            value: Continuous value (0-1)
            thresholds: List of threshold values
            
        Returns:
            State index (0, 1, 2, ...)
        """
        for i, threshold in enumerate(thresholds):
            if value <= threshold:
                return i
        return len(thresholds)

    def calculate_position_variance(self):
        """Calculate variance of position over history"""
        if len(self.pose_history) < 2:
            return 0.0
        
        x_values = [p['x'] for p in self.pose_history]
        y_values = [p['y'] for p in self.pose_history]
        z_values = [p['z'] for p in self.pose_history]
        
        x_var = self.calculate_variance(x_values)
        y_var = self.calculate_variance(y_values)
        z_var = self.calculate_variance(z_values)
        
        # Return total variance
        return math.sqrt(x_var + y_var + z_var)

    def calculate_variance(self, data):
        """Calculate variance of a list"""
        if len(data) < 2:
            return 0.0
        mean = sum(data) / len(data)
        variance = sum((x - mean) ** 2 for x in data) / len(data)
        return variance

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def save_network(self, filename="mission_control_network_snapshot.xdsl"):
        """Save the current network state to file"""
        try:
            self.net.write_file(filename)
            self.get_logger().info(f'Network saved to {filename}')
        except Exception as e:
            self.get_logger().error(f'Failed to save network: {e}')


def main(args=None):
    rclpy.init(args=args)

    # Create mission control Bayesian network node
    bayesian_node = MissionControlBayesianNetwork()

    try:
        bayesian_node.get_logger().info('🚀 Mission Control Bayesian Network is running...')
        bayesian_node.get_logger().info('📡 Subscribing to Blueye ROS2 topics')
        bayesian_node.get_logger().info('🔄 Network will update in real-time based on sensor data')

        rclpy.spin(bayesian_node)

    except KeyboardInterrupt:
        bayesian_node.get_logger().info('Shutting down...')
        bayesian_node.save_network()

    finally:
        bayesian_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
