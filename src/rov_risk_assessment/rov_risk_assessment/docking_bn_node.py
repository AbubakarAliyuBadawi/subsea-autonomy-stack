#!/usr/bin/env python3
"""
Docking-Specialized Bayesian Network Node

This node implements the docking-specific risk assessment framework using
a specialized 24-node Bayesian Network derived from the full mission network.

Key Differences from Full Mission:
    - Removed: Wind, Waves, MissionPhase, TaskComplexity, TaskCriticality, OperatorWorkload
    - Specialized for docking operations only
    - Integrates ArUco marker detection for visual guidance assessment
    - Monitors fish-based docking clearance
    - Models human operator cognitive state (Fatigue/Stress/Attention)
    - Outputs docking-specific mode recommendation

Framework Contribution:
    This node demonstrates the systematic mapping from CoTA tasks to BN nodes:
    - ArUco detection (A3) → ArUcoMarkersVisible, DockingStationDetection
    - Clearance assessment (A5) → DockingClearance
    - Pose estimation (A4) → PoseEstimationQuality, PositionError
    - Human monitoring (H1-H4) → Attention, SituationalAwareness
    
Author: Badawi - PhD Research
Network: docking_specialized_bn.xdsl (24 nodes)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Int32, Bool
from geometry_msgs.msg import PoseStamped
import pysmile
import math
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
import os

# License activation (same as full mission)
pysmile.License((
    b"SMILE LICENSE b18e1861 e2fb7176 fd1b028d "
    b"THIS IS AN ACADEMIC LICENSE AND CAN BE USED "
    b"SOLELY FOR ACADEMIC RESEARCH AND TEACHING, "
    b"AS DEFINED IN THE BAYESFUSION ACADEMIC "
    b"SOFTWARE LICENSING AGREEMENT. "
    b"Serial #: 4aiii2myovtuprliry4nogll5 "
    b"Issued for: Abubakar Aliyu Badawi (badawi.abubakaraliyu@gmail.com) "
    b"Academic institution: Norwegian University of Science and Technology "
    b"Valid until: 2026-10-29 "
    b"Issued by BayesFusion activation server"
),[
    0xc1,0x74,0x1a,0x4e,0x81,0xdb,0x00,0x2e,0xd8,0xe3,0x62,0x95,0xa3,0x1d,0x14,0x19,
    0xca,0x73,0xba,0x84,0x77,0xe3,0x02,0x15,0x99,0xc7,0x09,0x8b,0xc0,0x7e,0x13,0xf9,
    0x84,0xd6,0x15,0xae,0x23,0x72,0x16,0x9f,0x92,0x86,0xd3,0x03,0xd1,0xe7,0x59,0xe8,
    0xe5,0xb5,0x73,0xe3,0xdb,0x9d,0xe0,0xc6,0x4d,0x4c,0x7f,0x31,0x9f,0x47,0x63,0xbd
])


class DockingBayesianNetwork(Node):
    def __init__(self):
        super().__init__('docking_bayesian_network')
        
        # Load the docking-specialized Bayesian Network
        try:
            package_share = get_package_share_directory('rov_risk_assessment')
            self.network_path = os.path.join(package_share, 'config', 'docking_bn_updated.xdsl')
            self.get_logger().info(f'Network path: {self.network_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to find package: {e}')
            raise
        
        self.net = pysmile.Network()
        
        try:
            self.net.read_file(self.network_path)
            self.get_logger().info(f'✓ Loaded Docking BN: {self.network_path}')
            
            all_nodes = self.net.get_all_node_ids()
            self.get_logger().info(f'✓ Network contains {len(all_nodes)} nodes (24 expected)')
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load docking network: {e}')
            raise

        # Store latest sensor data
        self.sensor_data = {
            'speed': None,
            'usbl_strength': None,
            'camera_quality': None,
            'battery_level': None,
            'altitude': None,
            'current': None,
            'aruco_visibility': None,
            'docking_detected': None,
            'fish_count': None,
            'pose_x': None,
            'pose_y': None,
            'pose_z': None,
            'fatigue': 0.2,  # Default low fatigue
            'stress': 0.2,   # Default low stress
        }

        # Historical data for pose quality
        self.pose_history = []
        self.max_history = 20

        # Mission timer for fatigue calculation
        self.mission_start_time = self.get_clock().now()

        # Setup ROS2 subscribers
        self.setup_subscribers()

        # Publishers for docking-specific outputs
        self.setup_publishers()

        # Timer for periodic network updates (1 Hz)
        self.create_timer(1.0, self.periodic_update)

        self.get_logger().info('✓ Docking Bayesian Network initialized')
        self.get_logger().info('🎯 Specialized for AUV docking operations')

    def setup_subscribers(self):
        """Setup ROS2 topic subscribers for docking evidence nodes"""
        
        # Vehicle state sensors (reuse from full mission)
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
        
        # Environmental (only Current for docking)
        self.current_sub = self.create_subscription(
            Float32, '/blueye/current', self.current_callback, 10)
        
        # Docking-specific: ArUco detection
        self.aruco_visibility_sub = self.create_subscription(
            String, '/blueye/aruco_visibility', self.aruco_visibility_callback, 10)
        
        self.docking_detected_sub = self.create_subscription(
            Bool, '/blueye/docking_station_detected', self.docking_detected_callback, 10)
        
        # Docking-specific: Fish-based clearance
        self.fish_count_sub = self.create_subscription(
            Int32, '/fish_detection/count', self.fish_count_callback, 10)
        
        # Pose estimation (for quality assessment)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/blueye/pose_estimated_board_stamped', 
            self.pose_callback, 10)
        
        # Human operator state (optional - can be manual or time-based)
        self.fatigue_sub = self.create_subscription(
            Float32, '/blueye/human/fatigue', self.fatigue_callback, 10)
        
        self.stress_sub = self.create_subscription(
            Float32, '/blueye/human/stress', self.stress_callback, 10)

        self.get_logger().info('✓ All ROS2 subscribers initialized')

    def setup_publishers(self):
        """Setup publishers for docking assessment outputs"""
        
        # Main outputs
        self.mode_pub = self.create_publisher(
            String, '/docking/mode_recommendation', 10)
        
        self.reliability_pub = self.create_publisher(
            Float32, '/docking/reliability', 10)
        
        self.visual_guidance_pub = self.create_publisher(
            String, '/docking/visual_guidance_quality', 10)
        
        self.approach_feasibility_pub = self.create_publisher(
            String, '/docking/approach_feasibility', 10)
        
        self.mode_auto_pub = self.create_publisher(
            Float32, '/docking/mode_autonomous_prob', 10)
        
        self.mode_human_pub = self.create_publisher(
            Float32, '/docking/mode_human_prob', 10)
        
        self.mode_shared_pub = self.create_publisher(
            Float32, '/docking/mode_shared_prob', 10)

        self.get_logger().info('✓ Docking output publishers initialized')

    # ============================================
    # DOCKING-SPECIFIC CALLBACKS
    # ============================================

    def aruco_visibility_callback(self, msg: String):
        """
        Process ArUco marker visibility (from your ArUco detection node)
        Maps to: A3.1 (Process camera frames to detect ArUco markers)
        BN Node: ArUcoMarkersVisible
        """
        visibility = msg.data  # "All", "Some", "None"
        self.sensor_data['aruco_visibility'] = visibility
        
        # ArUcoMarkersVisible states: All (0), Some (1), None (2)
        state_map = {
            'All': 0,
            'Some': 1,
            'None': 2
        }
        
        state = state_map.get(visibility, 2)
        
        try:
            self.net.set_evidence("ArUcoMarkersVisible", state)
            self.get_logger().debug(f'ArUco Visibility: {visibility} → state {state}')
        except Exception as e:
            self.get_logger().warning(f'Failed to set ArUcoMarkersVisible: {e}')

    def docking_detected_callback(self, msg: Bool):
        """
        Process docking station detection status (from your ArUco node)
        Maps to: A3.2 (D2: Determine if sufficient markers are visible)
        BN Node: DockingStationDetection
        """
        detected = msg.data
        self.sensor_data['docking_detected'] = detected
        
        # Determine detection state based on ArUco visibility
        aruco_vis = self.sensor_data.get('aruco_visibility', 'None')
        
        # DockingStationDetection states: Detected (0), Partial (1), NotDetected (2)
        if detected and aruco_vis == 'All':
            state = 0  # Detected (≥10 markers)
        elif detected and aruco_vis == 'Some':
            state = 1  # Partial (3-9 markers)
        else:
            state = 2  # NotDetected (<3 markers)
        
        try:
            self.net.set_evidence("DockingStationDetection", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set DockingStationDetection: {e}')

    def fish_count_callback(self, msg: Int32):
        """
        Process fish count for docking clearance assessment
        Maps to: A5.1 (Assess if path to docking station is unobstructed)
        BN Node: DockingClearance
        """
        fish_count = msg.data
        self.sensor_data['fish_count'] = fish_count
        
        # DockingClearance states: Clear (0), Obstructed (1), Unknown (2)
        # Logic: Fish inside docking cage = obstruction
        if fish_count == 0:
            state = 0  # Clear - no fish blocking
        elif fish_count > 0:
            state = 1  # Obstructed - fish detected
        else:
            state = 2  # Unknown
        
        try:
            self.net.set_evidence("DockingClearance", state)
            if fish_count > 0:
                self.get_logger().info(f'⚠️  Docking clearance OBSTRUCTED: {fish_count} fish detected')
        except Exception as e:
            self.get_logger().warning(f'Failed to set DockingClearance: {e}')

    def pose_callback(self, msg: PoseStamped):
        """
        Process pose estimation for quality assessment
        Maps to: A4.1 (Compute 6-DOF pose estimate from detected marker corners)
        BN Node: PoseEstimationQuality, PositionError
        """
        self.sensor_data['pose_x'] = msg.pose.position.x
        self.sensor_data['pose_y'] = msg.pose.position.y
        self.sensor_data['pose_z'] = msg.pose.position.z
        
        # Add to history for stability calculation
        self.pose_history.append({
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'timestamp': self.get_clock().now()
        })
        
        if len(self.pose_history) > self.max_history:
            self.pose_history.pop(0)
        
        # Calculate pose quality from stability
        # PoseEstimationQuality states: High (0), Medium (1), Low (2)
        if len(self.pose_history) >= 10:
            variance = self.calculate_position_variance()
            
            if variance < 0.05:
                pose_state = 0  # High quality
            elif variance < 0.15:
                pose_state = 1  # Medium quality
            else:
                pose_state = 2  # Low quality
            
            # PositionError states: Low (0), Medium (1), High (2)
            # Inverse of quality
            error_state = 2 - pose_state
            
            try:
                self.net.set_evidence("PoseEstimationQuality", pose_state)
                self.net.set_evidence("PositionError", error_state)
            except Exception as e:
                self.get_logger().warning(f'Failed to set pose evidence: {e}')

    # ============================================
    # REUSED CALLBACKS FROM FULL MISSION
    # ============================================

    def speed_callback(self, msg: Float32):
        """Process speed (reused from full mission)"""
        speed = msg.data
        self.sensor_data['speed'] = speed
        
        if speed < 0.5:
            state = 0  # Safe
        elif speed < 1.0:
            state = 1  # Moderate
        else:
            state = 2  # High
        
        try:
            self.net.set_evidence("Speed", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set Speed: {e}')

    def usbl_callback(self, msg: String):
        """Process USBL strength"""
        usbl_str = msg.data.lower()
        self.sensor_data['usbl_strength'] = usbl_str
        
        state_map = {'strong': 0, 'moderate': 1, 'weak': 2, 'lost': 3}
        state = state_map.get(usbl_str, 2)
        
        try:
            self.net.set_evidence("USBLStrength", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set USBLStrength: {e}')

    def camera_callback(self, msg: String):
        """Process camera quality"""
        camera_str = msg.data.lower()
        self.sensor_data['camera_quality'] = camera_str
        
        state_map = {'excellent': 0, 'good': 1, 'poor': 2, 'failed': 3}
        state = state_map.get(camera_str, 2)
        
        try:
            self.net.set_evidence("CameraQuality", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set CameraQuality: {e}')

    def battery_callback(self, msg: Float32):
        """Process battery level"""
        battery_pct = msg.data
        self.sensor_data['battery_level'] = battery_pct
        
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
            self.get_logger().warning(f'Failed to set BatteryLevel: {e}')

    def altitude_callback(self, msg: Float32):
        """Process altitude"""
        altitude = msg.data
        self.sensor_data['altitude'] = altitude
        
        if altitude > 2.0:
            state = 0  # Safe
        elif altitude > 1.0:
            state = 1  # Marginal
        else:
            state = 2  # Unsafe
        
        try:
            self.net.set_evidence("Altitude", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set Altitude: {e}')

    def current_callback(self, msg: Float32):
        """Process current"""
        current = msg.data
        self.sensor_data['current'] = current
        
        if current < 0.3:
            state = 0  # Low
        elif current < 0.7:
            state = 1  # Medium
        else:
            state = 2  # High
        
        try:
            self.net.set_evidence("Current", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set Current: {e}')

    # ============================================
    # HUMAN OPERATOR STATE
    # ============================================

    def fatigue_callback(self, msg: Float32):
        """Process operator fatigue (manual or time-based)"""
        fatigue = msg.data
        self.sensor_data['fatigue'] = fatigue
        
        # Fatigue states: Low (0), Medium (1), High (2)
        state = self.discretize(fatigue, [0.3, 0.6])
        
        try:
            self.net.set_evidence("Fatigue", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set Fatigue: {e}')

    def stress_callback(self, msg: Float32):
        """Process operator stress"""
        stress = msg.data
        self.sensor_data['stress'] = stress
        
        # Stress states: Low (0), Medium (1), High (2)
        state = self.discretize(stress, [0.3, 0.6])
        
        try:
            self.net.set_evidence("Stress", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set Stress: {e}')

    # ============================================
    # NETWORK INFERENCE AND PUBLISHING
    # ============================================

    def periodic_update(self):
        """Perform BN inference and publish results"""
        try:
            # Run inference
            self.net.update_beliefs()
            
            # Get docking-specific results
            docking_rel_probs = self.net.get_node_value("DockingReliability")
            mode_probs = self.net.get_node_value("DockingModeRecommendation")
            mode_states = self.net.get_outcome_ids("DockingModeRecommendation")
            
            # Publish mode probabilities (ADD THIS)
            auto_prob_msg = Float32()
            auto_prob_msg.data = float(mode_probs[0])
            self.mode_auto_pub.publish(auto_prob_msg)

            human_prob_msg = Float32()
            human_prob_msg.data = float(mode_probs[1])
            self.mode_human_pub.publish(human_prob_msg)

            shared_prob_msg = Float32()
            shared_prob_msg.data = float(mode_probs[2])
            self.mode_shared_pub.publish(shared_prob_msg)
            
            # Get intermediate assessments
            visual_guid_probs = self.net.get_node_value("VisualGuidanceQuality")
            visual_guid_states = self.net.get_outcome_ids("VisualGuidanceQuality")
            
            approach_feas_probs = self.net.get_node_value("ApproachFeasibility")
            approach_feas_states = self.net.get_outcome_ids("ApproachFeasibility")
            
            sitaware_probs = self.net.get_node_value("SituationalAwareness")
            auto_rel_probs = self.net.get_node_value("AutonomousControlReliability")
            
            # Calculate expected reliabilities
            reliability_values = [0.975, 0.90, 0.775, 0.60, 0.375]
            expected_docking_rel = sum(p * v for p, v in zip(docking_rel_probs, reliability_values))
            expected_auto_rel = sum(p * v for p, v in zip(auto_rel_probs, reliability_values))
            expected_sitaware = sum(p * v for p, v in zip(sitaware_probs, [0.95, 0.80, 0.55, 0.25]))
            
            # Find best modes
            best_mode_idx = mode_probs.index(max(mode_probs))
            recommended_mode = mode_states[best_mode_idx]
            mode_confidence = mode_probs[best_mode_idx]
            
            best_visual_idx = visual_guid_probs.index(max(visual_guid_probs))
            visual_quality = visual_guid_states[best_visual_idx]
            
            best_approach_idx = approach_feas_probs.index(max(approach_feas_probs))
            approach_status = approach_feas_states[best_approach_idx]
            
            # Publish outputs
            mode_msg = String()
            mode_msg.data = recommended_mode
            self.mode_pub.publish(mode_msg)
            
            rel_msg = Float32()
            rel_msg.data = float(expected_docking_rel)
            self.reliability_pub.publish(rel_msg)
            
            visual_msg = String()
            visual_msg.data = visual_quality
            self.visual_guidance_pub.publish(visual_msg)
            
            approach_msg = String()
            approach_msg.data = approach_status
            self.approach_feasibility_pub.publish(approach_msg)
            
            # Comprehensive logging
            self.get_logger().info('=' * 80)
            self.get_logger().info('DOCKING BAYESIAN NETWORK - RISK ASSESSMENT')
            self.get_logger().info('=' * 80)
            
            self.get_logger().info('\n📡 DOCKING SENSORS:')
            self.get_logger().info(f'  ArUco Markers: {self.sensor_data.get("aruco_visibility", "N/A")}')
            self.get_logger().info(f'  Station Detected: {self.sensor_data.get("docking_detected", "N/A")}')
            self.get_logger().info(f'  Fish Count: {self.sensor_data.get("fish_count", "N/A")}')
            self.get_logger().info(f'  Camera Quality: {self.sensor_data.get("camera_quality", "N/A")}')
            self.get_logger().info(f'  USBL Strength: {self.sensor_data.get("usbl_strength", "N/A")}')
            
            self.get_logger().info('\n🎯 DOCKING ASSESSMENT:')
            self.get_logger().info(f'  Visual Guidance: {visual_quality}')
            self.get_logger().info(f'  Approach Feasibility: {approach_status}')
            self.get_logger().info(f'  Docking Reliability: {expected_docking_rel:.3f}')
            
            self.get_logger().info('\n🤖 SYSTEM STATE:')
            self.get_logger().info(f'  Autonomous Reliability: {expected_auto_rel:.3f}')
            self.get_logger().info(f'  Operator Situational Awareness: {expected_sitaware:.3f}')
            
            self.get_logger().info('\n✨ DOCKING MODE RECOMMENDATION:')
            self.get_logger().info(f'  Autonomous: {mode_probs[0]:6.2%}  {"█" * int(mode_probs[0] * 30)}')
            self.get_logger().info(f'  Human:      {mode_probs[1]:6.2%}  {"█" * int(mode_probs[1] * 30)}')
            self.get_logger().info(f'  Shared:     {mode_probs[2]:6.2%}  {"█" * int(mode_probs[2] * 30)}')
            self.get_logger().info(f'\n  ➜ RECOMMENDED: {recommended_mode.upper()} (Confidence: {mode_confidence:.1%})')
            
            if mode_confidence < 0.5:
                self.get_logger().warn('  ⚠️  WARNING: Low confidence - consider shared control')
            
            if self.sensor_data.get('fish_count', 0) > 0:
                self.get_logger().warn(f'  🐟 CLEARANCE ISSUE: {self.sensor_data["fish_count"]} fish in docking area!')
            
            self.get_logger().info('=' * 80 + '\n')
            
        except Exception as e:
            self.get_logger().error(f'Network update error: {e}')

    # ============================================
    # UTILITY FUNCTIONS
    # ============================================

    def discretize(self, value: float, thresholds: list) -> int:
        """Convert continuous value to discrete state"""
        for i, threshold in enumerate(thresholds):
            if value <= threshold:
                return i
        return len(thresholds)

    def calculate_position_variance(self):
        """Calculate position variance for pose quality"""
        if len(self.pose_history) < 2:
            return 0.0
        
        x_values = [p['x'] for p in self.pose_history]
        y_values = [p['y'] for p in self.pose_history]
        z_values = [p['z'] for p in self.pose_history]
        
        x_var = self.calculate_variance(x_values)
        y_var = self.calculate_variance(y_values)
        z_var = self.calculate_variance(z_values)
        
        return math.sqrt(x_var + y_var + z_var)

    def calculate_variance(self, data):
        """Calculate variance"""
        if len(data) < 2:
            return 0.0
        mean = sum(data) / len(data)
        return sum((x - mean) ** 2 for x in data) / len(data)


def main(args=None):
    rclpy.init(args=args)
    
    docking_bn = DockingBayesianNetwork()
    
    try:
        docking_bn.get_logger().info('🚀 Docking Bayesian Network is running...')
        docking_bn.get_logger().info('🎯 Specialized risk assessment for AUV docking operations')
        
        rclpy.spin(docking_bn)
        
    except KeyboardInterrupt:
        docking_bn.get_logger().info('Shutting down...')
    finally:
        docking_bn.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
