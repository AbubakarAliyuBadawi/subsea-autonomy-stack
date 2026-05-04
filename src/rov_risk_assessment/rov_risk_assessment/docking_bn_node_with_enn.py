#!/usr/bin/env python3
"""
Docking Bayesian Network Node with Evidential Neural Network Integration

This node extends the original docking_bn_node.py by replacing fixed threshold
discretization with learned soft probabilities from an Evidential Neural Network.

Key Changes from Original:
    - Added ENN predictor for Altitude and Speed nodes
    - Uses set_virtual_evidence() instead of set_evidence() for soft probabilities
    - Maintains a rolling buffer of sensor data for temporal inference
    - Falls back to threshold-based discretization when ENN uncertainty is high

Real-time Operation:
    - ENN runs at 10Hz (collecting sensor data)
    - BN inference runs at 1Hz (using latest ENN predictions)
    - Latency: ~5ms per ENN inference on GPU, ~20ms on CPU

Author: Badawi - PhD Research
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Int32, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import pysmile
import math
import numpy as np
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from collections import deque
import os
import sys

# Add paper2 directory to path for ENN imports
sys.path.append('/home/badawi/Desktop/PhD_Autumn_2025/paper2')

# Import ENN predictor
try:
    from inference_enn import ENNPredictor
    ENN_AVAILABLE = True
except ImportError as e:
    print(f"Warning: ENN not available: {e}")
    ENN_AVAILABLE = False

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


class DockingBayesianNetworkWithENN(Node):
    def __init__(self):
        super().__init__('docking_bayesian_network_enn')

        # ============================================
        # CONFIGURATION
        # ============================================
        self.USE_ENN = True  # Set to False to disable ENN and use thresholds
        self.ENN_UNCERTAINTY_THRESHOLD = 0.6  # Fall back to thresholds if uncertainty > this
        self.ENN_WINDOW_SIZE = 20  # Timesteps (2 seconds at 10Hz)
        self.ENN_RATE = 10.0  # Hz - rate for collecting sensor data

        # ============================================
        # LOAD BAYESIAN NETWORK
        # ============================================
        try:
            package_share = get_package_share_directory('rov_risk_assessment')
            self.network_path = os.path.join(package_share, 'config', 'docking_specialized_bn.xdsl')
            self.get_logger().info(f'Network path: {self.network_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to find package: {e}')
            raise

        self.net = pysmile.Network()

        try:
            self.net.read_file(self.network_path)
            self.get_logger().info(f'✓ Loaded Docking BN: {self.network_path}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load docking network: {e}')
            raise

        # ============================================
        # INITIALIZE ENN PREDICTOR
        # ============================================
        self.enn_predictor = None
        self.enn_ready = False

        if self.USE_ENN and ENN_AVAILABLE:
            try:
                model_dir = '/home/badawi/Desktop/PhD_Autumn_2025/paper2/models'
                self.enn_predictor = ENNPredictor(
                    model_path=f'{model_dir}/best_model.pt',
                    norm_path=f'{model_dir}/normalization_params.pt',
                    device='cuda'  # Use 'cpu' if no GPU
                )
                self.get_logger().info('✓ ENN Predictor loaded successfully')
                self.enn_ready = True
            except Exception as e:
                self.get_logger().error(f'❌ Failed to load ENN: {e}')
                self.get_logger().warn('Falling back to threshold-based discretization')
                self.enn_ready = False
        else:
            self.get_logger().info('ENN disabled or not available, using thresholds')

        # ============================================
        # SENSOR DATA STORAGE
        # ============================================
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
            'fatigue': 0.2,
            'stress': 0.2,
        }

        # ============================================
        # ENN FEATURE BUFFER (for temporal inference)
        # ============================================
        # Rolling buffer to store sensor features for ENN
        self.feature_buffer = deque(maxlen=self.ENN_WINDOW_SIZE)

        # Latest raw sensor values for feature extraction
        self.latest_odom = None
        self.latest_imu = None
        self.latest_thrusters = [0.0, 0.0, 0.0, 0.0]
        self.latest_current = 0.0

        # Latest ENN predictions (updated at 10Hz, used by BN at 1Hz)
        self.enn_probs_altitude = None
        self.enn_probs_speed = None
        self.enn_uncertainty_altitude = 1.0
        self.enn_uncertainty_speed = 1.0

        # Pose history for quality assessment
        self.pose_history = []
        self.max_history = 20

        # Mission timer
        self.mission_start_time = self.get_clock().now()

        # ============================================
        # SETUP ROS2
        # ============================================
        self.setup_subscribers()
        self.setup_publishers()

        # Timer for ENN feature collection and inference (10 Hz)
        if self.enn_ready:
            self.create_timer(1.0 / self.ENN_RATE, self.enn_inference_callback)

        # Timer for BN inference (1 Hz)
        self.create_timer(1.0, self.periodic_update)

        self.get_logger().info('✓ Docking BN with ENN initialized')
        if self.enn_ready:
            self.get_logger().info('🧠 Using Evidential Neural Network for Altitude/Speed')
        else:
            self.get_logger().info('📐 Using threshold-based discretization')

    def setup_subscribers(self):
        """Setup ROS2 topic subscribers"""

        # Odometry (for ENN features)
        self.odom_sub = self.create_subscription(
            Odometry, '/blueye/odometry_flu/gt', self.odom_callback, 10)

        # IMU (for ENN features)
        self.imu_sub = self.create_subscription(
            Imu, '/blueye/imu', self.imu_callback, 10)

        # Thrusters (for ENN features)
        self.thruster1_sub = self.create_subscription(
            Float32, '/blueye/thruster_1/cmd_vel',
            lambda msg: self.thruster_callback(msg, 0), 10)
        self.thruster2_sub = self.create_subscription(
            Float32, '/blueye/thruster_2/cmd_vel',
            lambda msg: self.thruster_callback(msg, 1), 10)
        self.thruster3_sub = self.create_subscription(
            Float32, '/blueye/thruster_3/cmd_vel',
            lambda msg: self.thruster_callback(msg, 2), 10)
        self.thruster4_sub = self.create_subscription(
            Float32, '/blueye/thruster_4/cmd_vel',
            lambda msg: self.thruster_callback(msg, 3), 10)

        # Other sensors (original)
        self.usbl_sub = self.create_subscription(
            String, '/blueye/usbl_strength', self.usbl_callback, 10)
        self.camera_sub = self.create_subscription(
            String, '/blueye/camera_quality', self.camera_callback, 10)
        self.battery_sub = self.create_subscription(
            Float32, '/blueye/battery_level', self.battery_callback, 10)
        self.current_sub = self.create_subscription(
            Float32, '/blueye/current', self.current_callback, 10)

        # Docking-specific
        self.aruco_visibility_sub = self.create_subscription(
            String, '/blueye/aruco_visibility', self.aruco_visibility_callback, 10)
        self.docking_detected_sub = self.create_subscription(
            Bool, '/blueye/docking_station_detected', self.docking_detected_callback, 10)
        self.fish_count_sub = self.create_subscription(
            Int32, '/fish_detection/count', self.fish_count_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/blueye/pose_estimated_board_stamped',
            self.pose_callback, 10)

        # Human operator state
        self.fatigue_sub = self.create_subscription(
            Float32, '/blueye/human/fatigue', self.fatigue_callback, 10)
        self.stress_sub = self.create_subscription(
            Float32, '/blueye/human/stress', self.stress_callback, 10)

        self.get_logger().info('✓ All ROS2 subscribers initialized')

    def setup_publishers(self):
        """Setup publishers for docking assessment outputs"""
        self.mode_pub = self.create_publisher(String, '/docking/mode_recommendation', 10)
        self.reliability_pub = self.create_publisher(Float32, '/docking/reliability', 10)
        self.visual_guidance_pub = self.create_publisher(String, '/docking/visual_guidance_quality', 10)
        self.approach_feasibility_pub = self.create_publisher(String, '/docking/approach_feasibility', 10)
        self.mode_auto_pub = self.create_publisher(Float32, '/docking/mode_autonomous_prob', 10)
        self.mode_human_pub = self.create_publisher(Float32, '/docking/mode_human_prob', 10)
        self.mode_shared_pub = self.create_publisher(Float32, '/docking/mode_shared_prob', 10)

        # New: ENN uncertainty publishers
        self.enn_uncertainty_alt_pub = self.create_publisher(Float32, '/docking/enn_uncertainty_altitude', 10)
        self.enn_uncertainty_spd_pub = self.create_publisher(Float32, '/docking/enn_uncertainty_speed', 10)

        self.get_logger().info('✓ Docking output publishers initialized')

    # ============================================
    # ENN FEATURE EXTRACTION AND INFERENCE
    # ============================================

    def extract_enn_features(self):
        """
        Extract feature vector for ENN from latest sensor data.
        Must match the features used during training!

        Returns:
            numpy array of shape (36,) or None if data not ready
        """
        if self.latest_odom is None or self.latest_imu is None:
            return None

        odom = self.latest_odom
        imu = self.latest_imu

        features = []

        # Odometry features (13)
        features.extend([
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
            odom.twist.twist.angular.x,
            odom.twist.twist.angular.y,
            odom.twist.twist.angular.z,
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        ])

        # IMU features (10)
        features.extend([
            imu.orientation.x,
            imu.orientation.y,
            imu.orientation.z,
            imu.orientation.w,
            imu.angular_velocity.x,
            imu.angular_velocity.y,
            imu.angular_velocity.z,
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
        ])

        # Thruster commands (4)
        features.extend(self.latest_thrusters)

        # Current (1)
        features.append(self.latest_current)

        # Derived features (8) - must match training!
        vel_x = odom.twist.twist.linear.x
        vel_y = odom.twist.twist.linear.y
        vel_z = odom.twist.twist.linear.z

        speed_magnitude = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
        horizontal_speed = math.sqrt(vel_x**2 + vel_y**2)
        vertical_speed = abs(vel_z)
        altitude_from_odom = odom.pose.pose.position.z

        total_thruster = sum(abs(t) for t in self.latest_thrusters)
        mean_thruster = total_thruster / 4

        acc_x = imu.linear_acceleration.x
        acc_y = imu.linear_acceleration.y
        acc_z = imu.linear_acceleration.z
        total_acceleration = math.sqrt(acc_x**2 + acc_y**2 + acc_z**2)

        ang_x = imu.angular_velocity.x
        ang_y = imu.angular_velocity.y
        ang_z = imu.angular_velocity.z
        total_angular_velocity = math.sqrt(ang_x**2 + ang_y**2 + ang_z**2)

        features.extend([
            speed_magnitude,
            horizontal_speed,
            vertical_speed,
            altitude_from_odom,
            total_thruster,
            mean_thruster,
            total_acceleration,
            total_angular_velocity,
        ])

        return np.array(features, dtype=np.float32)

    def enn_inference_callback(self):
        """
        Called at 10Hz to collect features and run ENN inference.
        """
        if not self.enn_ready:
            return

        # Extract features
        features = self.extract_enn_features()
        if features is None:
            return

        # Add to buffer
        self.feature_buffer.append(features)

        # Check if buffer is full
        if len(self.feature_buffer) < self.ENN_WINDOW_SIZE:
            return

        # Create window and run inference
        window = np.array(list(self.feature_buffer))

        try:
            probs_alt, probs_spd, unc_alt, unc_spd = self.enn_predictor.predict(window)

            # Store predictions for use by BN
            self.enn_probs_altitude = probs_alt
            self.enn_probs_speed = probs_spd
            self.enn_uncertainty_altitude = unc_alt
            self.enn_uncertainty_speed = unc_spd

            # Publish uncertainties
            unc_alt_msg = Float32()
            unc_alt_msg.data = float(unc_alt)
            self.enn_uncertainty_alt_pub.publish(unc_alt_msg)

            unc_spd_msg = Float32()
            unc_spd_msg.data = float(unc_spd)
            self.enn_uncertainty_spd_pub.publish(unc_spd_msg)

        except Exception as e:
            self.get_logger().error(f'ENN inference failed: {e}')

    # ============================================
    # SENSOR CALLBACKS
    # ============================================

    def odom_callback(self, msg: Odometry):
        """Store latest odometry for ENN"""
        self.latest_odom = msg
        # Also update sensor_data for altitude (used by threshold fallback)
        self.sensor_data['altitude'] = msg.pose.pose.position.z

    def imu_callback(self, msg: Imu):
        """Store latest IMU for ENN"""
        self.latest_imu = msg

    def thruster_callback(self, msg, idx: int):
        """Store thruster commands for ENN"""
        self.latest_thrusters[idx] = msg.data

    def current_callback(self, msg: Float32):
        """Store current for ENN and threshold-based method"""
        self.latest_current = msg.data
        self.sensor_data['current'] = msg.data

        # Set evidence for Current node (still uses thresholds)
        if msg.data < 0.3:
            state = 0  # Low
        elif msg.data < 0.7:
            state = 1  # Medium
        else:
            state = 2  # High

        try:
            self.net.set_evidence("Current", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set Current: {e}')

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

    def aruco_visibility_callback(self, msg: String):
        """Process ArUco marker visibility"""
        visibility = msg.data
        self.sensor_data['aruco_visibility'] = visibility

        state_map = {'All': 0, 'Some': 1, 'None': 2}
        state = state_map.get(visibility, 2)

        try:
            self.net.set_evidence("ArUcoMarkersVisible", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set ArUcoMarkersVisible: {e}')

    def docking_detected_callback(self, msg: Bool):
        """Process docking station detection"""
        detected = msg.data
        self.sensor_data['docking_detected'] = detected

        aruco_vis = self.sensor_data.get('aruco_visibility', 'None')

        if detected and aruco_vis == 'All':
            state = 0  # Detected
        elif detected and aruco_vis == 'Some':
            state = 1  # Partial
        else:
            state = 2  # NotDetected

        try:
            self.net.set_evidence("DockingStationDetection", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set DockingStationDetection: {e}')

    def fish_count_callback(self, msg: Int32):
        """Process fish count for clearance"""
        fish_count = msg.data
        self.sensor_data['fish_count'] = fish_count

        if fish_count == 0:
            state = 0  # Clear
        elif fish_count > 0:
            state = 1  # Obstructed
        else:
            state = 2  # Unknown

        try:
            self.net.set_evidence("DockingClearance", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set DockingClearance: {e}')

    def pose_callback(self, msg: PoseStamped):
        """Process pose estimation"""
        self.sensor_data['pose_x'] = msg.pose.position.x
        self.sensor_data['pose_y'] = msg.pose.position.y
        self.sensor_data['pose_z'] = msg.pose.position.z

        self.pose_history.append({
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
        })

        if len(self.pose_history) > self.max_history:
            self.pose_history.pop(0)

        if len(self.pose_history) >= 10:
            variance = self.calculate_position_variance()

            if variance < 0.05:
                pose_state = 0  # High quality
            elif variance < 0.15:
                pose_state = 1  # Medium quality
            else:
                pose_state = 2  # Low quality

            error_state = 2 - pose_state

            try:
                self.net.set_evidence("PoseEstimationQuality", pose_state)
                self.net.set_evidence("PositionError", error_state)
            except Exception as e:
                self.get_logger().warning(f'Failed to set pose evidence: {e}')

    def fatigue_callback(self, msg: Float32):
        """Process operator fatigue"""
        fatigue = msg.data
        self.sensor_data['fatigue'] = fatigue

        state = self.discretize(fatigue, [0.3, 0.6])

        try:
            self.net.set_evidence("Fatigue", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set Fatigue: {e}')

    def stress_callback(self, msg: Float32):
        """Process operator stress"""
        stress = msg.data
        self.sensor_data['stress'] = stress

        state = self.discretize(stress, [0.3, 0.6])

        try:
            self.net.set_evidence("Stress", state)
        except Exception as e:
            self.get_logger().warning(f'Failed to set Stress: {e}')

    # ============================================
    # BN INFERENCE WITH ENN INTEGRATION
    # ============================================

    def set_altitude_evidence(self):
        """
        Set Altitude evidence using ENN (soft) or thresholds (hard).
        """
        use_enn = (
            self.enn_ready and
            self.enn_probs_altitude is not None and
            self.enn_uncertainty_altitude < self.ENN_UNCERTAINTY_THRESHOLD
        )

        if use_enn:
            # Use ENN soft probabilities
            probs = self.enn_probs_altitude.tolist()
            try:
                self.net.set_virtual_evidence("Altitude", probs)
                self.get_logger().debug(
                    f'Altitude (ENN): probs={probs}, unc={self.enn_uncertainty_altitude:.3f}'
                )
                return 'enn'
            except Exception as e:
                self.get_logger().warning(f'Failed to set virtual evidence for Altitude: {e}')

        # Fallback to threshold-based
        altitude = self.sensor_data.get('altitude')
        if altitude is not None:
            if altitude > 2.0:
                state = 0  # Safe
            elif altitude > 1.0:
                state = 1  # Marginal
            else:
                state = 2  # Unsafe

            try:
                self.net.set_evidence("Altitude", state)
                self.get_logger().debug(f'Altitude (threshold): state={state}')
                return 'threshold'
            except Exception as e:
                self.get_logger().warning(f'Failed to set Altitude evidence: {e}')

        return 'none'

    def set_speed_evidence(self):
        """
        Set Speed evidence using ENN (soft) or thresholds (hard).
        """
        use_enn = (
            self.enn_ready and
            self.enn_probs_speed is not None and
            self.enn_uncertainty_speed < self.ENN_UNCERTAINTY_THRESHOLD
        )

        if use_enn:
            # Use ENN soft probabilities
            probs = self.enn_probs_speed.tolist()
            try:
                self.net.set_virtual_evidence("Speed", probs)
                self.get_logger().debug(
                    f'Speed (ENN): probs={probs}, unc={self.enn_uncertainty_speed:.3f}'
                )
                return 'enn'
            except Exception as e:
                self.get_logger().warning(f'Failed to set virtual evidence for Speed: {e}')

        # Fallback to threshold-based
        if self.latest_odom is not None:
            vel = self.latest_odom.twist.twist.linear
            speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

            if speed < 0.5:
                state = 0  # Safe
            elif speed < 1.0:
                state = 1  # Moderate
            else:
                state = 2  # High

            try:
                self.net.set_evidence("Speed", state)
                self.get_logger().debug(f'Speed (threshold): state={state}')
                return 'threshold'
            except Exception as e:
                self.get_logger().warning(f'Failed to set Speed evidence: {e}')

        return 'none'

    def periodic_update(self):
        """Perform BN inference and publish results (1 Hz)"""
        try:
            # Set Altitude and Speed evidence (ENN or threshold)
            alt_method = self.set_altitude_evidence()
            spd_method = self.set_speed_evidence()

            # Run BN inference
            self.net.update_beliefs()

            # Get results
            docking_rel_probs = self.net.get_node_value("DockingReliability")
            mode_probs = self.net.get_node_value("DockingModeRecommendation")
            mode_states = self.net.get_outcome_ids("DockingModeRecommendation")

            visual_guid_probs = self.net.get_node_value("VisualGuidanceQuality")
            visual_guid_states = self.net.get_outcome_ids("VisualGuidanceQuality")

            approach_feas_probs = self.net.get_node_value("ApproachFeasibility")
            approach_feas_states = self.net.get_outcome_ids("ApproachFeasibility")

            # Calculate expected reliabilities
            reliability_values = [0.975, 0.90, 0.775, 0.60, 0.375]
            expected_docking_rel = sum(p * v for p, v in zip(docking_rel_probs, reliability_values))

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

            # Publish mode probabilities
            auto_msg = Float32()
            auto_msg.data = float(mode_probs[0])
            self.mode_auto_pub.publish(auto_msg)

            human_msg = Float32()
            human_msg.data = float(mode_probs[1])
            self.mode_human_pub.publish(human_msg)

            shared_msg = Float32()
            shared_msg.data = float(mode_probs[2])
            self.mode_shared_pub.publish(shared_msg)

            # Logging
            self.get_logger().info('=' * 70)
            self.get_logger().info('DOCKING BN + ENN RISK ASSESSMENT')
            self.get_logger().info('=' * 70)

            self.get_logger().info(f'\n🧠 EVIDENCE METHOD:')
            self.get_logger().info(f'  Altitude: {alt_method.upper()}' +
                (f' (unc={self.enn_uncertainty_altitude:.3f})' if alt_method == 'enn' else ''))
            self.get_logger().info(f'  Speed: {spd_method.upper()}' +
                (f' (unc={self.enn_uncertainty_speed:.3f})' if spd_method == 'enn' else ''))

            if self.enn_probs_altitude is not None:
                self.get_logger().info(f'\n📊 ENN PROBABILITIES:')
                self.get_logger().info(f'  Altitude: Safe={self.enn_probs_altitude[0]:.3f}, '
                    f'Marginal={self.enn_probs_altitude[1]:.3f}, Unsafe={self.enn_probs_altitude[2]:.3f}')
                self.get_logger().info(f'  Speed: Safe={self.enn_probs_speed[0]:.3f}, '
                    f'Moderate={self.enn_probs_speed[1]:.3f}, High={self.enn_probs_speed[2]:.3f}')

            self.get_logger().info(f'\n🎯 DOCKING ASSESSMENT:')
            self.get_logger().info(f'  Visual Guidance: {visual_quality}')
            self.get_logger().info(f'  Approach Feasibility: {approach_status}')
            self.get_logger().info(f'  Docking Reliability: {expected_docking_rel:.3f}')

            self.get_logger().info(f'\n✨ MODE RECOMMENDATION:')
            self.get_logger().info(f'  Autonomous: {mode_probs[0]:6.2%}')
            self.get_logger().info(f'  Human:      {mode_probs[1]:6.2%}')
            self.get_logger().info(f'  Shared:     {mode_probs[2]:6.2%}')
            self.get_logger().info(f'  ➜ RECOMMENDED: {recommended_mode.upper()} ({mode_confidence:.1%})')

            self.get_logger().info('=' * 70 + '\n')

        except Exception as e:
            self.get_logger().error(f'Periodic update error: {e}')

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

    node = DockingBayesianNetworkWithENN()

    try:
        node.get_logger().info('🚀 Docking BN with ENN is running...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
