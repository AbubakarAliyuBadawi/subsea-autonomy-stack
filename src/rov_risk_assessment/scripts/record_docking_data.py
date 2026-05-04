#!/usr/bin/env python3
"""
Docking Bayesian Network - Data Recorder

Records real-time data from docking BN topics for later analysis and plotting.

Usage:
    # Record for 5 minutes
    python3 record_docking_data.py --duration 300 --output docking_nominal.json
    
    # Record until Ctrl+C
    python3 record_docking_data.py --output docking_experiment.json
    
    # Then plot the results
    python3 plot_docking_results.py --data docking_experiment.json

Author: Badawi - PhD Research
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32, Bool
import json
import time
from datetime import datetime
import signal
import sys


class DockingDataRecorder(Node):
    def __init__(self, duration=None, output_file='docking_data.json'):
        super().__init__('docking_data_recorder')
        
        self.duration = duration
        self.output_file = output_file
        self.start_time = time.time()
        
        # Data storage
        self.data = {
            'timestamps': [],
            'time': [],
            'mode_autonomous': [],
            'mode_human': [],
            'mode_shared': [],
            'docking_reliability': [],
            'visual_guidance': [],
            'approach_feasibility': [],
            'aruco_visibility': [],
            'docking_detected': [],
            'fish_count': [],
            'camera_quality': [],
            'usbl_strength': [],
            'battery_level': [],
            'current': [],
            'fatigue': [],
            'stress': [],
            'autonomous_reliability': [],
            'human_reliability': [],
            'situational_awareness': [],
        }
        
        # Latest values
        self.latest = {}
        
        # Create subscribers for all relevant topics
        self.create_subscriptions()
        
        # Timer for periodic data recording (10 Hz)
        self.create_timer(0.1, self.record_data)
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('📊 DOCKING DATA RECORDER STARTED')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Output file: {output_file}')
        if duration:
            self.get_logger().info(f'  Duration: {duration} seconds')
        else:
            self.get_logger().info(f'  Duration: Until Ctrl+C')
        self.get_logger().info('  Recording at 10 Hz')
        self.get_logger().info('=' * 60)
    
    def create_subscriptions(self):
        """Subscribe to all docking BN topics"""
        
        # Mode recommendation (need to parse string with probabilities)
        self.mode_sub = self.create_subscription(
            String, '/docking/mode_recommendation', self.mode_callback, 10)
        
        # Docking outputs
        self.reliability_sub = self.create_subscription(
            Float32, '/docking/reliability', 
            lambda msg: self.store_value('docking_reliability', msg.data), 10)
        
        self.visual_sub = self.create_subscription(
            String, '/docking/visual_guidance_quality',
            lambda msg: self.store_value('visual_guidance', msg.data), 10)
        
        self.approach_sub = self.create_subscription(
            String, '/docking/approach_feasibility',
            lambda msg: self.store_value('approach_feasibility', msg.data), 10)
        
        # Sensor inputs
        self.aruco_sub = self.create_subscription(
            String, '/blueye/aruco_visibility',
            lambda msg: self.store_value('aruco_visibility', msg.data), 10)
        
        self.docking_det_sub = self.create_subscription(
            Bool, '/blueye/docking_station_detected',
            lambda msg: self.store_value('docking_detected', msg.data), 10)
        
        self.fish_sub = self.create_subscription(
            Int32, '/fish_detection/count',
            lambda msg: self.store_value('fish_count', msg.data), 10)
        
        self.camera_sub = self.create_subscription(
            String, '/blueye/camera_quality',
            lambda msg: self.store_value('camera_quality', msg.data), 10)
        
        self.usbl_sub = self.create_subscription(
            String, '/blueye/usbl_strength',
            lambda msg: self.store_value('usbl_strength', msg.data), 10)
        
        self.battery_sub = self.create_subscription(
            Float32, '/blueye/battery_level',
            lambda msg: self.store_value('battery_level', msg.data), 10)
        
        self.current_sub = self.create_subscription(
            Float32, '/blueye/current',
            lambda msg: self.store_value('current', msg.data), 10)
        
        # Human operator state
        self.fatigue_sub = self.create_subscription(
            Float32, '/blueye/human/fatigue',
            lambda msg: self.store_value('fatigue', msg.data), 10)
        
        self.stress_sub = self.create_subscription(
            Float32, '/blueye/human/stress',
            lambda msg: self.store_value('stress', msg.data), 10)
        
        self.get_logger().info('✓ Subscribed to all docking topics')
    
    def mode_callback(self, msg: String):
        """Parse mode recommendation (just store the string for now)"""
        self.store_value('mode_recommendation', msg.data)
    
    def store_value(self, key, value):
        """Store latest value"""
        self.latest[key] = value
    
    def record_data(self):
        """Record current data snapshot"""
        elapsed = time.time() - self.start_time
        
        # Check if we should stop
        if self.duration and elapsed > self.duration:
            self.get_logger().info(f'\n⏱️  Duration reached ({self.duration}s)')
            self.save_and_exit()
            return
        
        # Record timestamp
        self.data['timestamps'].append(datetime.now().isoformat())
        self.data['time'].append(elapsed)
        
        # Record all values (use None if not available yet)
        for key in self.data.keys():
            if key not in ['timestamps', 'time']:
                value = self.latest.get(key, None)
                self.data[key].append(value)
        
        # Log progress every 10 seconds
        if int(elapsed) % 10 == 0 and int(elapsed) > 0:
            samples = len(self.data['timestamps'])
            self.get_logger().info(
                f'📊 Recording... {int(elapsed)}s elapsed, {samples} samples'
            )
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        self.get_logger().info('\n\n⚠️  Ctrl+C detected')
        self.save_and_exit()
    
    def save_and_exit(self):
        """Save data and exit"""
        samples = len(self.data['timestamps'])
        
        if samples == 0:
            self.get_logger().warn('No data recorded!')
            sys.exit(0)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('💾 SAVING DATA')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Samples: {samples}')
        self.get_logger().info(f'  Duration: {self.data["time"][-1]:.1f}s')
        self.get_logger().info(f'  Output: {self.output_file}')
        
        # Convert to JSON-serializable format
        json_data = {}
        for key, values in self.data.items():
            json_data[key] = [
                float(v) if isinstance(v, (int, float)) else 
                str(v) if v is not None else None 
                for v in values
            ]
        
        # Save to file
        with open(self.output_file, 'w') as f:
            json.dump(json_data, f, indent=2)
        
        self.get_logger().info(f'✅ Data saved to {self.output_file}')
        self.get_logger().info('=' * 60)
        self.get_logger().info('\nTo plot results:')
        self.get_logger().info(f'  python3 plot_docking_results.py --data {self.output_file}')
        self.get_logger().info('=' * 60)
        
        sys.exit(0)


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Record docking BN data')
    parser.add_argument('--duration', type=int, default=None,
                       help='Recording duration in seconds (default: until Ctrl+C)')
    parser.add_argument('--output', type=str, default='docking_data.json',
                       help='Output JSON file')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    recorder = DockingDataRecorder(
        duration=args.duration,
        output_file=args.output
    )
    
    try:
        rclpy.spin(recorder)
    except SystemExit:
        pass
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
