#!/usr/bin/env python3
"""
Environmental Data Publisher - Real marine data from Stormglass.io
Publishes REAL ocean currents, waves, wind AND USBL signal strength
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped
import requests
import random
import math
from datetime import datetime


class EnvironmentalDataPublisher(Node):
    def __init__(self):
        super().__init__('environmental_data_publisher')
        
        self.stormglass_api_key = "7c5802fc-d39d-11f0-9b8c-0242ac130003-7c58036a-d39d-11f0-9b8c-0242ac130003"
        
        # Location: Trondheim, Norway (NTNU)
        self.lat = 63.4305
        self.lon = 10.3951
        
        # Publishers for environmental nodes in BN
        self.current_pub = self.create_publisher(Float32, '/blueye/current', 10)
        self.wind_pub = self.create_publisher(Float32, '/blueye/wind', 10)
        self.waves_pub = self.create_publisher(Float32, '/blueye/waves', 10)
        self.usbl_strength_pub = self.create_publisher(String, '/blueye/usbl_strength', 10)  # NEW!
        
        # Subscribe to USBL position for strength calculation
        self.usbl_sub = self.create_subscription(
            PoseStamped, '/blueye/usbl', self.usbl_callback, 10)
        
        # Stormglass API configuration
        self.stormglass_url = 'https://api.stormglass.io/v2/weather/point'
        
        # Cached environmental values
        self.wind = 5.0
        self.current = 0.2
        self.waves = 0.5
        
        # USBL strength calculation state
        self.last_distance = None
        self.multipath_probability = 0.1
        
        # API call counter
        self.api_calls_today = 0
        self.last_api_call_date = datetime.now().date()
        
        # Check API key
        if self.stormglass_api_key == "YOUR_STORMGLASS_API_KEY_HERE":
            self.get_logger().error('❌ STORMGLASS API KEY NOT CONFIGURED!')
            raise ValueError("Stormglass API key not configured")
        
        # Fetch initial data
        self.fetch_and_publish()
        
        # Timer: Fetch from API every 30 minutes
        self.create_timer(1800.0, self.fetch_and_publish)
        
        # Timer: Publish environmental data at 1 Hz
        self.create_timer(1.0, self.publish_environmental_data)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('✓ Environmental Data Publisher started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'📍 Location: Trondheim ({self.lat}, {self.lon})')
        self.get_logger().info('🌊 Data source: Stormglass.io (REAL marine data)')
        self.get_logger().info('📡 USBL Strength: Physics-based calculation')
        self.get_logger().info('🔄 Update interval: 30 minutes')
        self.get_logger().info('=' * 60)
    
    # ========================================================================
    # USBL STRENGTH CALCULATION (NEW)
    # ========================================================================
    
    def calculate_snr(self, distance):
        """
        Calculate Signal-to-Noise Ratio for USBL
        Considers distance, environmental conditions, and multipath
        """
        # Transmit power (typical USBL)
        transmit_power = 185.0  # dB re 1µPa
        
        # Geometric spreading loss: 20*log10(distance)
        if distance < 1.0:
            distance = 1.0
        geometric_loss = 20 * math.log10(distance)
        
        # Absorption loss (frequency-dependent, using 25 kHz)
        f = 25.0  # kHz
        absorption_coef = (0.11 * f**2) / (1 + f**2) + 44 * f**2 / (4100 + f**2)
        absorption_loss = absorption_coef * (distance / 1000.0)  # distance in km
        
        # Environmental noise (affected by current, wind, waves)
        base_noise = 50.0  # dB
        current_noise = self.current * 3.0  # ~3dB per m/s
        wind_noise = self.wind * 1.5  # Wind creates surface noise
        wave_noise = self.waves * 2.0  # Wave action creates noise
        environmental_noise = base_noise + current_noise + wind_noise + wave_noise
        
        # Multipath effect (random interference)
        multipath = 0.0
        if random.random() < self.multipath_probability:
            if random.random() < 0.3:
                multipath = random.uniform(3, 6)  # Constructive
            else:
                multipath = random.uniform(-10, -3)  # Destructive
        
        # Calculate SNR
        signal = transmit_power - geometric_loss - absorption_loss + multipath
        snr = signal - environmental_noise
        
        return snr
    
    def snr_to_strength(self, snr):
        """Convert SNR to categorical strength"""
        if snr > 20:
            return "Strong"
        elif snr > 12:
            return "Moderate"
        elif snr > 6:
            return "Weak"
        else:
            return "Lost"
    
    def usbl_callback(self, msg: PoseStamped):
        """Calculate USBL strength when position updates"""
        # Calculate distance from docking station
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        distance = math.sqrt(x**2 + y**2 + z**2)
        
        # Calculate SNR considering environmental conditions
        snr = self.calculate_snr(distance)
        
        # Convert to strength
        strength = self.snr_to_strength(snr)
        
        # Detect wildpoints (sudden position jumps)
        if self.last_distance is not None:
            position_jump = abs(distance - self.last_distance)
            if position_jump > 10.0:  # >10m jump = suspicious
                strength = "Weak"  # Degrade due to likely bad measurement
        
        self.last_distance = distance
        
        # Publish
        strength_msg = String()
        strength_msg.data = strength
        self.usbl_strength_pub.publish(strength_msg)
    
    # ========================================================================
    # ENVIRONMENTAL DATA (EXISTING)
    # ========================================================================
    
    def fetch_and_publish(self):
        """Fetch REAL marine data from Stormglass API"""
        
        # Reset daily counter
        today = datetime.now().date()
        if today != self.last_api_call_date:
            self.api_calls_today = 0
            self.last_api_call_date = today
        
        # Check quota
        if self.api_calls_today >= 45:
            self.get_logger().warning('⚠️  Approaching API limit, using cached values')
            return
        
        try:
            params = {
                'lat': self.lat,
                'lng': self.lon,
                'params': 'waveHeight,currentSpeed,windSpeed'
            }
            headers = {'Authorization': self.stormglass_api_key}
            
            response = requests.get(
                self.stormglass_url, 
                params=params, 
                headers=headers, 
                timeout=15
            )
            
            if response.status_code == 200:
                data = response.json()
                current_hour = data['hours'][0]
                self.api_calls_today += 1
                
                # Extract environmental data
                wind_sources = ['noaa', 'sg', 'meteo', 'icon', 'dwd']
                self.wind = self.get_best_value(current_hour, 'windSpeed', wind_sources, self.wind)
                
                current_sources = ['noaa', 'sg', 'meteo']
                self.current = self.get_best_value(current_hour, 'currentSpeed', current_sources, self.current)
                
                wave_sources = ['noaa', 'sg', 'meteo', 'icon', 'dwd']
                self.waves = self.get_best_value(current_hour, 'waveHeight', wave_sources, self.waves)
                
                self.get_logger().info('=' * 60)
                self.get_logger().info('🌊 REAL MARINE DATA UPDATED')
                self.get_logger().info('=' * 60)
                self.get_logger().info(f'Wind:    {self.wind:.2f} m/s')
                self.get_logger().info(f'Current: {self.current:.3f} m/s')
                self.get_logger().info(f'Waves:   {self.waves:.2f} m')
                self.get_logger().info(f'API calls: {self.api_calls_today}/50')
                self.get_logger().info('=' * 60)
                
            else:
                self.get_logger().warning(f'⚠️  API error: {response.status_code}')
                
        except Exception as e:
            self.get_logger().warning(f'⚠️  Error: {e}')
    
    def get_best_value(self, hour_data, param_name, sources, default):
        """Extract best value from multiple sources"""
        if param_name not in hour_data:
            return default
        
        param_data = hour_data[param_name]
        
        for source in sources:
            if source in param_data and param_data[source] is not None:
                return param_data[source]
        
        for key, value in param_data.items():
            if value is not None:
                return value
        
        return default
    
    def publish_environmental_data(self):
        """Publish environmental data with small variations"""
        
        # Add small noise
        current = self.current + random.uniform(-0.005, 0.005)
        wind = self.wind + random.uniform(-0.1, 0.1)
        waves = self.waves + random.uniform(-0.02, 0.02)
        
        # Publish
        current_msg = Float32()
        current_msg.data = max(0.0, current)
        self.current_pub.publish(current_msg)
        
        wind_msg = Float32()
        wind_msg.data = max(0.0, wind)
        self.wind_pub.publish(wind_msg)
        
        waves_msg = Float32()
        waves_msg.data = max(0.0, waves)
        self.waves_pub.publish(waves_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = EnvironmentalDataPublisher()
        rclpy.spin(node)
    except ValueError as e:
        print(f"\n❌ Error: {e}\n")
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()