#!/usr/bin/env python3
"""
Environmental Data Publisher - Real marine data from Stormglass.io
Publishes REAL ocean currents, waves, and wind to /blueye/* topics
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import requests
import random
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
        
        # Stormglass API configuration
        self.stormglass_url = 'https://api.stormglass.io/v2/weather/point'
        
        # Cached environmental values (fallback if API fails)
        self.wind = 5.0
        self.current = 0.2
        self.waves = 0.5
        
        # API call counter (for monitoring free tier limit: 50/day)
        self.api_calls_today = 0
        self.last_api_call_date = datetime.now().date()
        
        # Check if API key is configured
        if self.stormglass_api_key == "YOUR_STORMGLASS_API_KEY_HERE":
            self.get_logger().error('=' * 60)
            self.get_logger().error('❌ STORMGLASS API KEY NOT CONFIGURED!')
            self.get_logger().error('=' * 60)
            self.get_logger().error('Please set your API key in the code:')
            self.get_logger().error('  self.stormglass_api_key = "your_actual_key"')
            self.get_logger().error('')
            self.get_logger().error('Get your key from:')
            self.get_logger().error('  https://stormglass.io/dashboard')
            self.get_logger().error('=' * 60)
            raise ValueError("Stormglass API key not configured")
        
        # Fetch initial data
        self.fetch_and_publish()
        
        # Timer: Fetch from API every 30 minutes (1800 seconds)
        # 30 min interval = max 48 calls/day (within free tier of 50/day)
        self.create_timer(1800.0, self.fetch_and_publish)
        
        # Timer: Publish at 1 Hz with small variations
        self.create_timer(1.0, self.publish_with_variations)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('✓ Environmental Data Publisher started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'📍 Location: Trondheim ({self.lat}, {self.lon})')
        self.get_logger().info('🌊 Data source: Stormglass.io (REAL marine data)')
        self.get_logger().info('🔄 Update interval: 30 minutes')
        self.get_logger().info('📊 Free tier limit: 50 calls/day')
        self.get_logger().info('=' * 60)
    
    def fetch_and_publish(self):
        """Fetch REAL marine data from Stormglass API"""
        
        # Reset daily counter if it's a new day
        today = datetime.now().date()
        if today != self.last_api_call_date:
            self.api_calls_today = 0
            self.last_api_call_date = today
            self.get_logger().info('🔄 New day - API call counter reset')
        
        # Check if we're approaching daily limit
        if self.api_calls_today >= 45:
            self.get_logger().warning('⚠️  Approaching daily API limit (50 calls/day)')
            self.get_logger().warning('   Using cached values to preserve quota')
            return
        
        try:
            # Request parameters
            params = {
                'lat': self.lat,
                'lng': self.lon,
                'params': 'waveHeight,currentSpeed,windSpeed'
            }
            
            headers = {
                'Authorization': self.stormglass_api_key
            }
            
            self.get_logger().info('📡 Fetching data from Stormglass...')
            
            response = requests.get(
                self.stormglass_url, 
                params=params, 
                headers=headers, 
                timeout=15
            )
            
            if response.status_code == 200:
                data = response.json()
                current_hour = data['hours'][0]  # Current hour data
                
                # Increment API call counter
                self.api_calls_today += 1
                
                # Extract data from available sources
                # Stormglass provides multiple sources (noaa, sg, meteo, etc.)
                # We'll try to get the best available
                
                # Get Wind Speed (m/s)
                wind_sources = ['noaa', 'sg', 'meteo', 'icon', 'dwd']
                self.wind = self.get_best_value(current_hour, 'windSpeed', wind_sources, 5.0)
                
                # Get Current Speed (m/s) - REAL ocean current
                current_sources = ['noaa', 'sg', 'meteo']
                self.current = self.get_best_value(current_hour, 'currentSpeed', current_sources, 0.2)
                
                # Get Wave Height (m) - REAL wave data
                wave_sources = ['noaa', 'sg', 'meteo', 'icon', 'dwd']
                self.waves = self.get_best_value(current_hour, 'waveHeight', wave_sources, 0.5)
                
                self.get_logger().info('=' * 60)
                self.get_logger().info('🌊 REAL MARINE DATA UPDATED (Stormglass)')
                self.get_logger().info('=' * 60)
                self.get_logger().info(f'Wind Speed:    {self.wind:.2f} m/s')
                self.get_logger().info(f'Current Speed: {self.current:.3f} m/s (REAL ocean current)')
                self.get_logger().info(f'Wave Height:   {self.waves:.2f} m (REAL wave data)')
                self.get_logger().info(f'API calls today: {self.api_calls_today}/50')
                self.get_logger().info('=' * 60)
                
            elif response.status_code == 401:
                self.get_logger().error('❌ AUTHENTICATION FAILED')
                self.get_logger().error('   Your API key is invalid or expired')
                self.get_logger().error('   Check your key at: https://stormglass.io/dashboard')
                
            elif response.status_code == 402:
                self.get_logger().error('❌ QUOTA EXCEEDED')
                self.get_logger().error('   You have exceeded your daily limit (50 calls/day)')
                self.get_logger().error('   Using cached values...')
                
            else:
                self.get_logger().warning(f'⚠️  API returned status {response.status_code}')
                self.get_logger().warning(f'   Response: {response.text[:200]}')
                
        except requests.exceptions.Timeout:
            self.get_logger().warning('⚠️  API request timed out')
            self.get_logger().warning('   Using cached values...')
            
        except requests.exceptions.RequestException as e:
            self.get_logger().warning(f'⚠️  Network error: {e}')
            self.get_logger().warning('   Using cached values...')
            
        except Exception as e:
            self.get_logger().warning(f'⚠️  Unexpected error: {e}')
            self.get_logger().warning('   Using cached values...')
    
    def get_best_value(self, hour_data, param_name, sources, default):
        """
        Extract the best available value from multiple data sources
        
        Args:
            hour_data: Data for current hour from API
            param_name: Parameter name (e.g., 'windSpeed', 'currentSpeed')
            sources: List of source names to try (e.g., ['noaa', 'sg'])
            default: Default value if no source available
            
        Returns:
            Best available value or default
        """
        if param_name not in hour_data:
            return default
        
        param_data = hour_data[param_name]
        
        # Try each source in order of preference
        for source in sources:
            if source in param_data and param_data[source] is not None:
                return param_data[source]
        
        # If no preferred source, return first available
        for key, value in param_data.items():
            if value is not None:
                return value
        
        return default
    
    def publish_with_variations(self):
        """Publish environmental data with small random variations"""
        
        # Add small noise to make data more realistic between API updates
        current = self.current + random.uniform(-0.005, 0.005)
        wind = self.wind + random.uniform(-0.1, 0.1)
        waves = self.waves + random.uniform(-0.02, 0.02)
        
        # Publish Current (m/s)
        current_msg = Float32()
        current_msg.data = max(0.0, current)
        self.current_pub.publish(current_msg)
        
        # Publish Wind (m/s)
        wind_msg = Float32()
        wind_msg.data = max(0.0, wind)
        self.wind_pub.publish(wind_msg)
        
        # Publish Waves (m)
        waves_msg = Float32()
        waves_msg.data = max(0.0, waves)
        self.waves_pub.publish(waves_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = EnvironmentalDataPublisher()
        rclpy.spin(node)
    except ValueError as e:
        print(f"\n❌ Configuration Error: {e}\n")
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()