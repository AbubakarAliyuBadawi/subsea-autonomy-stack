#!/usr/bin/env python3
"""
Environmental Data Publisher — Real Blueye hardware version

Changes from simulator version:
  - Subscribes to /blueye/gps (geometry_msgs/Point, x=lat, y=lon, z=depth)
    instead of /blueye/usbl (PoseStamped with relative position).
  - USBL distance is computed from docking station GPS coordinates using an
    approximate flat-earth projection (accurate within ~1 km).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Point
import requests
import random
import math
from datetime import datetime


# Docking station GPS coordinates (from mission.py)
DOCK_LAT = 63.4414548287786
DOCK_LON = 10.3482882678509


def gps_distance_m(lat1, lon1, lat2, lon2, depth=0.0):
    """Approximate distance in metres between two GPS points (flat-earth)."""
    dlat = (lat2 - lat1) * 111320.0
    dlon = (lon2 - lon1) * 111320.0 * math.cos(math.radians(lat1))
    return math.sqrt(dlat**2 + dlon**2 + depth**2)


class EnvironmentalDataPublisher(Node):
    def __init__(self):
        super().__init__('environmental_data_publisher')

        self.stormglass_api_key = "7c5802fc-d39d-11f0-9b8c-0242ac130003-7c58036a-d39d-11f0-9b8c-0242ac130003"

        # Location used for Stormglass API query (updated live from GPS)
        self.lat = DOCK_LAT
        self.lon = DOCK_LON

        # Publishers
        self.current_pub = self.create_publisher(Float32, '/blueye/current', 10)
        self.wind_pub = self.create_publisher(Float32, '/blueye/wind', 10)
        self.waves_pub = self.create_publisher(Float32, '/blueye/waves', 10)
        self.usbl_strength_pub = self.create_publisher(String, '/blueye/usbl_strength', 10)

        # Subscribe to GPS position from blueye_telemetry.py
        self.gps_sub = self.create_subscription(
            Point, '/blueye/gps', self.gps_callback, 10)

        self.stormglass_url = 'https://api.stormglass.io/v2/weather/point'

        # Cached environmental values
        self.wind = 5.0
        self.current = 0.2
        self.waves = 0.5

        # USBL strength state
        self.last_distance = None
        self.multipath_probability = 0.1

        # API call counter
        self.api_calls_today = 0
        self.last_api_call_date = datetime.now().date()

        # Fetch initial environmental data
        self.fetch_and_publish()

        self.create_timer(1800.0, self.fetch_and_publish)
        self.create_timer(1.0, self.publish_environmental_data)

        self.get_logger().info('Environmental Data Publisher started (real Blueye)')
        self.get_logger().info(f'Docking station reference: ({DOCK_LAT:.6f}, {DOCK_LON:.6f})')

    # -------------------------------------------------------------------------
    # GPS callback — replaces USBL PoseStamped callback
    # -------------------------------------------------------------------------

    def gps_callback(self, msg: Point):
        """Calculate USBL signal strength from GPS distance to docking station."""
        lat = msg.x
        lon = msg.y
        depth = msg.z

        # Update API query location to current drone position
        self.lat = lat
        self.lon = lon

        distance = gps_distance_m(lat, lon, DOCK_LAT, DOCK_LON, depth)

        snr = self.calculate_snr(distance)
        strength = self.snr_to_strength(snr)

        # Degrade on sudden position jumps (wildpoint detection)
        if self.last_distance is not None:
            if abs(distance - self.last_distance) > 10.0:
                strength = 'Weak'

        self.last_distance = distance

        strength_msg = String()
        strength_msg.data = strength
        self.usbl_strength_pub.publish(strength_msg)

    # -------------------------------------------------------------------------
    # SNR / strength calculation (unchanged from simulator version)
    # -------------------------------------------------------------------------

    def calculate_snr(self, distance):
        transmit_power = 185.0
        if distance < 1.0:
            distance = 1.0
        geometric_loss = 20 * math.log10(distance)

        f = 25.0
        absorption_coef = (0.11 * f**2) / (1 + f**2) + 44 * f**2 / (4100 + f**2)
        absorption_loss = absorption_coef * (distance / 1000.0)

        base_noise = 50.0
        current_noise = self.current * 3.0
        wind_noise = self.wind * 1.5
        wave_noise = self.waves * 2.0
        environmental_noise = base_noise + current_noise + wind_noise + wave_noise

        multipath = 0.0
        if random.random() < self.multipath_probability:
            if random.random() < 0.3:
                multipath = random.uniform(3, 6)
            else:
                multipath = random.uniform(-10, -3)

        signal = transmit_power - geometric_loss - absorption_loss + multipath
        return signal - environmental_noise

    def snr_to_strength(self, snr):
        if snr > 20:
            return 'Strong'
        elif snr > 12:
            return 'Moderate'
        elif snr > 6:
            return 'Weak'
        else:
            return 'Lost'

    # -------------------------------------------------------------------------
    # Stormglass API (unchanged)
    # -------------------------------------------------------------------------

    def fetch_and_publish(self):
        today = datetime.now().date()
        if today != self.last_api_call_date:
            self.api_calls_today = 0
            self.last_api_call_date = today

        if self.api_calls_today >= 45:
            self.get_logger().warning('Approaching Stormglass API limit, using cached values')
            return

        try:
            params = {
                'lat': self.lat,
                'lng': self.lon,
                'params': 'waveHeight,currentSpeed,windSpeed'
            }
            headers = {'Authorization': self.stormglass_api_key}
            response = requests.get(self.stormglass_url, params=params, headers=headers, timeout=15)

            if response.status_code == 200:
                data = response.json()
                current_hour = data['hours'][0]
                self.api_calls_today += 1

                wind_sources = ['noaa', 'sg', 'meteo', 'icon', 'dwd']
                self.wind = self.get_best_value(current_hour, 'windSpeed', wind_sources, self.wind)

                current_sources = ['noaa', 'sg', 'meteo']
                self.current = self.get_best_value(current_hour, 'currentSpeed', current_sources, self.current)

                wave_sources = ['noaa', 'sg', 'meteo', 'icon', 'dwd']
                self.waves = self.get_best_value(current_hour, 'waveHeight', wave_sources, self.waves)

                self.get_logger().info(
                    f'Marine data updated — wind: {self.wind:.2f} m/s, '
                    f'current: {self.current:.3f} m/s, waves: {self.waves:.2f} m'
                )
            else:
                self.get_logger().warning(f'Stormglass API error: {response.status_code}')

        except Exception as e:
            self.get_logger().warning(f'Stormglass fetch failed: {e}')

    def get_best_value(self, hour_data, param_name, sources, default):
        if param_name not in hour_data:
            return default
        param_data = hour_data[param_name]
        for source in sources:
            if source in param_data and param_data[source] is not None:
                return param_data[source]
        for value in param_data.values():
            if value is not None:
                return value
        return default

    def publish_environmental_data(self):
        current = max(0.0, self.current + random.uniform(-0.005, 0.005))
        wind = max(0.0, self.wind + random.uniform(-0.1, 0.1))
        waves = max(0.0, self.waves + random.uniform(-0.02, 0.02))

        msg = Float32()
        msg.data = current
        self.current_pub.publish(msg)

        msg.data = wind
        self.wind_pub.publish(msg)

        msg.data = waves
        self.waves_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = EnvironmentalDataPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
