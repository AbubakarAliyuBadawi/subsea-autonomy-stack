from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'rov_risk_assessment'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.xdsl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Badawi',
    maintainer_email='abubakar.a.badawi@ntnu.no',
    description='Bayesian Network-based risk assessment for ROV missions',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_control_bn = rov_risk_assessment.mission_control_bn_node:main',
            'weather_api = rov_risk_assessment.weather_api:main',
            'battery_level = rov_risk_assessment.battey_level:main',
            'speed = rov_risk_assessment.speed:main',
            'altitude = rov_risk_assessment.altitude:main',
            'camera_quality = rov_risk_assessment.camera_quality:main',
            'turbidity = rov_risk_assessment.turbidity:main',
            'trust = rov_risk_assessment.trust:main',
            'test_publishers = rov_risk_assessment.test_ros_publishers:main',
            'test_usbl = rov_risk_assessment.test_usbl:main',
        ],
    },
)