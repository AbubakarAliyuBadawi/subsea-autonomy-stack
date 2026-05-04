from setuptools import setup
import os
from glob import glob

package_name = 'blueye_risk_assessment'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xdsl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Badawi',
    maintainer_email='abubakar.a.badawi@ntnu.no',
    description='Real-hardware Bayesian Network risk assessment for Blueye ROV docking',
    license='MIT',
    entry_points={
        'console_scripts': [
            'docking_bn      = blueye_risk_assessment.docking_bn_node:main',
            'weather_api     = blueye_risk_assessment.weather_api:main',
            'battery_level   = blueye_risk_assessment.battery_level:main',
            'camera_quality  = blueye_risk_assessment.camera_quality:main',
            'operator_state  = blueye_risk_assessment.operator_state_publisher:main',
            'turbidity       = blueye_risk_assessment.turbidity:main',
        ],
    },
)
