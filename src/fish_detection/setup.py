from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'fish_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Badawi',
    maintainer_email='abubakar.a.badawi@ntnu.no',
    description='YOLO-based ROS 2 fish detection node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fish_detection = fish_detection.fish_detection_node:main',
        ],
    },
)
