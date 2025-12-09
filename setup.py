from setuptools import setup
import os
from glob import glob

package_name = 'arms_wifi_py_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 package for bilateral teleoperation of Trossen robotic arms with force feedback over WiFi',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leader_node = arms_wifi_py_bridge.leader_node:main',
            'follower_node = arms_wifi_py_bridge.follower_node:main',
        ],
    },
)