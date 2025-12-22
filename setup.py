from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slam_mqtt_server'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # RViz configs
        (os.path.join('share', package_name, 'rviz'), 
            glob(os.path.join('rviz', '*.rviz'))),
        # Config files
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*'))),
    ],
    install_requires=[
        'setuptools',
        'paho-mqtt>=1.6.0',
        'Flask>=2.0.0',
        'PyYAML>=6.0',
        'numpy>=1.24.0',
        'opencv-python>=4.8.0',
        'requests>=2.31.0',
    ],
    zip_safe=True,
    maintainer='ky51301130-jpg',
    maintainer_email='ky51301130@gmail.com',
    description='Pinky Robot Server - MQTT Bridge, Map Builder, AI Vision',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unified_server = slam_mqtt_server.unified_server:main',
            'server_mqtt_bridge = slam_mqtt_server.server_mqtt_bridge:main',
            'nav2_map_builder = slam_mqtt_server.nav2_map_builder:main',
            'ai_vision_analyzer = slam_mqtt_server.ai_vision_analyzer:main',
            'web_rviz_bridge = slam_mqtt_server.web_rviz_bridge:main',
        ],
    },
)
