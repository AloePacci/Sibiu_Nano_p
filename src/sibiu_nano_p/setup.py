from setuptools import setup
import os
from glob import glob

package_name = 'sibiu_nano_p'
submodule = 'sibiu_nano_p/submodulos'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodule],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, "config"), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alejandro',
    maintainer_email='alexcpesp@gmail.com',
    description='Working with Sibiu Nano + trying to make it an ASV',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone = sibiu_nano_p.dronekit_node:main',
            'mqtt = sibiu_nano_p.mqtt_node:main',
            'mission = sibiu_nano_p.mission_node:main',
            'sensors = sibiu_nano_p.sensor_module_node:main',
            'planner = sibiu_nano_p.planner_node:main',
            'watchdog = sibiu_nano_p.watchdog_node:main',
            'camera = sibiu_nano_p.camera_node:main',
        ],
    },
)
