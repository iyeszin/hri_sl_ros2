#!/usr/bin/env python
from setuptools import setup
import os
from glob import glob

package_name = 'rh8d_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cpsfeith',
    maintainer_email='nikolaus.feith@unileoben.ac.at',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rh8d_collector_node = rh8d_ros2.rh8d_collector:main'
        ],
    },
)
