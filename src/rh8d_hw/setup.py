from setuptools import setup
import os
from glob import glob

package_name = 'rh8d_hw'
dynamixel_sdk = package_name + "/dynamixel_sdk"


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, dynamixel_sdk],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

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
            'rh8d_hand_node = rh8d_hw.rh8d_hand_node:main',
            'sensor_node = rh8d_hw.sensor_reading_publishing_node:main',
            'rh8d_tactile_combined = rh8d_hw.rh8d_tactile_combined_node:main',
        ],
    },
)
