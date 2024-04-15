import os
from glob import glob
from setuptools import setup


package_name = 'ros_sign_language_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,
              package_name+"/weight"],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iyeszin',
    maintainer_email='iyeszin@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inference_node = ros_sign_language_recognition.inference:main',
            'ui_inf_node = ros_sign_language_recognition.ui_inf:main',
        ],
    },
)
