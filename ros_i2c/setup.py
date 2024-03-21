from setuptools import setup
import os
from glob import glob

package_name = 'ros_i2c'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Valentin Schröter',
    maintainer_email='vasc9380@th-wildau.de',
    description='ROS2 nodes for low level hardware interfaces.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge = ros_i2c.bridge:main',
        ],
    },
)
