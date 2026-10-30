import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ekf_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mihajlo Stevanovic',
    maintainer_email='sm220315d@student.etf.bg.ac.rs',
    description='EKF line-feature localization for TurtleBot3 (ROS 2)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ekf_line_localization = ekf_localization.ekf_line_localization:main',
            'split_and_merge_node = ekf_localization.split_and_merge:main',
            'waypoint_mission = ekf_localization.waypoint_mission:main',
        ],
    },
)
