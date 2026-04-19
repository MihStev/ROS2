from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'smart_house'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mihajlo',
    maintainer_email='mihajlo@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'temp_sensor = smart_house.temp_sensor:main',
            'humidity_sensor = smart_house.humidity_sensor:main',
            'light_sensor = smart_house.light_sensor:main',
            'system_visualization = smart_house.system_visualization:main',
        ],
    },
)
