from setuptools import find_packages, setup

package_name = 'smart_house_interfaces'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mihajlo',
    maintainer_email='mihastevanovic04@gmail.com',
    description='Python client server',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [ 'lock = smart_house_interfaces.service_lock_function:main',
                            'alarm = smart_house_interfaces.service_alarm_function:main',
        ],
    },
)
