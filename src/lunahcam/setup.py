from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'lunahcam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share/', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='egsa',
    maintainer_email='egsa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],  # Use tests_require for test dependencies
    entry_points={
        'console_scripts': [
            'telemetry_data = lunahcam.telemetry_data:main',
            'bus_interface = lunahcam.bus_interface:main',
            'cmd_handler = lunahcam.cmd_handler:main',
            'pre_shutdown = lunahcam.pre_shutdown:main',
            'payload = lunahcam.payload:main',
            'thermal_control = lunahcam.thermal_control:main'
        ],
    },
)
