import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ipex_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(
		os.path.join('share', package_name, 'launch'),
		glob('launch/*.launch.py')
	),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='graco',
    maintainer_email='graco@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'drive_straight_server = ipex_motion.drive_straight_server:main',
		'set_arm_angles_server = ipex_motion.set_arm_angles_server:main',
		'set_drum_speeds_server = ipex_motion.set_drum_speeds_server:main',
		'fake_payload_sensor = ipex_motion.fake_payload_sensor:main',
		'excavate_server = ipex_motion.excavate_server:main',
		'deposit_server = ipex_motion.deposit_server:main',
		'cmd_vel_bridge = ipex_motion.cmd_vel_bridge:main',
		'controller = ipex_motion.controller:main',
        ],
    },
)
