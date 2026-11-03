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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='graco',
    maintainer_email='graco@todo.todo',
    description='Roxey teleop (controller), command mux and cmd_vel bridge.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = ipex_motion.cmd_vel_bridge:main',
            'controller = ipex_motion.controller:main',
            'cmd_mux = ipex_motion.cmd_mux:main',
        ],
    },
)
