from setuptools import find_packages, setup

package_name = 'ipex_energy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='graco',
    maintainer_email='gracon_mangnall@mines.edu',
    description='Power and energy accounting for Roxey (per channel, per subsystem, total).',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'energy_meter = ipex_energy.energy_meter:main',
        ],
    },
)
