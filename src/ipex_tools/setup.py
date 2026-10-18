from setuptools import find_packages, setup

package_name = 'ipex_tools'

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
    description='Operator tools for Roxey, including the roxey command.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'roxey = ipex_tools.roxey:main',
        ],
    },
)
