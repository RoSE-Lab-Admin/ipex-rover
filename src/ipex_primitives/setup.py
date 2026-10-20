from setuptools import find_packages, setup

package_name = 'ipex_primitives'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='graco',
    maintainer_email='gracon_mangnall@mines.edu',
    description='Roxey motion primitives behind one action, with time and energy per run.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'primitive_server = ipex_primitives.server:main',
        ],
    },
)
