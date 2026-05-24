from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mapnav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alexmoran',
    maintainer_email='alexmoran@todo.todo',
    description='Terrain-relative navigation ROS2 node using SRTM height maps',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapnav_node = mapnav_pkg.mapnav_node:main',
        ],
    },
)
