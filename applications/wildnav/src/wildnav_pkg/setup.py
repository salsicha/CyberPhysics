from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'wildnav_pkg'

setup(
    name=package_name,
    version='0.1.0',
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
    maintainer='Alex Moran',
    maintainer_email='salsicha@gmail.com',
    description='Low-rate visual geolocation and correction fusion for ROS 2',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'wildnav_node = wildnav_pkg.wildnav_node:main',
            'navigation_fusion_node = wildnav_pkg.navigation_fusion:main',
            'wildnav_seed_cache = wildnav_pkg.seed_cache:main',
            'nav_evaluator = wildnav_pkg.nav_evaluator:main',
        ],
    },
)
