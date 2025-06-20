from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ik_bag_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='alexander.oldemeier@tum.de',
    description='ROS2 package for processing ROS bags and computing IK solutions using MoveIt',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_processor = ik_bag_processor.ik_processor:main',
        ],
    },
)
