from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'realsense_elevation_mapper'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sequor',
    maintainer_email='dev@sequorrobotics.com',
    description='RealSense PointCloud2 -> local elevation map ROS2 node.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'local_elevation_mapper_node = '
            'realsense_elevation_mapper.local_elevation_mapper_node:main',
        ],
    },
)
