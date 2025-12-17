from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'turtlebot4_broadcast'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suet',
    maintainer_email='suet.lee@uni-konstanz.de',
    description='Receive positional data for turtlebots from simulation or tracking systems, and broadcast to namespaced topics.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'super_broadcaster_node = turtlebot4_broadcast.super_broadcaster_node:main',
        ],
    },
)
