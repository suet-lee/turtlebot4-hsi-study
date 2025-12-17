from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'turtlebot4_team'

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
    description='Code for HSI robot sharing experiments for turtlebots.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flocking_node = turtlebot4_team.flocking_node:main',
            'teaming_node = turtlebot4_team.teaming_node:main',
            'task_node = turtlebot4_team.task_node:main',
            'shuffle_node = turtlebot4_team.shuffle_node:main',
        ],
    },
)
