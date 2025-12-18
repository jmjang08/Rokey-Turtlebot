from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'turtlebot4_nav2pose'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='inseonyu7@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nav2pose_node = turtlebot4_nav2pose.nav2pose_node:main',
            'init_pose_publisher = turtlebot4_nav2pose.init_pose_publisher:main',
        ],
    },
)
