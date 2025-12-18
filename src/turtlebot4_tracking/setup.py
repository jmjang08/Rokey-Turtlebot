from setuptools import setup
from glob import glob


package_name = 'turtlebot4_tracking'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='me',
    maintainer_email='me@example.com',
    description='YOLO + Depth Fusion + MoveRobot package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'detection = turtlebot4_tracking.detection_node:main',
            'tracking = turtlebot4_tracking.tracking_node:main',
            'target_calculator = turtlebot4_tracking.target_calculator:main',
        ],
    },
)
