from setuptools import find_packages, setup

package_name = 'twocops_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/docking.launch.py',
            'launch/twocops_bringup.launch.py',
        ]),
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
            'docking_node = twocops_bringup.docking_node:main',
            'dock_state_manager = twocops_bringup.dock_state_manager:main',
            'transition_manager = twocops_bringup.transition_manager:main',
            'nav2_ready_waiter = twocops_bringup.nav2_ready_waiter:main',
        ],
    },
)
