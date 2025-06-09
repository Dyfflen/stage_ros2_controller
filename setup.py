import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'stage_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dyfflen',
    maintainer_email='dyfflen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	"py_node = stage_navigation.navigation_node:main",
            "publisher = stage_navigation.publisher:main",
            "subscriber = stage_navigation.subscriber:main",
            "add_two_num = stage_navigation.add_two_num:main",
            "bug2_navigator = stage_navigation.bot:main",
            'run_bug2 = stage_navigation.run_bug2:main',
            'robot_navigation_node = stage_navigation.robot_navigation_node:main',
            'ekf = stage_navigation.ekf:main',
            "homing_beacon = stage_navigation.homing_beacon:main"
        ],
    },
)
