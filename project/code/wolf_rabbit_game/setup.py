from setuptools import find_packages, setup
from glob import glob

package_name = 'wolf_rabbit_game'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Single-package ROS 2 starter project for a TurtleBot4 wolf-and-rabbit game.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yolo_detector = wolf_rabbit_game.yolo_detector:main',
            'geofence_node = wolf_rabbit_game.geofence_node:main',
            'rabbit_fsm = wolf_rabbit_game.rabbit_fsm:main',
            'wolf_fsm = wolf_rabbit_game.wolf_fsm:main',
            'referee_node = wolf_rabbit_game.referee_node:main',
        ],
    },
)
