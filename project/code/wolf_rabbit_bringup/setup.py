from setuptools import setup
from glob import glob

package_name = 'wolf_rabbit_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team',
    maintainer_email='team@example.com',
    description='Launch and configuration for the full wolf-rabbit system.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dummy_entry = wolf_rabbit_bringup.dummy_entry:main',
        ],
    },
)
