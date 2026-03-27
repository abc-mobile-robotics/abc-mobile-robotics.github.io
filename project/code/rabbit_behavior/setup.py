from setuptools import setup

package_name = 'rabbit_behavior'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team',
    maintainer_email='team@example.com',
    description='Rabbit finite-state machine for search and escape.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rabbit_fsm = rabbit_behavior.rabbit_fsm:main',
        ],
    },
)
