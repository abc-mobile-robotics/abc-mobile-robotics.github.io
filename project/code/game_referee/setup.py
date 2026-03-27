from setuptools import setup

package_name = 'game_referee'

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
    description='Game-state management and capture detection.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'game_referee_node = game_referee.game_referee_node:main',
        ],
    },
)
