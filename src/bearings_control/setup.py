from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bearings_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Incluir archivo YAML y launch
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuban',
    maintainer_email='yuban.vasquez@gmail.com',
    description='Multi-agent bearings-based formation controller for Mavic drones in Webots.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bearings_control_node = bearings_control.bearings_control_node:main',
            'formation_visualizer_node = bearings_control.formation_visualizer_node:main',
            'bearings_3d_control_node = bearings_control.bearings_3d_control_node:main',
        ],
    },
)
