import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'roskit_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'websockets>=12.0',
        'cbor2>=5.6.0',
        'numpy>=1.24.0',
        'Pillow>=10.0.0',
    ],
    zip_safe=True,
    maintainer='RosKit Contributors',
    maintainer_email='your-email@example.com',
    description='High-performance web bridge for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge = roskit_bridge.bridge_node:main',
        ],
    },
)
