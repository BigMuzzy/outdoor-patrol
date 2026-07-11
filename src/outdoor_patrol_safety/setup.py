from glob import glob

from setuptools import find_packages, setup

package_name = 'outdoor_patrol_safety'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Outdoor Patrol Team',
    maintainer_email='dev@example.com',
    description='Geometric 2D-LiDAR forward safety brake (M3, ADR-013).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_safety = outdoor_patrol_safety.scan_safety_node:main',
        ],
    },
)
