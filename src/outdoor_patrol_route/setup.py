from glob import glob

from setuptools import find_packages, setup

package_name = 'outdoor_patrol_route'

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
        ('share/' + package_name + '/config', glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Outdoor Patrol Team',
    maintainer_email='dev@example.com',
    description='GNSS teach-and-repeat route recording and following.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'route_recorder = outdoor_patrol_route.route_recorder:main',
            'route_follower = outdoor_patrol_route.route_follower:main',
        ],
    },
)
