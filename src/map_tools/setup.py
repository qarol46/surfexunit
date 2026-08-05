import os
from glob import glob
from setuptools import setup

package_name = 'map_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Map publisher, inflation overlay, zones publisher',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_publisher = map_tools.map_publisher:main',
            'inflation_publisher = map_tools.inflation_publisher:main',
            'zones_publisher = map_tools.zones_publisher:main',
            'path_planner = map_tools.path_planner:main',
        ],
    },
)