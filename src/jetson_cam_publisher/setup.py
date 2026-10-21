from setuptools import setup
import os
from glob import glob

package_name = 'jetson_cam_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kirill',
    maintainer_email='kirill@example.com',
    description='Publishes images from Jetson CSI cameras using GStreamer and OpenCV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jetson_cam_node = jetson_cam_publisher.jetson_cam_publisher:main',
        ],
    },
)