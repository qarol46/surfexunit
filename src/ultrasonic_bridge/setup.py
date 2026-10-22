from setuptools import find_packages, setup

package_name = 'ultrasonic_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Bridge for HC-SR04 ultrasonic sensor via Serial',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultrasonic_node = ultrasonic_bridge.ultrasonic_node:main',
        ],
    },
)