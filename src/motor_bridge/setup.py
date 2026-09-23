from setuptools import find_packages, setup

package_name = 'motor_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tural',
    maintainer_email='tural.bozkurt999@gmail.com',
    description='Serial bridge between ROS2 and the ESP32-S3 motor controller',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_bridge_node = motor_bridge.serial_bridge_node:main',
        ],
    },
)
