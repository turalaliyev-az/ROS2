from setuptools import find_packages, setup

package_name = 'rover_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tural',
    maintainer_email='tural.bozkurt999@gmail.com',
    description='Object recognition for the rover: people, chairs and tables',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detector = rover_perception.detector:main',
        ],
    },
)
