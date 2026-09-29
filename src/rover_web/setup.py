import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rover_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'static'), glob('rover_web/static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tural',
    maintainer_email='tural.bozkurt999@gmail.com',
    description='Browser control panel for the restaurant rover',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rover_web_server = rover_web.server:main',
        ],
    },
)
