from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'spie_vehicle_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']), 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spie',
    maintainer_email='t112360140@ntut.org.tw',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={},
    entry_points={
        'console_scripts': [
            'spie_vehicle_interface_exec = spie_vehicle_interface.interface:main',
        ],
    },
)
