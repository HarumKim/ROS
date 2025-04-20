from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'act1_3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harum Kim',
    maintainer_email='A00836962@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_movement = act1_3.robot_movement:main',
            'sphere_movement = act1_3.sphere_movement:main',
            'teleop_bridge = act1_3.teleop_bridge:main',
        ],
    },
)


