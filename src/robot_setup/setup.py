import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_setup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'config'),glob('config/*.yaml')),
        (os.path.join('share',package_name,'launch'),glob('launch/*.launch.py')),
        (os.path.join('share',package_name,'urdf'),glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swarnava',
    maintainer_email='swarnava@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'warehouse_mission = robot_setup.mission:main',
            'detect_and_shoot = robot_setup.detect_shoot:main',
        ],
    },
)
