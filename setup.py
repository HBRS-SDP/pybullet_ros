import os
from glob import glob
from setuptools import setup
from pathlib import Path

package_name = 'pybullet_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}/plugins', f'{package_name}/sdf'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
        (os.path.join('share', package_name), glob('config/*.yaml')),
        (os.path.join('share', package_name), glob('resource/urdf/r2d2_robot/*')),
        (os.path.join('share', package_name), glob('resource/urdf/laser_scanner/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros_user',
    maintainer_email='simon.deussen@pm.me',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pybullet_ros2 = pybullet_ros2.pybullet_ros2:main'
        ],
    },
)
