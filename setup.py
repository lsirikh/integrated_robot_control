from setuptools import setup
import os
from glob import glob

package_name = 'integrated_robot_control'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lsirikh',
    maintainer_email='lsirikh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = integrated_robot_control.control_node:main',
            'data_report_node = integrated_robot_control.data_report_node:main',
            'robot_control_profile = integrated_robot_control.robot_control_profile:main',  # 이 부분을 추가
        ],
    },
)
