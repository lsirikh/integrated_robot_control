from setuptools import setup
import os
from glob import glob

package_name = 'integrated_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),  # 이 부분을 확인
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = integrated_robot_control.control_node:main',
            'robot_control_node = integrated_robot_control.rpi_robot_control:main',
            'ekf_node = integrated_robot_control.ekf_node:main',  # ekf_node 추가

        ],
    },
)
