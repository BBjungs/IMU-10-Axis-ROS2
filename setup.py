import os, glob
from setuptools import setup

package_name = 'wit_ros2_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob.glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob.glob('meshes/*') + ['IMU.stp']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        'wit_ros2_imu = wit_ros2_imu.wit_ros2_imu:main',
        'imu_odom = wit_ros2_imu.imu_odom:main',
        ],
    },
)
