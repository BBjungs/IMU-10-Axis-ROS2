import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share_directory = get_package_share_directory('wit_ros2_imu')
    rviz_config = os.path.join(
        package_share_directory,
        'config',
        'imu.rviz'
    )
    urdf_file = os.path.join(
        package_share_directory,
        'urdf',
        'imu.urdf'
    )

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    rviz_and_imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu',
        remappings=[('/imu/data_raw', '/imu/data')],
        parameters=[{'port': '/dev/imu_usb'},
                    {"baud": 9600}],
        output="screen"

    )

    imu_odom_node = Node(
        package='wit_ros2_imu',
        executable='imu_odom',
        name='imu_odom',
        parameters=[
            {'imu_topic': '/imu/data'},
            {'odom_topic': '/odom'},
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
            {'publish_tf': True},
            {'integrate_position': False},
            {'axis_correction_yaw': 0.0},
        ],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    rviz_display_node = Node(
        package='rviz2',
        executable="rviz2",
        arguments=['-d', rviz_config],
        output="screen"
    )

    return LaunchDescription(
        [
            rviz_and_imu_node,
            imu_odom_node,
            robot_state_publisher_node,
            rviz_display_node
        ]
    )
