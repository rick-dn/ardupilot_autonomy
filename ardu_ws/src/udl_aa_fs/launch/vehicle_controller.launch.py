import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('udl_aa_fs'),
        'config',
        'vehicle_controller.yaml',
    )

    return LaunchDescription([
        Node(
            package='udl_aa_fs',
            executable='vehicle_controller',
            name='vehicle_controller',
            parameters=[config],
            output='screen'
        ),
    ])
