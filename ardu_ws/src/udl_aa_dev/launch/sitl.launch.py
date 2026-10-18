"""
SITL - vehicle_node only. There is no RC in SITL, so rc_monitor is not
launched; sequences are run by hand with scripts/launch_dev.sh --target N.

Assumes ArduPilot SITL and MAVROS are already running, same as
scripts/launch_fs.sh.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('udl_aa_dev'),
        'config',
        'udl_aa_dev.yaml',
    )

    return LaunchDescription([
        Node(
            package='udl_aa_dev',
            executable='vehicle_node',
            name='vehicle_node',
            parameters=[config],
            output='screen'
        ),
    ])
