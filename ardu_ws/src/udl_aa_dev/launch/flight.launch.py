"""
Physical flight - vehicle_node plus rc_monitor.

rc_monitor is the carried-over version from the Pi: it subscribes to
/mavros/rc/in directly and spawns sequences with `ros2 run`, so the switch
path and scripts/launch_dev.sh --target N start the identical executable.

Assumes MAVROS is already running - on the Pi it is mavros.service.
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
        Node(
            package='udl_aa_dev',
            executable='rc_monitor',
            name='rc_monitor',
            output='screen'
        ),
    ])
