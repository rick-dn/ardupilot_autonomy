from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='udl_aa_fc',
            executable='udl_aa_fc',
            name='udl_aa_fc',
            output='screen'
        ),
    ])
