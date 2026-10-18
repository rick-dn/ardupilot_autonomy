import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Required, not optional. The commander declares its parameters from
    # whatever the config supplies rather than naming any of them - the
    # conditions own their thresholds - so without this file the conditions are
    # handed an empty dict and the first tick raises KeyError. Failing loudly is
    # the point: defaulting the thresholds in code would put volts back in the
    # commander.
    config = os.path.join(
        get_package_share_directory('udl_aa_fc'),
        'config',
        'safety.yaml',
    )

    return LaunchDescription([
        Node(
            package='udl_aa_fc',
            executable='udl_aa_fc',
            name='udl_aa_fc',
            parameters=[config],
            output='screen'
        ),
    ])
