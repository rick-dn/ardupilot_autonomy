from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # MAVROS node
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            parameters=[{
                'fcu_url': '/dev/ttyACM0:57600',  # Adjust port
            }],
            output='log'
        ),
        
        # Vehicle Interface
        Node(
            package='ardupilot_autonomy',
            executable='vehicle_interface',
            name='vehicle_interface',
            output='log'
        ),
        
        # RC Monitor
        Node(
            package='ardupilot_autonomy',
            executable='rc_monitor',
            name='rc_monitor',
            output='log'
        ),
    ])
