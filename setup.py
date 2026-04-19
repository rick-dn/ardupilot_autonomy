from setuptools import setup

package_name = 'ardupilot_autonomy'

# setup(
#     name=package_name,
#     version='0.1.0',
#     packages=[package_name, f'{package_name}.utils'],
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='Your Name',
#     maintainer_email='hello@usefuldynamics.io',
#     description='ArduPilot MAVROS autonomy package',
#     license='MIT',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             'vehicle_interface = ardupilot_autonomy.vehicle_interface:main',
#         ],
#     },
# )


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.actions', f'{package_name}.utils', f'{package_name}.sequences', f'{package_name}.vision'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', ['launch/startup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='hello@usefuldynamics.io',
    description='ArduPilot MAVROS autonomy package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_interface = ardupilot_autonomy.vehicle_interface:main',
            'takeoff_sequence = ardupilot_autonomy.sequences.takeoff_sequence:main',
            'rc_monitor = ardupilot_autonomy.rc_monitor:main',
            'scan_sequence = ardupilot_autonomy.sequences.scan_sequence:main',
            'scan_sequence_gps = ardupilot_autonomy.sequences.scan_sequence_gps:main',
            'object_detector = ardupilot_autonomy.vision.object_detector:main',
            'follow_sequence = ardupilot_autonomy.sequences.follow_sequence:main',
            'test_neu = ardupilot_autonomy.sequences.test_neu_waypoint:main',
            'test_gps = ardupilot_autonomy.sequences.test_gps_waypoint:main',
            'follow_node = ardupilot_autonomy.follow_node:main',
            'follow_node_unit_test = ardupilot_autonomy.follow_node_unit_test:main',
        ],
    },
)
