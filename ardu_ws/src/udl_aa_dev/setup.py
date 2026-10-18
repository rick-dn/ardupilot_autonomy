from setuptools import setup

package_name = 'udl_aa_dev'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.sequences'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', [
            'launch/sitl.launch.py',
            'launch/flight.launch.py',
        ]),
        (f'share/{package_name}/config', ['config/udl_aa_dev.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='hello@usefuldynamics.io',
    description='Development stack - mavros adapter published as topics, '
                'driven by standalone sequences',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Nodes
            'vehicle_node = udl_aa_dev.vehicle_node:main',
            'rc_monitor = udl_aa_dev.rc_monitor:main',
            # Sequences - each standalone, run start to finish
            'takeoff = udl_aa_dev.sequences.takeoff:main',
            'goto_local = udl_aa_dev.sequences.goto_local:main',
            'land = udl_aa_dev.sequences.land:main',
        ],
    },
)
