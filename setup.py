from setuptools import setup

package_name = 'ardupilot_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.actions', f'{package_name}.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
