from setuptools import setup

package_name = 'udl_aa_fc'

setup(
    name=package_name,
    version='0.1.0',
    packages=[
        package_name,
        f'{package_name}.sequences',
        f'{package_name}.sequences.test',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', ['launch/udl_aa_fc.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='hello@usefuldynamics.io',
    description='Flight commander for the ArduPilot autonomy stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'udl_aa_fc = udl_aa_fc.vehicle_commander:main',
            'test_goto_local = udl_aa_fc.sequences.test.test_goto_local:main',
            'test_goto_body = udl_aa_fc.sequences.test.test_goto_body:main',
            'test_velocity = udl_aa_fc.sequences.test.test_velocity:main',
            'test_goto_global = udl_aa_fc.sequences.test.test_goto_global:main',
        ],
    },
)
