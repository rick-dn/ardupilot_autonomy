from setuptools import setup

package_name = 'udl_aa_fc'

setup(
    name=package_name,
    version='0.1.0',
    # No sequences here. The commander decides which sequence should run and
    # publishes a request for it; udl_aa_seqs holds every sequence and runs
    # them, so this package would only have carried a second copy that nothing
    # executed.
    packages=[
        package_name,
        f'{package_name}.conditions',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', ['launch/udl_aa_fc.launch.py']),
        (f'share/{package_name}/config', ['config/safety.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='hello@usefuldynamics.io',
    description='Flight commander for the ArduPilot autonomy stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        # The standalone test scripts moved with the sequences - they are
        # `ros2 run udl_aa_seqs test_*` now.
        'console_scripts': [
            'udl_aa_fc = udl_aa_fc.vehicle_commander:main',
        ],
    },
)
