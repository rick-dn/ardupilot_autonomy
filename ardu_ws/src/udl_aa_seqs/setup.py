from setuptools import setup

package_name = 'udl_aa_seqs'

setup(
    name=package_name,
    version='0.1.0',
    # sequences.backup is deliberately absent: those are kept for reference and
    # are not ported onto the topic contract, so they are copied but not
    # installed.
    packages=[
        package_name,
        f'{package_name}.sequences',
        f'{package_name}.sequences.test',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='hello@usefuldynamics.io',
    description='Flight sequences for the ArduPilot autonomy stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        # Standalone test scripts only. There is no node entry point yet - the
        # sequences are here, but what starts them is not decided.
        'console_scripts': [
            'test_goto_local = udl_aa_seqs.sequences.test.test_goto_local:main',
            'test_goto_body = udl_aa_seqs.sequences.test.test_goto_body:main',
            'test_velocity = udl_aa_seqs.sequences.test.test_velocity:main',
            'test_goto_global = udl_aa_seqs.sequences.test.test_goto_global:main',
            'aruco_lander = udl_aa_seqs.sequences.test.aruco_lander:main',
        ],
    },
)
