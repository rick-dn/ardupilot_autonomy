from glob import glob

from setuptools import setup

package_name = 'udl_aa_ss'

setup(
    name=package_name,
    version='0.1.0',
    packages=[
        package_name,
        f'{package_name}.conditions',
        f'{package_name}.sequences',
    ],
    # launch/ and config/ are globbed rather than listed. Both are empty for
    # now, and a glob returns nothing rather than failing the build - so a
    # launch file or a yaml can be dropped in later without editing this file.
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='hello@usefuldynamics.io',
    description='Safety stack for the ArduPilot autonomy stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        # Deliberately empty. The node entry point lands with the node:
        #   'udl_aa_ss = udl_aa_ss.safety_commander:main'
        # Declaring it before the module exists builds fine but leaves a
        # `ros2 run` that fails with an ImportError instead of a clear
        # "executable not found".
        'console_scripts': [],
    },
)
