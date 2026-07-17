from setuptools import setup

package_name = 'dofbot_arm_lib'

setup(
    name=package_name,
    version='0.0.1',
    # The ROS package is dofbot_arm_lib (REP-144 requires a lowercase name);
    # the module it installs is Arm_Lib, matching the vendor import path.
    packages=['Arm_Lib'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='jim',
    maintainer_email='jim@dinunzio.com',
    description='YB-SD15M serial bus-servo driver for the DOFBOT arm, '
                'installed as the vendor-compatible Arm_Lib Python module.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
