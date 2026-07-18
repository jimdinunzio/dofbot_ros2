from glob import glob

from setuptools import setup

package_name = 'dofbot_ctrl'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jim',
    maintainer_email='jim@dinunzio.com',
    description='DOFBOT arm control nodes: servo/URDF angle mapping and a '
                'read-only joint-state mirror.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_mirror = dofbot_ctrl.joint_state_mirror:main',
            'gui_teleop = dofbot_ctrl.gui_teleop:main',
        ],
    },
)
