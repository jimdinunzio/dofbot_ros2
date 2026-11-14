from glob import glob

from setuptools import setup

package_name = 'dofbot_ctrl'

setup(
    name=package_name,
    version='0.0.1',
    # tuning/ is a subpackage, so it must be named explicitly -- packages=
    # is not recursive, and omitting it installs the entry points without the
    # modules they point at.
    packages=[package_name, package_name + '.tuning'],
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
    description='DOFBOT arm control: servo/URDF angle mapping, a joint-state '
                'mirror, analytic kinematics and a programmatic pick-and-place '
                'layer over MoveIt.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_mirror = dofbot_ctrl.joint_state_mirror:main',
            'gui_teleop = dofbot_ctrl.gui_teleop:main',
            'calibrate_zero = dofbot_ctrl.calibrate_zero:main',
            'set_torque = dofbot_ctrl.set_torque:main',
            'check_jaws = dofbot_ctrl.check_jaws:main',
            'moveit_bridge = dofbot_ctrl.moveit_bridge:main',
            'move_to_state = dofbot_ctrl.move_to_state:main',
            'chassis_collision = dofbot_ctrl.chassis_collision:main',
            'pick_place = dofbot_ctrl.pick_place:main',
            'camera_server = dofbot_ctrl.camera_server:main',
            'arm_worker = dofbot_ctrl.arm_worker:main',
            'vision_check = dofbot_ctrl.vision_check:main',
            'calibrate_view = dofbot_ctrl.calibrate_view:main',
            'wave_arm = dofbot_ctrl.wave_arm:main',
            'measure_bus = dofbot_ctrl.tuning.measure_bus:main',
            'measure_tracking = dofbot_ctrl.tuning.measure_tracking:main',
            'rescore_held = dofbot_ctrl.tuning.rescore_held:main',
        ],
    },
)
