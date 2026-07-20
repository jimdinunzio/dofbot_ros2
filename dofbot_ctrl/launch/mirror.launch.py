"""
Read-only mirror: physical arm -> RViz.

Starts robot_state_publisher (URDF -> /tf from /joint_states), RViz, and the
joint_state_mirror node. Deliberately does NOT start joint_state_publisher --
the mirror is the sole publisher of /joint_states, and a second one would fight
it on the topic.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'release_torque', default_value='true', choices=['true', 'false'],
        description='Disable servo torque so the arm can be posed by hand.'))
    ld.add_action(DeclareLaunchArgument(
        'port', default_value='/dev/ttyTHS1',
        description='Serial port for the bus servos.'))
    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='true', choices=['true', 'false'],
        description='Start RViz; set false to run headless (e.g. on the Jetson, '
                    'with RViz on a remote machine).'))

    # robot_state_publisher only (via urdf_launch description.launch.py).
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch',
                              'description.launch.py']),
        launch_arguments={
            'urdf_package': 'dofbot_description',
            'urdf_package_path':
                PathJoinSubstitution(['urdf', 'dofbot.urdf'])}.items()))

    ld.add_action(Node(
        package='dofbot_ctrl', executable='joint_state_mirror',
        name='joint_state_mirror', output='screen',
        parameters=[{'release_torque': LaunchConfiguration('release_torque'),
                     'port': LaunchConfiguration('port')}]))

    ld.add_action(Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('dofbot_ctrl'), 'rviz', 'mirror.rviz'])]))

    return ld
