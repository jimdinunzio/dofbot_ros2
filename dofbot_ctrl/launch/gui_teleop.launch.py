"""
Drive the real arm from RViz jsp-GUI sliders, seeded to the arm's current pose.

  robot_state_publisher  URDF -> /tf, publishes /robot_description
  gui_teleop             reads hardware once -> /seed_joint_states (latched),
                         then writes /joint_states commands to the servos
  joint_state_publisher_gui  sliders; source_list seeds them from the hardware
                         pose, publishes /joint_states
  rviz2

gui_teleop owns /dev/ttyTHS1. Do NOT run the mirror at the same time.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('port', default_value='/dev/ttyTHS1'))
    ld.add_action(DeclareLaunchArgument('move_time_ms', default_value='500'))

    # robot_state_publisher (publishes /robot_description that jsp-gui reads).
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch',
                              'description.launch.py']),
        launch_arguments={
            'urdf_package': 'dofbot_description',
            'urdf_package_path':
                PathJoinSubstitution(['urdf', 'dofbot.urdf'])}.items()))

    ld.add_action(Node(
        package='dofbot_ctrl', executable='gui_teleop', name='gui_teleop',
        output='screen',
        parameters=[{'port': LaunchConfiguration('port'),
                     'move_time_ms': LaunchConfiguration('move_time_ms'),
                     'seed_topic': 'seed_joint_states'}]))

    ld.add_action(Node(
        package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui', output='screen',
        parameters=[{'source_list': ['seed_joint_states']}]))

    ld.add_action(Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('dofbot_ctrl'), 'rviz', 'mirror.rviz'])]))

    return ld
