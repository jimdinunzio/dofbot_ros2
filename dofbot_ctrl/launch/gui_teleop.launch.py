"""
Drive the real arm from RViz jsp-GUI sliders, seeded to the arm's current pose.

  robot_state_publisher  URDF -> /tf, publishes /robot_description
  gui_teleop             reads hardware once -> /seed_joint_states (latched),
                         then writes /joint_states commands to the servos
  joint_state_publisher_gui  sliders; source_list seeds them from the hardware
                         pose, publishes /joint_states
  rviz2

Split into two sides so it can run distributed:
  hardware=true  -> robot_state_publisher + gui_teleop node  (the arm side;
                    owns /dev/ttyTHS1, must run on the machine with the arm)
  guis=true      -> jsp-gui sliders + RViz                   (the GUI side)

Default runs both locally. For a Jetson + laptop setup:
  Jetson:  ros2 launch dofbot_ctrl gui_teleop.launch.py guis:=false
  laptop:  ros2 launch dofbot_ctrl gui_teleop.launch.py hardware:=false

The two sides talk over ROS 2 (seed_joint_states, joint_states, robot_description,
tf), so the laptop needs a matching ROS_DOMAIN_ID and dofbot_ctrl built for the
RViz config. Do NOT run the mirror at the same time as the hardware side.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('port', default_value='/dev/ttyTHS1'))
    ld.add_action(DeclareLaunchArgument('move_time_ms', default_value='500'))
    ld.add_action(DeclareLaunchArgument(
        'hardware', default_value='true', choices=['true', 'false'],
        description='Run robot_state_publisher + the gui_teleop node (the arm '
                    'side). Set false on a machine without the arm.'))
    ld.add_action(DeclareLaunchArgument(
        'guis', default_value='true', choices=['true', 'false'],
        description='Run jsp-gui sliders + RViz (the GUI side). Set false on the '
                    'headless Jetson and run these on the laptop instead.'))

    hardware = IfCondition(LaunchConfiguration('hardware'))
    guis = IfCondition(LaunchConfiguration('guis'))

    # --- arm side ---
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch',
                              'description.launch.py']),
        launch_arguments={
            'urdf_package': 'dofbot_description',
            'urdf_package_path':
                PathJoinSubstitution(['urdf', 'dofbot.urdf'])}.items(),
        condition=hardware))

    ld.add_action(Node(
        package='dofbot_ctrl', executable='gui_teleop', name='gui_teleop',
        output='screen', condition=hardware,
        parameters=[{'port': LaunchConfiguration('port'),
                     'move_time_ms': LaunchConfiguration('move_time_ms'),
                     'seed_topic': 'seed_joint_states'}]))

    # --- GUI side ---
    ld.add_action(Node(
        package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui', output='screen', condition=guis,
        parameters=[{'source_list': ['seed_joint_states']}]))

    ld.add_action(Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=guis,
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('dofbot_ctrl'), 'rviz', 'mirror.rviz'])]))

    return ld
