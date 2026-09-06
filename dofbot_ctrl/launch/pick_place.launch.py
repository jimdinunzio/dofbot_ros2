"""
The stack `pick_place` needs: MoveIt on mock joints, plus the servo bridge.

    # on the robot, driving the real arm
    ros2 launch dofbot_ctrl pick_place.launch.py
    # simulation only -- no serial port touched, safe anywhere
    ros2 launch dofbot_ctrl pick_place.launch.py rviz:=false bridge:=false
    # headless (e.g. over ssh), with RViz run separately on a laptop
    ros2 launch dofbot_ctrl pick_place.launch.py rviz:=false

For that last case, the laptop runs RVIZ AND NOTHING ELSE:

    ros2 launch dofbot_moveit moveit_rviz.launch.py

Do NOT run this file on the laptop with rviz:=true to get the display.
`bridge:=false` only drops moveit_bridge; move_group, ros2_control_node,
robot_state_publisher and the controller spawners all still start, so a second
copy of the whole stack joins the domain. The symptom is two /move_action
servers -- "Ignoring unexpected goal response", "unknown goal response,
ignoring...", CONTROL_FAILED, and an arm driven by two controllers at once.

then, in another terminal:

    ros2 run dofbot_ctrl pick_place -- --check-states
    # x y z is the object CENTRE in base_link: the floor height plus half the
    # object's own HEIGHT. 0.039 is an upright 122 mm can on carpet
    # (-0.022 + 0.061); 0.30 is where the can wants to be, not the closest it
    # can sit -- see jetson-nano-services/arm-service/README.md,
    # "Where to put the robot"
    ros2 run dofbot_ctrl pick_place -- --pick 0.30 0.0 0.039
    ros2 run dofbot_ctrl pick_place -- --place

This is dofbot_moveit's demo.launch.py plus moveit_bridge, with two nodes
deliberately left out:

- chassis_collision. Superseded. It injected the chassis and floor as runtime
  planning-scene objects (Path A); chassis.xacro now puts them in the URDF as
  real links (Path B), so running it would add a second copy of both.
- joint_state_mirror. It reads the servos and publishes /joint_states, which is
  exactly what ros2_control's mock hardware is already doing here, and it holds
  /dev/ttyTHS1, which moveit_bridge needs. Two publishers on /joint_states also
  make MoveIt's idea of the robot state flicker between them.

STOP gui_teleop AND joint_state_mirror BEFORE LAUNCHING THIS ON HARDWARE.
moveit_bridge needs exclusive access to the serial port.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _include(launch_file, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('dofbot_moveit'), 'launch', launch_file])),
        condition=condition)


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        'dofbot_description', package_name='dofbot_moveit').to_moveit_configs()

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'bridge', default_value='true', choices=['true', 'false'],
        description='Run moveit_bridge, which drives the real servos from '
                    '/joint_states. Set false for simulation-only.'))
    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='true', choices=['true', 'false'],
        description='Start RViz with the MoveIt display.'))
    ld.add_action(DeclareLaunchArgument(
        'port', default_value='/dev/ttyTHS1',
        description='Serial port for the bus servos.'))
    ld.add_action(DeclareLaunchArgument(
        'encoder_rate', default_value='0.0',
        description='Hz for moveit_bridge to read the encoders back onto '
                    '/servo_states, for measure_tracking. 0 disables it. Also '
                    'settable live: ros2 param set /moveit_bridge '
                    'encoder_rate 10.0'))

    # world -> base_link, from the SRDF virtual joint.
    ld.add_action(_include('static_virtual_joint_tfs.launch.py'))
    # URDF -> /tf, and it latches /robot_description.
    ld.add_action(_include('rsp.launch.py'))
    ld.add_action(_include('move_group.launch.py'))
    ld.add_action(_include('moveit_rviz.launch.py',
                           condition=IfCondition(LaunchConfiguration('rviz'))))

    # Mock joints. MoveIt plans against these and the controller plays the
    # trajectory back on /joint_states; moveit_bridge turns that into servo
    # writes. Execution is therefore OPEN-LOOP -- MoveIt believes the mock
    # joints, not the encoders, so a stalled or blocked servo goes unnoticed.
    ld.add_action(Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[moveit_config.robot_description,
                    str(moveit_config.package_path
                        / 'config/ros2_controllers.yaml')]))
    ld.add_action(_include('spawn_controllers.launch.py'))

    ld.add_action(Node(
        package='dofbot_ctrl', executable='moveit_bridge',
        name='moveit_bridge', output='screen',
        condition=IfCondition(LaunchConfiguration('bridge')),
        # The port and encoder_rate. grip_time_ms is deliberately NOT passed
        # from here: a launch default silently outranks the node's own, so the
        # value in moveit_bridge.py would stop being the value in use. Change
        # it there, or with `ros2 param set` on the running node.
        #
        # encoder_rate is safe to pass because its default here (0.0) is the
        # same as the node's, so passing it cannot change the default -- it
        # only lets the launch line set it. It is live-settable too.
        #
        # ParameterValue with an explicit type because a LaunchConfiguration
        # substitutes to a STRING, and encoder_rate is declared double -- the
        # node would refuse to start on the type mismatch.
        parameters=[{'port': LaunchConfiguration('port'),
                     'encoder_rate': ParameterValue(
                         LaunchConfiguration('encoder_rate'),
                         value_type=float)}]))

    return ld
