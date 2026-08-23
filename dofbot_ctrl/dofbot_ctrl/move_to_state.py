#!/usr/bin/env python3
# coding: utf-8
"""
Move the arm to one of the saved states, from the command line.

    ros2 run dofbot_ctrl move_to_state ready
    ros2 run dofbot_ctrl move_to_state init
    # names, joint values and where the tool ends up -- needs nothing running
    ros2 run dofbot_ctrl move_to_state --list

The same states RViz's MoveIt panel offers, and the same planning: a joint-space
goal to move_group, planned by OMPL and collision-checked against the live
planning scene. What this adds over the panel is that it scripts, it logs, and
it is the same code path a pick takes to 'ready' or 'carry'.

The states live in moveit_client.NAMED_STATES and are mirrored into
dofbot_description.srdf, which is where the RViz panel reads them from.
test/test_named_states.py parses the SRDF and fails if the two disagree -- they
are two copies of the same numbers, and they have drifted before.

THIS DOES NOT TOUCH THE PLANNING SCENE, and that is the difference between it
and `pick_place --reset`. A move that fails immediately with
INVALID_MOTION_PLAN usually means the scene still holds an object from a run
that died partway: nothing can be planned out of a start state that is inside
one, because the plan's own first waypoint is invalid. Use `pick_place --reset`
for that -- it clears the scene and opens the gripper before it moves.

Whether the states are themselves collision-free is a question for the live
scene, not for this file: `pick_place --check-states` asks move_group.
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from dofbot_ctrl import dofbot_kinematics as kin
from dofbot_ctrl.moveit_client import NAMED_STATES, DofbotMoveIt, MoveItError


def print_states(out=sys.stdout):
    """The catalogue, with the tool pose our own FK puts each state at."""
    print('%-11s %s  %s' % ('state',
                            ' '.join('%8s' % n.replace('_Joint', '')
                                     for n in kin.JOINT_NAMES),
                            'tcp (x, y, z) phi'), file=out)
    for name in sorted(NAMED_STATES):
        joints = NAMED_STATES[name]
        x, y, z, phi, _roll = kin.fk(joints)
        print('%-11s %s  (%.3f, %.3f, %.3f) %.3f'
              % (name, ' '.join('%8.4f' % q for q in joints), x, y, z, phi),
              file=out)


class StateMover(Node):

    def __init__(self):
        super().__init__('move_to_state')
        self.mc = DofbotMoveIt(self)

    def move(self, name):
        self.mc.move_named(name)
        # FK on the live joints, so this reports where the arm ACTUALLY ended
        # up rather than reprinting the target. On mock joints those are the
        # same thing; on the real arm they are not, and execution is open-loop.
        x, y, z, phi, roll = self.mc.current_pose()
        self.get_logger().info(
            'at %r: tcp (%.3f, %.3f, %.3f) phi=%.3f roll=%.3f'
            % (name, x, y, z, phi, roll))


def main(args=None):
    parser = argparse.ArgumentParser(
        prog='move_to_state', description=__doc__.split('\n\n')[0])
    parser.add_argument('state', nargs='?',
                        help='saved state to move to (%s)'
                             % ', '.join(sorted(NAMED_STATES)))
    parser.add_argument('--list', action='store_true',
                        help='print the saved states and exit, without '
                             'connecting to move_group')
    cli = parser.parse_args(remove_ros_args(sys.argv)[1:])

    # Both of these are settled before rclpy.init, so a typo costs nothing and
    # says what the choices are rather than timing out on a service call.
    if cli.list:
        print_states()
        return 0
    if cli.state is None:
        parser.print_usage(sys.stderr)
        print('give a state name: %s' % ', '.join(sorted(NAMED_STATES)),
              file=sys.stderr)
        return 2
    if cli.state not in NAMED_STATES:
        print('unknown state %r; have %s'
              % (cli.state, ', '.join(sorted(NAMED_STATES))), file=sys.stderr)
        return 2

    rclpy.init(args=args)
    node = StateMover()
    status = 0
    try:
        node.move(cli.state)
    except MoveItError as exc:
        node.get_logger().error(str(exc))
        status = 1
    except KeyboardInterrupt:
        status = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == '__main__':
    sys.exit(main())
