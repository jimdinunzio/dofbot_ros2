#!/usr/bin/env python3
# coding: utf-8
"""
Move the arm to one of the saved states, from the command line.

    ros2 run dofbot_ctrl move_to_state stow
    ros2 run dofbot_ctrl move_to_state init
    # let go of whatever is held, then park -- what a shutdown runs
    ros2 run dofbot_ctrl move_to_state --open-gripper init
    # open the jaws and move nothing
    ros2 run dofbot_ctrl move_to_state --open-gripper
    # names, joint values and where the tool ends up -- needs nothing running
    ros2 run dofbot_ctrl move_to_state --list
    # which saved state is the arm in now? prints `STATE: <name>` or
    # `STATE: none`; needs the stack up, moves nothing
    ros2 run dofbot_ctrl move_to_state --where

The same states RViz's MoveIt panel offers, and the same planning: a joint-space
goal to move_group, planned by OMPL and collision-checked against the live
planning scene. What this adds over the panel is that it scripts, it logs, and
it is the same code path a pick takes to 'stow'.

The states live in moveit_client.NAMED_STATES and are mirrored into
dofbot_description.srdf, which is where the RViz panel reads them from.
test/test_named_states.py parses the SRDF and fails if the two disagree -- they
are two copies of the same numbers, and they have drifted before.

--open-gripper DROPS WHATEVER IS HELD, where it is, before any move. The order
is deliberate and is the one `pick_place --reset` uses: parking first would
carry the object up and drop it from there instead. It also detaches and
removes whatever the planning scene has attached, as `pick_place --drop` does
-- once the jaws are open the object is on the floor, and a scene still
hanging it from the gripper would carry a phantom can into every move after.

A HELD OBJECT TRAVELS WITH THE ARM, so a state can be unreachable only because
of it: 'init' folds the tool low, and a can hanging from it goes into the
floor. That fails as a goal in collision naming the object, and the fix is to
let go first -- --open-gripper here, or `pick_place --place` -- not to change
the state.

OTHERWISE THIS DOES NOT TOUCH THE PLANNING SCENE, and that is the difference
between it and `pick_place --reset`. A move that fails immediately with
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
from dofbot_ctrl import scene_markers
from dofbot_ctrl.moveit_client import (NAMED_STATES, DofbotMoveIt,
                                       MoveItError, ros_begin, ros_end)

# The one line --where prints for a program to read. Everything else it prints
# is for a human.
STATE_PREFIX = 'STATE: '

# How close every joint must be to a saved state to count as in it, radians.
# /joint_states is ros2_control's mock hardware, i.e. the commanded position,
# so an arm that finished a move to a state is on it exactly; this only absorbs
# float noise. About half a degree -- far below the gap between any two states.
STATE_TOLERANCE = 0.01


def state_of(joints, tolerance=STATE_TOLERANCE):
    """The saved state `joints` is in, or None if it is in none of them.

    Every joint must be within `tolerance`. If two states ever overlapped at
    that tolerance the nearer one wins, but none do.
    """
    best, best_err = None, None
    for name, target in NAMED_STATES.items():
        err = max(abs(q - t) for q, t in zip(joints, target))
        if err <= tolerance and (best_err is None or err < best_err):
            best, best_err = name, err
    return best


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

    def open_gripper(self):
        """Open the jaws, then forget what the scene says they were holding.

        Scene AFTER jaws, as in pick_place.drop(): a failed open leaves the
        scene still saying what the gripper holds, which is the truth.
        """
        self.mc.open_gripper()
        _world, held = self.mc.object_ids()
        for name in held:
            self.get_logger().info('dropped %r: detaching it and removing it '
                                   'from the scene' % name)
            self.mc.detach(name)
            self.mc.remove(name)
        if held:
            # Marker ids are per process, so DELETEALL is the only way to
            # take down what pick_place drew.
            scene_markers.MeshMarkers(self).clear()

    def move(self, name):
        try:
            self.mc.move_named(name)
        except MoveItError as exc:
            _world, held = self.mc.object_ids()
            hit = [h for h in held if h in str(exc)]
            if not hit:
                raise
            raise MoveItError(
                '%s -- %r is held, and %r puts it there. Let go first: '
                'move_to_state --open-gripper %s drops it where the arm is, '
                'pick_place --place puts it over the trash'
                % (exc, hit[0], name, name)) from exc
        # FK on the live joints, so this reports where the arm ACTUALLY ended
        # up rather than reprinting the target. On mock joints those are the
        # same thing; on the real arm they are not, and execution is open-loop.
        x, y, z, phi, roll = self.mc.current_pose()
        self.get_logger().info(
            'at %r: tcp (%.3f, %.3f, %.3f) phi=%.3f roll=%.3f'
            % (name, x, y, z, phi, roll))

    def where(self):
        """Print which saved state the arm is in, as STATE_PREFIX + name."""
        joints = self.mc.current_joints()
        name = state_of(joints)
        self.get_logger().info('joints %s: %s'
                               % (['%.3f' % q for q in joints],
                                  'at %r' % name if name
                                  else 'not at any saved state'))
        print('%s%s' % (STATE_PREFIX, name or 'none'), flush=True)
        return name


def main(args=None):
    parser = argparse.ArgumentParser(
        prog='move_to_state', description=__doc__.split('\n\n')[0])
    parser.add_argument('state', nargs='?',
                        help='saved state to move to (%s)'
                             % ', '.join(sorted(NAMED_STATES)))
    parser.add_argument('--list', action='store_true',
                        help='print the saved states and exit, without '
                             'connecting to move_group')
    parser.add_argument('--open-gripper', action='store_true',
                        help='open the jaws first, dropping whatever is held '
                             'where it is. STATE becomes optional: on its own '
                             'this lets go and moves nothing')
    parser.add_argument('--where', action='store_true',
                        help="print which saved state the arm is in, as "
                             "'STATE: <name>' or 'STATE: none'; moves nothing")
    cli = parser.parse_args(remove_ros_args(sys.argv)[1:])

    # Both of these are settled before rclpy.init, so a typo costs nothing and
    # says what the choices are rather than timing out on a service call.
    if cli.list:
        print_states()
        return 0
    if cli.where and (cli.state is not None or cli.open_gripper):
        print('--where moves nothing; give it no state and no --open-gripper',
              file=sys.stderr)
        return 2
    if cli.state is None and not cli.open_gripper and not cli.where:
        parser.print_usage(sys.stderr)
        print('give a state name: %s' % ', '.join(sorted(NAMED_STATES)),
              file=sys.stderr)
        return 2
    if cli.state is not None and cli.state not in NAMED_STATES:
        print('unknown state %r; have %s'
              % (cli.state, ', '.join(sorted(NAMED_STATES))), file=sys.stderr)
        return 2

    began = ros_begin(args)
    node = StateMover()
    status = 0
    try:
        if cli.where:
            node.where()
        # Jaws before wheels: see --open-gripper in the docstring.
        if cli.open_gripper:
            node.open_gripper()
        if cli.state is not None:
            node.move(cli.state)
    except MoveItError as exc:
        node.get_logger().error(str(exc))
        status = 1
    except KeyboardInterrupt:
        status = 130
    finally:
        node.destroy_node()
        ros_end(began)
    return status


if __name__ == '__main__':
    sys.exit(main())
