#!/usr/bin/env python3
# coding: utf-8
"""
Run the arm's commands one after another in one process that stays up.

    ros2 run dofbot_ctrl arm_worker

arm_server starts this when the arm is enabled and stops it when it is
disabled. It is not for a terminal: by hand, run the commands themselves
(`ros2 run dofbot_ctrl pick_place -- --place`), which work exactly as before.

WHY IT EXISTS. A command run as its own process spent 3.5-4.5 s before the arm
was asked to do anything, measured 2026-11-12: 0.45 s in `ros2 run`, 1.3 s
importing and building a node, and 1.5-3 s for that node's new DDS participant
to find move_group. Planning the move took 0.3 s. Here the imports are done
once and one context, kept alive by a node that does nothing else, has already
found move_group; a command starts moving the arm in a few tenths of a second.

EACH COMMAND STILL GETS A NODE OF ITS OWN, built and destroyed by the same
main() that `ros2 run` calls. That is deliberate: those programs keep what
they know in the planning scene and nothing in themselves, because each used
to be a fresh process, and a node that lived across commands would be the
first thing able to remember a can the scene has forgotten.

THE WIRE, over this process's own stdin and stdout:

    stdin    one JSON object per line: {"id": "<token>", "argv": ["pick_place",
             "--pick", "0.3", "0", "0.035"]}. argv[0] is one of COMMANDS.
    stdout   READY once, when a command would start at once; then, per command,
             everything the command printed, followed by
             DONE_PREFIX + "<token> <exit status>".

stderr carries the ROS log and is expected to be merged into stdout by whoever
started this, as arm_server does. The token is the caller's, so a command that
prints something shaped like the last line cannot end itself early.

TO ABORT A COMMAND, SIGINT THIS PROCESS. The command unwinds as it does under
Ctrl-C and the process exits 130 WITHOUT a DONE line, because the signal shuts
the context down with it. Start another.
"""

import json
import sys
import traceback

import rclpy
from moveit_msgs.srv import GetPlanningScene
from rclpy.node import Node

from dofbot_ctrl import move_to_state, pick_place, vision_check, wave_arm

# What may be asked for, by the name its `ros2 run` executable has.
COMMANDS = {
    'pick_place': pick_place.main,
    'move_to_state': move_to_state.main,
    'wave_arm': wave_arm.main,
    'vision_check': vision_check.main,
}

READY = 'ARM_WORKER_READY'
DONE_PREFIX = 'ARM_WORKER_DONE '

# The service whose appearance says move_group has been found. Any of its
# services would do; this is the first one a pick calls.
PROBE_SERVICE = '/get_planning_scene'

# Seconds to wait for it before saying READY anyway. The stack is up before
# this is started, so the wait is the 1.5-3 s of discovery and no more; past
# this the first command waits for move_group itself, as it always did.
PROBE_TIMEOUT = 15.0


def run(argv):
    """One command, as its exit status. Never raises, bar KeyboardInterrupt
    escaping a command that does not catch it."""
    if not argv or argv[0] not in COMMANDS:
        print('arm_worker: no such command %r; have %s'
              % (argv[0] if argv else '', ', '.join(sorted(COMMANDS))),
              file=sys.stderr)
        return 2
    # The entry points read sys.argv, as they must under `ros2 run`.
    sys.argv = [str(a) for a in argv]
    try:
        if argv[0] == 'vision_check':
            return int(COMMANDS[argv[0]](sys.argv[1:]) or 0)
        return int(COMMANDS[argv[0]]() or 0)
    except SystemExit as exc:
        # argparse rejecting a command line, or --help.
        return exc.code if isinstance(exc.code, int) else 1
    except KeyboardInterrupt:
        return 130
    except Exception:
        traceback.print_exc()
        return 1


def main():
    # A pipe is block-buffered by default, and the caller is waiting on lines.
    sys.stdout.reconfigure(line_buffering=True)
    rclpy.init()
    # Holds the DDS participant open between commands. Without a node the
    # context has none, and every command would rediscover move_group.
    anchor = Node('arm_worker', start_parameter_services=False)
    probe = anchor.create_client(GetPlanningScene, PROBE_SERVICE)
    if not probe.wait_for_service(timeout_sec=PROBE_TIMEOUT):
        print('arm_worker: %s not found in %.0fs; the first command will '
              'wait for it' % (PROBE_SERVICE, PROBE_TIMEOUT), file=sys.stderr)
    anchor.destroy_client(probe)
    print(READY)
    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                request = json.loads(line)
                token, argv = str(request['id']), list(request['argv'])
            except (ValueError, KeyError, TypeError):
                print('arm_worker: not a request: %r' % line, file=sys.stderr)
                continue
            status = run(argv)
            sys.stderr.flush()
            if not rclpy.ok():
                # SIGINT shut the context down under the command, and nothing
                # more can run on it. No DONE line: the caller must not take
                # this process for one that can be handed the next command,
                # and the exit status says what happened to this one.
                return 130
            print('%s%s %d' % (DONE_PREFIX, token, status))
    except KeyboardInterrupt:
        return 130
    finally:
        anchor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
