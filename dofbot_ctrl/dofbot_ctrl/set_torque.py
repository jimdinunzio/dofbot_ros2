#!/usr/bin/env python3
# coding: utf-8
"""
Switch servo torque on or off, with the ROS stack DOWN.

    ros2 run dofbot_ctrl set_torque off    # the arm goes limp -- support it
    ros2 run dofbot_ctrl set_torque on     # it holds wherever it now is

TORQUE OFF MEANS THE ARM IS DISABLED, so this refuses to run while anything
else holds the servo port -- moveit_bridge above all. A limp arm under a
running bridge is the state that left the base 16 degrees off on 2026-10-23:
MoveIt and the bridge kept believing a pose the arm was no longer in, and the
next move would have snapped it back at full tracking speed. Disable the arm
first (arm_client disable, or stop the launch); enabling it again brings the
bridge up, and the bridge turns torque back on and eases the arm to its model
pose on the way.

Torque packets are unacknowledged, so each is sent twice. 'on' leaves the
servos holding where they stand; it does not move anything.
"""

import argparse
import sys

from Arm_Lib import Arm_Device

from dofbot_ctrl.serial_port import port_rivals

PORT = '/dev/ttyTHS1'


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog='set_torque', description=__doc__.split('\n\n')[0])
    parser.add_argument('state', choices=('on', 'off'))
    parser.add_argument('--port', default=PORT)
    cli = parser.parse_args(argv)

    rivals = port_rivals(cli.port)
    if rivals:
        print('%s is held by %s. Torque off means the arm is disabled: stop '
              'the stack first (arm_client disable). With the stack up, its '
              'bridge already keeps torque on.'
              % (cli.port, ', '.join(rivals)), file=sys.stderr)
        return 1

    arm = Arm_Device(com=cli.port)
    for _ in range(2):
        arm.Arm_serial_set_torque(1 if cli.state == 'on' else 0)
    print('torque %s%s' % (cli.state,
                           ' -- the arm is limp; support it'
                           if cli.state == 'off' else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
