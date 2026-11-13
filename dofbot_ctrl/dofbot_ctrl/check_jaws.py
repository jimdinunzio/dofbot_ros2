#!/usr/bin/env python3
# coding: utf-8
"""
Exercise the EMPTY jaws and prove the gripper servo follows, with the stack DOWN.

    ros2 run dofbot_ctrl check_jaws

Opens, closes to the stop and opens again, reading servo 6 back after each move
and comparing it with what was commanded. Exits 0 if the jaws followed, 1 if
they did not, 2 if the servo never answered at all.

WHY: execution is open loop. moveit_bridge writes positions and nothing reads
them back, so a gripper servo that has stopped responding looks exactly like a
working one until a can is dropped or never picked up. On 2026-11-10 the jaws
did just that: unresponsive until torque was released, the jaws moved by hand
and the arm enabled again. This finds that state before a pick does.

IT DOES NOT CALIBRATE ANYTHING. There is no force or contact sensing at the
jaw, so the squeeze an object gets is still graspable's number; this only
says whether the servo goes where it is told.

RECOVERY is the hand fix, minus the hand: torque off on servo 6, back on, and
try once more. A servo that has latched into stall protection after a long
squeeze is released by that. If the retry fails too it needs a person.

It OPENS FIRST, so anything in the jaws is dropped where the arm stands.
arm_server runs it at enable_arm, where that costs nothing new: the launch's
first sync drives the jaws to the model's open pose anyway.

Needs the port to itself, like set_torque: refuses while anything holds it.
"""

import argparse
import sys
import time

from Arm_Lib import Arm_Device
from Arm_Lib.arm_driver import _ADDR_TORQUE, _CMD_WRITE

from dofbot_ctrl import gripper
from dofbot_ctrl.joint_map import urdf_to_servo
from dofbot_ctrl.serial_port import port_rivals

PORT = '/dev/ttyTHS1'
GRIP_ID = 6

# Servo degrees the read-back may miss the target by. Empty extended jaws
# read 163-168 against a 169 close (2026-11-10, five runs), so 6 was too tight;
# a jaw that did not move misses by ~130, so there is room to spare.
TOLERANCE = 10.0

# Move times, ms. Matched to moveit_bridge's open_time_ms / grip_time_ms
# defaults: the check exercises the jaws at the speed a pick does.
OPEN_MS = 400
CLOSE_MS = 1000

# How long past the move time to keep reading before calling it stuck. Read
# repeatedly rather than once after a fixed sleep, so a healthy servo passes
# as soon as it arrives and only a stuck one costs the full wait.
SETTLE_S = 1.0
POLL_S = 0.05

# Seconds to wait after a torque packet before commanding a position. The
# servo DROPS a position write that arrives a few ms after torque-on --
# measured 2026-11-10, 3 of 3 closes ignored at a 3 ms gap, 0 of 3 at 300 ms.
# Without this wait the first version of this check failed its own close.
TORQUE_SETTLE_S = 0.3


def _servo_for(angle):
    """Servo-6 degrees for a Rlink1_Joint angle, through joint_map."""
    return urdf_to_servo([0.0] * 5 + [angle])[5]


def _move(arm, target, ms):
    """Command servo 6 to `target` and wait for it. (ok, last reading)."""
    arm.Arm_serial_servo_write(GRIP_ID, target, ms)
    deadline = time.monotonic() + ms / 1000.0 + SETTLE_S
    reading = None
    while time.monotonic() < deadline:
        time.sleep(POLL_S)
        r = arm.Arm_serial_servo_read_f(GRIP_ID, retries=3)
        if r is None:
            continue
        reading = r
        if abs(reading - target) <= TOLERANCE:
            return True, reading
    return False, reading


def _exercise(arm, open_deg, close_deg):
    """Open, close, open. Returns a list of failure lines, empty if it passed."""
    failures = []
    for name, target, ms in (('open', open_deg, OPEN_MS),
                             ('close', close_deg, CLOSE_MS),
                             ('open', open_deg, OPEN_MS)):
        ok, reading = _move(arm, target, ms)
        print('  %-5s -> %5.1f servo deg: read %s %s'
              % (name, target,
                 '  none' if reading is None else '%5.1f' % reading,
                 'ok' if ok else 'MISSED'), flush=True)
        if not ok:
            failures.append('%s to %.1f read back %s'
                            % (name, target, 'nothing' if reading is None
                               else '%.1f' % reading))
    return failures


def _torque_cycle(arm):
    """Torque off then on for servo 6 alone, each sent twice (unacknowledged).

    Arm_serial_set_torque addresses all six, which would drop the arm; the
    gripper is the only servo in question, so only it is touched.
    """
    for val in (0, 1):
        for _ in range(2):
            arm._send(GRIP_ID, _CMD_WRITE, _ADDR_TORQUE, [val])
            time.sleep(0.003)
        time.sleep(TORQUE_SETTLE_S)


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog='check_jaws', description=__doc__.split('\n\n')[1])
    parser.add_argument('--port', default=PORT)
    cli = parser.parse_args(argv)

    rivals = port_rivals(cli.port)
    if rivals:
        print('%s is held by %s; check_jaws needs the stack down.'
              % (cli.port, ', '.join(rivals)), file=sys.stderr)
        return 1

    arm = Arm_Device(com=cli.port)
    try:
        if arm.Arm_serial_servo_read_f(GRIP_ID, retries=5) is None:
            print('JAWS: no reply from the gripper servo -- arm powered off, '
                  'or the cable at the end of the chain', flush=True)
            return 2

        open_deg = _servo_for(gripper.OPEN_ANGLE)
        close_deg = _servo_for(gripper.CLOSE_ANGLE)
        for _ in range(2):
            arm._send(GRIP_ID, _CMD_WRITE, _ADDR_TORQUE, [1])
            time.sleep(0.003)
        time.sleep(TORQUE_SETTLE_S)

        print('checking the empty jaws (%s fingers)' % gripper.PROFILE,
              flush=True)
        failures = _exercise(arm, open_deg, close_deg)
        if not failures:
            print('JAWS: ok', flush=True)
            return 0

        print('jaws did not follow; cycling gripper torque and retrying',
              flush=True)
        _torque_cycle(arm)
        retry = _exercise(arm, open_deg, close_deg)
        if not retry:
            print('JAWS: ok after a torque cycle (first try: %s)'
                  % '; '.join(failures), flush=True)
            return 0
        print('JAWS: stuck -- %s. Torque off (set_torque off), work the '
              'jaws by hand, and enable again' % '; '.join(retry), flush=True)
        return 1
    finally:
        arm.ser.close()


if __name__ == '__main__':
    sys.exit(main())
