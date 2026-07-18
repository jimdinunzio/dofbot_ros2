#!/usr/bin/env python3
# coding: utf-8
"""
Per-servo zero-offset calibration for the DOFBOT arm, by encoder read.

Disables torque so you can pose the arm by hand, then reads the servos and
reports the offset for each (median reading - 90). Posing and reading beats
jogging the servo by small commands, which lie inside the servo's dead band.

The offsets go into joint_map._ZERO_OFFSET. You can read as many times as you
like (e.g. straight-up for joints 2-5, then a bent pose to judge the base),
and take each joint's offset from whichever pose shows it best.

Owns /dev/ttyTHS1. Stop the mirror and gui_teleop first -- nothing else may
hold the port. The arm goes limp when torque drops, so support it.

Run:  ros2 run dofbot_ctrl calibrate_zero
"""

import statistics
import time

from Arm_Lib import Arm_Device

_CENTER = 90.0
_SAMPLES = 9
_CALIBRATABLE = (1, 2, 3, 4, 5)  # servo 6 (gripper) has its own range calibration


def read_offsets(arm):
    samples = {sid: [] for sid in range(1, 7)}
    for _ in range(_SAMPLES):
        for sid in range(1, 7):
            v = arm.Arm_serial_servo_read(sid)
            if v is not None:
                samples[sid].append(v)
        time.sleep(0.05)

    print('  servo  n   median  spread   offset')
    offsets = {}
    for sid in range(1, 7):
        s = samples[sid]
        if not s:
            print('    %d    0    ----    ----    (no reply)' % sid)
            continue
        med = statistics.median(s)
        offsets[sid] = round(med - _CENTER, 1)
        tag = '   (gripper)' if sid == 6 else ''
        print('    %d   %2d   %6.1f   %4d    %+.1f%s'
              % (sid, len(s), med, max(s) - min(s), med - _CENTER, tag))
    return offsets


def main():
    arm = Arm_Device(com='/dev/ttyTHS1')
    print(__doc__)
    input('Press Enter to DISABLE torque (arm goes limp -- support it)... ')
    arm.Arm_serial_set_torque(0)
    print('Torque off.\n')

    try:
        while True:
            input('Pose the arm to true zero, hold steady, press Enter to read... ')
            offsets = read_offsets(arm)
            print('\n  _ZERO_OFFSET = {%s}   (servo 6 excluded)\n'
                  % ', '.join('%d: %s' % (sid, offsets.get(sid, 0.0))
                              for sid in _CALIBRATABLE))
            if input('Read again from another pose? [y/N] ').strip().lower() != 'y':
                break
    finally:
        arm.Arm_serial_set_torque(1)
        print('Torque re-enabled -- arm holds its current pose.')


if __name__ == '__main__':
    main()
