#!/usr/bin/env python3
# coding: utf-8
"""
Servo degrees <-> URDF radians for the DOFBOT arm.

One place for the angle conventions, so the driver, /joint_states and MoveIt
cannot disagree about zero offsets, signs or the gripper range.

Conventions are the vendor's (dofbot_ctrl/SimulateToArm.py), with its double
inversion removed: SimulateToArm flips joints 2/3/4 with `180 - x` before
calling write6, and Arm_Lib.Arm_serial_servo_write flips them *again*
internally, so the two cancel. Arm_Driver keeps that internal flip, so this
module must not apply a second one -- it works in the same angle space the
driver's public API takes and returns.
"""

from math import degrees, radians

# Published joints. The other five gripper joints (Rlink2/3, Llink1/2/3) mimic
# Rlink1_Joint in the URDF, so robot_state_publisher derives them from it.
ARM_JOINT_NAMES = ('arm1_Joint', 'arm2_Joint', 'arm3_Joint',
                   'arm4_Joint', 'arm5_Joint')
GRIPPER_JOINT_NAME = 'Rlink1_Joint'
JOINT_NAMES = ARM_JOINT_NAMES + (GRIPPER_JOINT_NAME,)

# Servo mid-travel, which is URDF zero for the five arm joints.
CENTER_DEG = 90.0

# Per-servo sign of the URDF angle relative to the driver's angle.
_SIGN = {1: +1.0, 2: -1.0, 3: -1.0, 4: -1.0, 5: +1.0}

# Gripper: servo 6 degrees <-> the vendor's "grip angle", which is
# Rlink1_Joint in degrees, offset by CENTER_DEG.
# Endpoints are the empirical open/closed from the servo spec
# (YB-SD15M_Bus_Servo_Protocol.csv, section C): servo 0 = fully open,
# servo 170 = fully closed. This maps onto Rlink1_Joint's full URDF limit
# [0, 1.570] and keeps commands within the servo's effective 0-170 range.
# (The vendor's SimulateToArm.py used (30, 180), which clipped the open end
# and overshot the closed stop.) Direction still needs a hardware check --
# if RViz's gripper closes when the real one opens, reverse this tuple.
_GRIP_SERVO_RANGE = (0.0, 170.0)
_GRIP_ANGLE_RANGE = (90.0, 180.0)


def _lerp(x, x0, x1, y0, y1):
    """Linear map from [x0,x1] onto [y0,y1], clamped at both ends (matching
    the np.interp the vendor used)."""
    if x <= x0:
        return y0
    if x >= x1:
        return y1
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def servo_to_urdf(servo_deg):
    """Six driver angles (degrees, servo ids 1-6) -> six URDF angles (radians),
    ordered as JOINT_NAMES. A None entry (servo did not reply) stays None."""
    out = []
    for sid in range(1, 6):
        angle = servo_deg[sid - 1]
        out.append(None if angle is None
                   else radians(_SIGN[sid] * (angle - CENTER_DEG)))
    grip = servo_deg[5]
    if grip is None:
        out.append(None)
    else:
        grip_angle = _lerp(grip, *_GRIP_SERVO_RANGE, *_GRIP_ANGLE_RANGE)
        out.append(radians(grip_angle - CENTER_DEG))
    return out


def urdf_to_servo(joint_rad):
    """Six URDF angles (radians, ordered as JOINT_NAMES) -> six driver angles
    (degrees, servo ids 1-6). Inverse of servo_to_urdf, within the gripper's
    clamped range."""
    out = [CENTER_DEG + _SIGN[sid] * degrees(joint_rad[sid - 1])
           for sid in range(1, 6)]
    grip_angle = degrees(joint_rad[5]) + CENTER_DEG
    out.append(_lerp(grip_angle, *_GRIP_ANGLE_RANGE, *_GRIP_SERVO_RANGE))
    return out
