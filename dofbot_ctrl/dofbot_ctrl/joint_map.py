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

# Servo mid-travel, the nominal URDF zero for the five arm joints.
CENTER_DEG = 90.0

# Per-servo zero-offset (servo degrees): the servo command that puts a joint at
# true mechanical zero is CENTER_DEG + this. Corrects horns/servo zero not
# landing exactly at 90. Measured 2026-07-18 by posing the arm straight up with
# torque off and reading the encoders (reading - 90); stable to 0 spread, and
# 2/3/4 = +2 reproduced across two independent poses. Servo 1 (base) checked
# with the arm bent over and needs no adjustment.
_ZERO_OFFSET = {1: 0.0, 2: 2.0, 3: 2.0, 4: 2.0, 5: -2.0}

# Per-servo sign of the URDF angle relative to the driver's angle.
_SIGN = {1: +1.0, 2: -1.0, 3: -1.0, 4: -1.0, 5: +1.0}


def _center(sid):
    """Servo angle (degrees) that corresponds to this joint's URDF zero."""
    return CENTER_DEG + _ZERO_OFFSET[sid]

# Gripper: servo 6 degrees <-> the vendor's "grip angle", which is
# Rlink1_Joint in degrees, offset by CENTER_DEG.
# Endpoints are the empirical open/closed from the servo spec
# (YB-SD15M_Bus_Servo_Protocol.csv, section C). Direction verified on hardware:
# a low servo value is open and a high one is shut, so close_gripper() closes.
# Do not reverse this tuple.
#
# THE OPEN END IS NOT THE SERVO'S ZERO, AND THE REASON IS MECHANICAL. The linkage
# is over-centre: the gear arms sweep up to parallel with the gripper body, which
# is the WIDEST opening, and the servo can be driven past that point, at which
# the arms tip up the other way and the jaws start closing again. Span is not
# monotonic in servo value; it peaks at parallel. The open end of this range is
# that peak, found by reading /joint_states with the gear arms visibly parallel.
#
# Pinning the peak to 0 rad matters beyond display. Rlink1_Joint's URDF lower
# limit is 0, so that limit then lands on the peak and the planner cannot command
# past it into the narrowing side.
#
# gripper.jaw_angle_for() also inverts span -> angle assuming span falls
# monotonically as the angle grows, which is only true on this side of the peak.
# That is a second reason 0 has to BE the peak.
#
# The jaw GEOMETRY -- how far apart the jaws are at a given Rlink1_Joint angle --
# is a separate calibration in gripper.py, and its table is keyed to this range.
_GRIP_SERVO_RANGE = (33.0, 169.0)
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
                   else radians(_SIGN[sid] * (angle - _center(sid))))
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
    out = [_center(sid) + _SIGN[sid] * degrees(joint_rad[sid - 1])
           for sid in range(1, 6)]
    grip_angle = degrees(joint_rad[5]) + CENTER_DEG
    out.append(_lerp(grip_angle, *_GRIP_ANGLE_RANGE, *_GRIP_SERVO_RANGE))
    return out
