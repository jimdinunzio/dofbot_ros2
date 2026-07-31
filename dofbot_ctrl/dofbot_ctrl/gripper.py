#!/usr/bin/env python3
# coding: utf-8
"""
Jaw model for the DOFBOT gripper: jaw opening <-> Rlink1_Joint angle.

THIS IS A HARDWARE CALIBRATION, NOT A DERIVATION. Do not compute it from the
URDF. An earlier attempt to do exactly that -- forward kinematics down the
Rlink1/2/3 mimic chain -- produced a 25.8-85 mm range, which is wrong at both
ends. The FK was fine; the error was assuming the fingertip contact point sits
30 mm along Rlink2_Link (30 mm is the length of link 1, and has nothing to do
with where the jaw face is). The real gripping face is mesh geometry on the
link2/link3 coupler, and no joint frame in the URDF lies on it.

What is actually known, measured on the robot:

    Rlink1_Joint = 0.0    (SRDF 'open')   jaws  60 mm apart
    Rlink1_Joint = 1.5708 (SRDF 'close')  jaws   0 mm apart, fully shut

CALIBRATION PROCEDURE (to replace the straight line between those two points)
----------------------------------------------------------------------------
The linkage is a four-bar, so gap versus angle is a cosine-ish curve, NOT the
line interpolated below. To measure it:

  1. Stop moveit_bridge and every other holder of /dev/ttyTHS1.
  2. Command Rlink1_Joint in ~0.15 rad steps from 0 to 1.5708 (gui_teleop, or
     `ros2 action send_goal /grip_group_controller/gripper_cmd ...`).
  3. At each step measure the gap between the jaw faces with calipers, at the
     height where an object would actually be held.
  4. Also measure `tip_offset`: how far the contact point sits from
     Gripping_point_Link along the tool axis at that opening. It migrates as
     the jaws close, while Gripping_point_Link is a FIXED frame 68.09 mm out,
     so the planned TCP is only exactly right at one opening. This is the
     quantity Yahboom absorbs into hand-tuned -0.01/+0.01/+0.02 constants;
     dofbot_sorting/grasp.py admits as much ("the grasp point is set 1 cm behind
     the midpoint because of the gripper end position").
  5. Paste the rows into _TABLE, set CALIBRATED = True, and drop this note.

WHY THE COUPLING IS KEPT THIS NARROW
------------------------------------
A 355 ml can is 66 mm across against a 60 mm maximum opening, so the stock jaws
cannot clear the can body at all -- on approach the tips strike the wall and
knock it over. The gripper is expected to be replaced. Nothing above this module
depends on jaw geometry: IK, Cartesian segments, the planning scene, attach and
detach, transit and carry poses all sit above it. Every jaw dimension in the
codebase lives in this file, behind MAX_WIDTH, jaw_angle_for() and
tip_offset_for(), so a swap is a re-parameterisation rather than a rewrite.

A gripper swap touches exactly: the finger meshes/links and the
Gripping_point_Link offset in dofbot_description/urdf/dofbot.urdf; the mimic
multipliers if the linkage differs; grip_group and its open/close states in
dofbot_moveit/config/dofbot_description.srdf; _GRIP_SERVO_RANGE /
_GRIP_ANGLE_RANGE in joint_map.py; and _TABLE here.
"""

# False until the sweep in the docstring has actually been done. Callers can
# check it to decide whether to trust a mid-range opening.
CALIBRATED = False

# (Rlink1_Joint radians, jaw gap metres, tip offset metres).
# Only the endpoints are measured. The middle is a straight line and the real
# linkage is not linear -- that is what step 5 above fixes. tip_offset is 0.0
# throughout because it has not been measured yet, not because it is zero.
_TABLE = (
    (0.0000, 0.060, 0.0),
    (1.5708, 0.000, 0.0),
)

OPEN_ANGLE = _TABLE[0][0]
CLOSE_ANGLE = _TABLE[-1][0]

MAX_WIDTH = _TABLE[0][1]        # 0.060 m, jaws wide open

# Refuse anything within 3 mm of the stop. At full opening the jaws have no room
# to be positioned imprecisely, and the approach clips the object instead of
# clearing it. Rejecting up front beats failing halfway into a grasp.
CLEARANCE = 0.003
# Rounded to the micrometre: 0.060 - 0.003 is 0.056999999999999995 in binary
# floating point, which would reject a request for exactly 57 mm.
SAFE_MAX_WIDTH = round(MAX_WIDTH - CLEARANCE, 6)        # 0.057 m

# How much narrower than the object to command, so the jaws actually load up
# against it instead of resting at exact contact.
DEFAULT_SQUEEZE = 0.004


class GripperError(ValueError):
    """An opening this gripper cannot produce."""


def _interp(x, xs, ys):
    """Piecewise-linear interpolation over a table sorted ascending in x."""
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            span = xs[i] - xs[i - 1]
            f = 0.0 if span == 0 else (x - xs[i - 1]) / span
            return ys[i - 1] + f * (ys[i] - ys[i - 1])
    return ys[-1]


_ANGLES = [row[0] for row in _TABLE]
_WIDTHS = [row[1] for row in _TABLE]
_OFFSETS = [row[2] for row in _TABLE]
# Widths run the other way from angles, so the width-keyed lookups need the
# table reversed to stay ascending.
_R_WIDTHS = list(reversed(_WIDTHS))
_R_ANGLES = list(reversed(_ANGLES))
_R_OFFSETS = list(reversed(_OFFSETS))


def jaw_angle_for(width):
    """Rlink1_Joint angle (rad) that opens the jaws to `width` metres.

    Raises GripperError outside [0, SAFE_MAX_WIDTH] -- an object too wide for
    this gripper is caught here, before any motion is planned to it.
    """
    if width < 0.0:
        raise GripperError('negative jaw opening %.4f m' % width)
    if width > SAFE_MAX_WIDTH:
        raise GripperError(
            'needs a %.1f mm opening; this gripper manages %.1f mm and only '
            '%.1f mm with working clearance'
            % (width * 1e3, MAX_WIDTH * 1e3, SAFE_MAX_WIDTH * 1e3))
    return _interp(width, _R_WIDTHS, _R_ANGLES)


def jaw_width_for(angle):
    """Inverse of jaw_angle_for: the gap at a given Rlink1_Joint angle."""
    return _interp(angle, _ANGLES, _WIDTHS)


def tip_offset_for(width):
    """Where the jaw faces grip, relative to Gripping_point_Link, in metres
    along the tool axis. POSITIVE means past the TCP frame, further from the
    wrist -- so a positive offset means the TCP must sit that far BACK from the
    contact point, which at a downward grasp pitch is "hover a bit higher".

    The offset is a function of opening because the four-bar swings the fingers
    outward along the tool axis as it closes: measured off the URDF at zero
    pitch, Rlink2_Link's origin travels from z = 0.398 wide open to z = 0.429
    fully shut, while Gripping_point_Link stays put at 0.437. So the planned TCP
    is only in the right place at one opening.

    Returns 0.0: uncalibrated, see the module docstring. pick_place does NOT
    rely on that zero -- it searches the hover distance against the live scene
    and logs what it found, which is also the number to compare your calipers
    against when you do calibrate.
    """
    return _interp(width, _R_WIDTHS, _R_OFFSETS)


def grip_angle_for(width, squeeze=DEFAULT_SQUEEZE):
    """The angle to COMMAND to hold an object `width` across.

    Commands the jaws slightly past contact so they load against the object.
    The servo stalls there, which is the normal and expected end of a grasp.

    Do not collision-check this angle: the object stops the jaws at
    jaw_angle_for(width), so the commanded squeeze angle is a pose the gripper
    never reaches while it has hold of something.
    """
    return jaw_angle_for(max(0.0, width - squeeze))


def fits(width):
    """True if this gripper can open wide enough for `width`, with clearance."""
    return 0.0 <= width <= SAFE_MAX_WIDTH


def describe(width):
    """Why an object does or does not fit. For error messages and CLI output."""
    try:
        angle = jaw_angle_for(width)
    except GripperError as exc:
        return str(exc)
    note = '' if CALIBRATED else ' (UNCALIBRATED: interpolated on a two-point table)'
    return ('%.1f mm needs Rlink1_Joint = %.3f rad%s'
            % (width * 1e3, angle, note))
