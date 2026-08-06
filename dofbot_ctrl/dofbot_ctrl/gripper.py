#!/usr/bin/env python3
# coding: utf-8
"""
Jaw model for the DOFBOT gripper: jaw opening <-> Rlink1_Joint angle.

THE ARM HAS TWO GRIPPERS, AND THEY SWAP
---------------------------------------
The bolt-on finger extensions ([RL]link2_Link_Ext.STL) screw onto the ends of
the stock fingers and come off again with a screwdriver, so neither
configuration is "the" gripper:

    stock     0-60 mm    closes fully    the test block, not the can
    extended  50-105 mm  never shuts     the soda can, not the test block

They are near-complements. The extensions reach far enough to clear the can and
pay for it at the bottom of the range, stopping too far apart to touch the block
at all. Which objects are graspable is therefore a property of what is currently
bolted on, which is why graspable.fits() is a function and not a hand-maintained
list.

SELECTING THE PROFILE -- ONE SWITCH, BOTH SIDES
-----------------------------------------------
The environment variable DOFBOT_GRIPPER picks it, and dofbot.urdf reads THE SAME
VARIABLE through $(optenv ...) to include or drop the extension meshes.

That sharing is the whole point. If the URDF and this table disagree, every
grasp is wrong by whatever the extensions add, and it fails in the ugly way: the
arm drives into the object up to the knuckles because the planned TCP is short.
An env var was chosen over a xacro:arg because urdf_launch invokes xacro with no
arguments at all -- display, mirror and gui_teleop all go through it, so a
xacro:arg could never be overridden on those paths. Set the variable where the
nodes are launched from, on the machine that runs them.

An unrecognised value raises here at import. The URDF cannot raise, so it treats
anything that is not exactly 'stock' as extended. The two disagree deliberately:
a typo stops the Python layer dead rather than quietly planning to the wrong
fingers.

THE WIDTHS ARE A HARDWARE CALIBRATION, NOT A DERIVATION. Do not compute them
from the URDF. The real gripping face is mesh geometry on the link2/link3
coupler and no joint frame lies on it, so any FK down the Rlink1/2/3 mimic chain
is guessing where the jaw face sits.

WHY THE COUPLING IS KEPT THIS NARROW
------------------------------------
Nothing above this module depends on jaw geometry: IK, Cartesian segments, the
planning scene, attach and detach, transit and carry poses all sit above it.
Every jaw dimension in the codebase lives in this file, behind MAX_WIDTH,
MIN_WIDTH, jaw_angle_for(), jaw_width_for() and tip_offset_for(), so a swap is a
re-parameterisation rather than a rewrite. Fitting the extended fingers was
exactly that: a second table here, and the meshes.

A gripper swap touches exactly: the finger meshes/links and the
Gripping_point_Link offset in dofbot_description/urdf/dofbot.urdf; the mimic
multipliers if the linkage differs; grip_group and its open/close states in
dofbot_moveit/config/dofbot_description.srdf; _GRIP_SERVO_RANGE /
_GRIP_ANGLE_RANGE in joint_map.py; and a profile here.
"""

import os

# Each profile is (rows, calibrated). Rows are
# (Rlink1_Joint radians, jaw gap metres, tip offset metres).
#
# THE TIP OFFSETS in both profiles are derived, and legitimately so. They are the
# fingertip's position along the tool axis relative to Gripping_point_Link, from
# the arm5 -> Rlink1_Joint -> Rlink2_Joint chain applied to the outermost vertex
# of the finger mesh. The object is gripped BETWEEN THE FINGERTIPS, so that is
# where the contact is. Deriving the jaw WIDTH this way was the mistake called
# out above -- width depends on where the flat gripping face is, which is coupler
# mesh nobody has measured. The tip POSITION is just the end of the finger, and
# the mesh does know that.
#
# CAVEAT ON BOTH COLUMNS OF OFFSETS: this is the distance to the outermost
# VERTEX, i.e. the very end of the finger, while an object is held by the flat
# inner face a few mm short of that. Erring long is the safe direction -- it
# hovers back rather than driving in -- but it is worth a caliper check.

# STOCK JAWS, UNCALIBRATED. The widths are a two-point stub with a straight line
# between, which a four-bar is not; no caliper sweep has been run on this
# configuration. See _EXTENDED for the shape a real sweep produces.
#
# The offsets come off the outermost vertex of Rlink2_Link.STL and read a few mm
# long. Two independent signs say so: the extension's measured rise does not
# reconcile with these tips, and a rigid extension cannot change how far the tip
# SWINGS between open and shut, yet the extended profile's measured swing is
# smaller than this column's. Long is the safe direction -- the arm stops short
# rather than driving in -- so it stands. To fix, unbolt the extensions and
# repeat the touch-off described under _EXTENDED.
_STOCK = ((
    #  angle    gap    tip offset
    (0.0000, 0.060, 0.0117),
    (0.2000, 0.0524, 0.0177),
    (0.4000, 0.0447, 0.0234),
    (0.6000, 0.0371, 0.0287),
    (0.7850, 0.030, 0.0330),
    (1.0000, 0.0218, 0.0371),
    (1.2000, 0.0142, 0.0399),
    (1.4000, 0.0065, 0.0416),
    (1.5708, 0.000, 0.0421),
), False)

# EXTENDED FINGERS.
#
# WIDTHS are caliper-measured at COMMANDED angles, with the plastic gloves on,
# near the finger ends. Commanded is the useful direction, since this module
# answers "what do I command for this gap?". It also makes the gap column the
# FREE span: under load the object stops the jaws at its own width while the
# commanded angle carries on past, which is what DEFAULT_SQUEEZE is for.
#
# The angles are keyed to the servo range in joint_map. Change that range and
# every row here has to be re-keyed through servo position to match.
#
# The first row is NOT at the joint's lower limit. The linkage is over-centre and
# span peaks just above it, so that peak is OPEN_ANGLE, and the SRDF 'open' state
# and moveit_client.open_gripper() match it. Below it the interpolation clamps,
# which is optimistic, but the URDF's lower limit keeps the planner off that
# side. It is also the one row that is not a pure re-key, having been swept from
# past the peak, so it is the row worth spot-checking.
#
# OFFSETS are pinned at BOTH ENDS by touch-off: tool axis straight down,
# fingertips lowered onto the base plate's top face, reading tf
# base_link -> Gripping_point_Link, whose height above that face is the offset.
# The mesh supplies only the curve between those ends, affine-fitted onto them.
# THE MESH IS AN APPROXIMATION OF THE PART, NOT GROUND TRUTH -- never let it set
# an absolute here; when the two disagree the measurement wins. Its curvature is
# worth keeping even so, since a straight line between the endpoints is several
# mm out mid-travel.
#
# BACKLASH: one finger has play, so the pair is not symmetric whatever the URDF's
# mimic multiplier says. Two consequences no refinement of this table can fix:
#   1. The gap at a commanded angle has hysteresis. This was swept CLOSING and a
#      grasp also ends by closing, so the normal path matches the measurement.
#      Commanding a partial OPENING to a gap arrives from the other side and
#      lands wide.
#   2. The object does not sit on the gripper centreline -- the slack finger lags
#      and the object is pushed toward it, while scene_objects attaches it
#      symmetrically. That pose error is carried into the place, not cancelled.
_EXTENDED = ((
    #  angle    gap    tip offset
    (0.0340, 0.105, 0.0601),
    (0.1701, 0.100, 0.0649),
    (0.4251, 0.096, 0.0719),
    (0.5526, 0.095, 0.0751),
    (0.6901, 0.089, 0.0783),
    (0.8176, 0.085, 0.0810),
    (0.9339, 0.084, 0.0832),
    (1.0614, 0.080, 0.0852),
    (1.1889, 0.073, 0.0868),
    (1.3164, 0.068, 0.0880),
    (1.4326, 0.059, 0.0887),
    (1.5501, 0.053, 0.0890),
    (1.5708, 0.050, 0.0890),
), True)

_PROFILES = {'stock': _STOCK, 'extended': _EXTENDED}
DEFAULT_PROFILE = 'extended'
ENV_VAR = 'DOFBOT_GRIPPER'


class GripperError(ValueError):
    """An opening this gripper cannot produce."""


PROFILE = os.environ.get(ENV_VAR, DEFAULT_PROFILE).strip().lower() or DEFAULT_PROFILE
if PROFILE not in _PROFILES:
    raise GripperError(
        '%s=%r is not a gripper this arm has; expected one of %s. Refusing to '
        'guess: the URDF resolves an unknown value to the extended fingers, so '
        'guessing here could plan %.0f mm short of the real fingertips.'
        % (ENV_VAR, os.environ.get(ENV_VAR), sorted(_PROFILES),
           (_EXTENDED[0][0][2] - _STOCK[0][0][2]) * 1e3))

_TABLE, CALIBRATED = _PROFILES[PROFILE]

OPEN_ANGLE = _TABLE[0][0]
CLOSE_ANGLE = _TABLE[-1][0]

MAX_WIDTH = _TABLE[0][1]
# The extended fingers do not meet: at the 1.5708 stop they are still 50 mm
# apart, a real floor on what can be gripped rather than a soft limit. The stock
# jaws do close, so there this is 0.0 and only CLEARANCE keeps it off the stop.
MIN_WIDTH = _TABLE[-1][1]       # stock 0.000, extended 0.050

# Refuse anything within 3 mm of either stop. At full opening the jaws have no
# room to be positioned imprecisely and the approach clips the object instead of
# clearing it; at full closure there is no travel left to squeeze with. Rejecting
# up front beats failing halfway into a grasp.
CLEARANCE = 0.003
# Rounded to the micrometre: the subtraction lands just below the intended value
# in binary floating point, which would reject a request for exactly the limit.
SAFE_MAX_WIDTH = round(MAX_WIDTH - CLEARANCE, 6)
# Squeeze needs somewhere to go, so the smallest grippable object must sit far
# enough above the stop for grip_angle_for() to still aim below it.
SAFE_MIN_WIDTH = round(MIN_WIDTH + CLEARANCE, 6)

# How much narrower than the object to command, so the jaws actually load up
# against it instead of resting at exact contact.
#
# Not a guess: it is the squeeze in the one grasp known to work, taken from the
# angle the can was actually held at against its free span. Do not shrink it on
# the grounds that contact ought to be enough -- the compliant glove on each
# finger and the play in one of them both swallow travel before any force
# reaches the object.
DEFAULT_SQUEEZE = 0.0054


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

# The profile that is NOT fitted. There are only ever two.
OTHER_PROFILE = 'stock' if PROFILE == 'extended' else 'extended'


def _safe_bounds(rows):
    """(min, max) grippable width for a profile's rows, with clearance."""
    return (round(rows[-1][1] + CLEARANCE, 6), round(rows[0][1] - CLEARANCE, 6))


def _other_profile_hint(width):
    """', the other fingers do take it' -- or '' if neither can.

    The two profiles are near-complements and swapping them is a screwdriver, so
    an object rejected here is quite often an object the arm CAN pick up, just
    not as currently dressed. Saying so turns a dead end into an instruction.
    """
    low, high = _safe_bounds(_PROFILES[OTHER_PROFILE][0])
    if not low <= width <= high:
        return ''
    return ('; the %s gripper does take it (%.0f-%.0f mm), so this is a '
            'screwdriver away -- swap the fingers and set %s=%s'
            % (OTHER_PROFILE, low * 1e3, high * 1e3, ENV_VAR, OTHER_PROFILE))


def jaw_angle_for(width):
    """Rlink1_Joint angle (rad) that opens the jaws to `width` metres.

    Raises GripperError outside [SAFE_MIN_WIDTH, SAFE_MAX_WIDTH] -- an object
    too wide OR too narrow for the fitted profile is caught here, before any
    motion is planned to it. Which end bites depends on what is bolted on, and
    the message names the profile so the answer to "why won't it take this?" is
    "you have the other fingers on".
    """
    if width < 0.0:
        raise GripperError('negative jaw opening %.4f m' % width)
    if width > SAFE_MAX_WIDTH:
        raise GripperError(
            'needs a %.1f mm opening; the %s gripper manages %.1f mm and only '
            '%.1f mm with working clearance%s'
            % (width * 1e3, PROFILE, MAX_WIDTH * 1e3, SAFE_MAX_WIDTH * 1e3,
               _other_profile_hint(width)))
    if width < SAFE_MIN_WIDTH:
        # Two genuinely different faults share this branch. The extended fingers
        # stop 50 mm apart, so a smaller object is not gripped at all -- it sits
        # between them untouched. The stock jaws do close, so there the only
        # complaint is that there is no travel left to squeeze with.
        if MIN_WIDTH > 0.0:
            raise GripperError(
                'only %.1f mm across; the %s fingers shut to %.1f mm, so it '
                'would pass straight between them%s'
                % (width * 1e3, PROFILE, MIN_WIDTH * 1e3,
                   _other_profile_hint(width)))
        raise GripperError(
            'only %.1f mm across; the %s jaws close on it but leave no room to '
            'squeeze, which needs %.1f mm'
            % (width * 1e3, PROFILE, SAFE_MIN_WIDTH * 1e3))
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
    outward along the tool axis as it closes, so the planned TCP is only in the
    right place at one opening. Ignoring it drives the arm into the object up
    to the knuckles, and getting the PROFILE wrong misses by the whole length of
    the extensions.

    Do not replace this with a search for the smallest backoff that is merely
    collision-free. That finds the deepest reach that does not trip a contact,
    which is not the same thing as gripping at the tips. It is a lookup.
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
    # Validate the OBJECT, then clamp the squeezed target to the stop. Passing
    # width - squeeze straight through would reject objects just above
    # SAFE_MIN_WIDTH for being too narrow, when it is the deliberate overshoot
    # that went under the limit, not the object.
    jaw_angle_for(width)
    return _interp(max(MIN_WIDTH, width - squeeze), _R_WIDTHS, _R_ANGLES)


def fits(width):
    """True if this gripper can grip `width`, with clearance at both stops."""
    return SAFE_MIN_WIDTH <= width <= SAFE_MAX_WIDTH


def describe(width):
    """Why an object does or does not fit. For error messages and CLI output."""
    try:
        angle = jaw_angle_for(width)
    except GripperError as exc:
        return str(exc)
    note = '' if CALIBRATED else (
        ' (UNCALIBRATED: %s widths are interpolated on a two-point table)'
        % PROFILE)
    return ('%.1f mm needs Rlink1_Joint = %.3f rad on the %s gripper%s'
            % (width * 1e3, angle, PROFILE, note))
