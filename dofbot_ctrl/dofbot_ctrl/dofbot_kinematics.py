#!/usr/bin/env python3
# coding: utf-8
"""
Closed-form forward and inverse kinematics for the DOFBOT arm.

Pure math -- no ROS imports -- so it is unit-testable offline and reusable by the
perception/LLM layer that will eventually call pick(x, y, z).

WHY THIS EXISTS
---------------
The arm is 5-DOF: yaw, three pitches, roll. It cannot reach an arbitrary 6-DOF
pose, because the approach azimuth is locked to the base yaw -- the reachable
poses form a manifold, and most orientations are simply not on it.

This is NOT a claim that MoveIt's solver is broken. It is not: measured with
/compute_ik on 120 exactly-reachable goals, the stock KDL plugin solves 116 at
the 5 ms timeout it shipped with and 119 at 0.5 s, and /compute_cartesian_path
returns fraction = 1.000 on the segments the pick sequence uses. Feed it a pose
the arm can actually strike and it does the job.

The problem is upstream of the solver: to hand MoveIt a pose goal you must
already know an achievable quaternion, and working one out is this arm's
kinematics. What the arm can reach is position (x, y, z) + tool pitch + tool
roll -- 5 numbers for 5 joints -- and that parameterisation admits a closed-form
solution, so nothing has to be searched for or guessed. That is what this module
provides, and why it is the interface the rest of dofbot_ctrl is written
against. It is also exact, branch-aware, limit-aware and fast enough to run at
every waypoint of a Cartesian segment.

CONVENTIONS
-----------
Frame is base_link: +x forward, +z up, origin at the underside of the arm's bottom
mounting plate.

    phi   tool tilt measured from VERTICAL, = theta2 + theta3 + theta4.
          phi = 0     -> tool points straight up (all joints zero; SRDF 'up' state)
          phi = pi/2  -> tool points horizontally forward
          phi = pi    -> tool points straight down
    roll  theta5, rotation of the gripper about the tool axis.

Note this differs from Yahboom's convention, which measures the angle between the
tool and the TABLE and solves to the arm5 rotation centre rather than the TCP.
Their `pitch = 1.04` is therefore NOT directly usable as a phi -- convert it via
fk() before treating any vendor constant as a phi.

GEOMETRY (dofbot_description/urdf/dofbot.urdf, all metres; the first two
segments are measured on the physical arm and differ from Yahboom's CAD -- see
the standoff note below)
---------------------------------------------------------------
    base_link  --(z 0.0745)-->  arm1  (yaw theta1, about z)
               --(z 0.033)-->   arm2  (pitch theta2, about y)   shoulder at Z0
               --(z 0.08285)--> arm3  (pitch theta3, about y)   L1
               --(z 0.08285)--> arm4  (pitch theta4, about y)   L2
               --(x -0.00215, z 0.078149)--> arm5 (roll theta5, about z)
               --(x -0.00265, z 0.068091)--> Gripping_point_Link   (the TCP)

The small y offsets in the URDF (5e-05, 0.00055, ...) lie along the pitch
rotation axis, so they accumulate as a constant LATERAL offset of the tool from
the plane swept by theta1. They total 0.605 mm. Small, but modelled exactly here
rather than dropped, because doing so costs three lines and makes fk/ik round-trip
to machine precision instead of ~1 mm.

The TCP's own x/y offset sits AFTER the theta5 roll, so it swings in and out of
the pitch plane as the wrist rolls. That is also modelled exactly.

The fixed rpy="3.1416 -1.5708 0" on Gripping_Joint reorients the TCP frame but
does not move it, so it does not enter the position kinematics at all.
"""

from math import atan2, cos, degrees, hypot, sin, sqrt

# --- link geometry, transcribed from the URDF -------------------------------

# MEASURED ON THE PHYSICAL ARM, and _D_BASE deliberately DIFFERS from Yahboom's
# CAD. Measured in Fusion against the mesh: the shipped standoffs are EXACTLY
# 50.0 mm and this arm's are EXACTLY 32.0 mm, so the difference is exactly
# 18.0 mm. That is the ONLY difference -- the shoulder bracket is the same part,
# so _D_SHOULDER keeps the CAD's 0.033 and the whole correction lands on
# _D_BASE. The shoulder ends up 107.5 mm above the plate underside, not 125.5.
#
# Anchor on the STANDOFF difference, not on a measured shoulder height. Both
# standoff figures are direct measurements of the same feature, so their
# difference is exact; deriving 17.5 mm from a measured 108 mm shoulder instead
# carried the rounding in that reading, and put Z0 0.5 mm too high.
#
# Shorter standoffs are the better hardware for this task, which is worth
# knowing before anyone "fixes" it by ordering longer ones. Raising the shoulder
# buys height but costs FLOOR reach, because the arm must reach further down:
#   50 mm standoffs (Z0=0.1255): 452.5 mm max height, 278.6 mm floor radius
#   32 mm standoffs (Z0=0.1075): 434.5 mm max height, 287.0 mm floor radius
# Picking off the floor wants the radius.
#
# base_link.STL was shortened by the same 18 mm in Fusion, so the visual mesh
# and the collision primitives now agree. If RViz ever shows the base standing
# proud of the collision cylinder again, that mesh has been reverted to stock.
_D_BASE = 0.0745        # base_link -> arm1_Joint  (CAD says 0.0925)
_D_SHOULDER = 0.033     # arm1_Joint -> arm2_Joint (same as CAD)
Z0 = _D_BASE + _D_SHOULDER          # 0.1075, height of the shoulder pitch axis
L1 = 0.08285            # arm2 -> arm3
L2 = 0.08285            # arm3 -> arm4

# arm4 -> arm5, expressed in the arm4 frame
_A5_X, _A5_Y, _A5_Z = -0.00215, -4.5e-05, 0.078149
# arm5 -> TCP, expressed in the arm5 frame (i.e. after the theta5 roll)
_TCP_X, _TCP_Y, _TCP_Z = -0.00265, 9.7552e-05, 0.068091

# y offsets of the arm2/arm3/arm4 joint origins. The pitch axis is y throughout,
# so these do not rotate relative to one another and simply sum.
_LAT_PRE = 5e-05 + 0.00055 + 5e-05      # 0.00065

# Joint limits, transcribed per joint from the URDF's <limit> tags. They are NOT
# all the same and they are NOT symmetric: arm1 swings -109..+108 degrees, a 217
# degree sector, while the other four are +-89.95.
#
# An earlier version of this module used a single symmetric JOINT_LIMIT = 1.57
# for all five. That silently clipped the base yaw to +-90 and threw away 19
# degrees of working sector on each side -- targets out at the edge came back as
# "no workable approach" when the arm reaches them comfortably. If you add a
# limit check anywhere, read it from here.
JOINT_LIMITS = (
    (-1.9024, 1.8850),      # arm1_Joint  yaw   -109.00 .. +108.00 deg
    (-1.5700, 1.5700),      # arm2_Joint  pitch
    (-1.5700, 1.5700),      # arm3_Joint  pitch
    (-1.5700, 1.5700),      # arm4_Joint  pitch
    (-1.5700, 1.5700),      # arm5_Joint  roll
)

# Kept for callers that want a single conservative number (the tightest joint).
# Prefer JOINT_LIMITS; this cannot express arm1's wider, asymmetric range.
JOINT_LIMIT = 1.57


def in_limits(joints):
    """True if every angle is within its own joint's URDF range."""
    return all(lo <= q <= hi for q, (lo, hi) in zip(joints, JOINT_LIMITS))


# Slack on the 2R reach test. A fully-extended elbow (theta3 = 0) puts the wrist
# centre at exactly L1 + L2, and that distance comes back from fk() as
# 0.16570000000000001 -- rounding, but enough to fail a bare `>` and reject a
# perfectly ordinary posture. The acos argument is clamped below, so admitting a
# nanometre of overshoot costs nothing and the solution stays exact.
_REACH_EPS = 1e-9
JOINT_NAMES = ('arm1_Joint', 'arm2_Joint', 'arm3_Joint', 'arm4_Joint', 'arm5_Joint')

ELBOW_BRANCHES = ('up', 'down')
TIPS = ('tcp', 'arm5')


class Unreachable(ValueError):
    """Raised by ik(strict=True) when no valid solution exists."""


def _last_segment(roll, tip):
    """Geometry of the final segment, resolved in the arm4 frame.

    Returns (dx, dz, lat): the in-plane x and z components of the vector from the
    arm4 origin to the tool point, plus the lateral (out-of-plane) offset.

    For tip='arm5' the roll is irrelevant -- rotation about z cannot move a point
    on the z axis of the same frame, and arm5's origin is where the roll happens.
    """
    if tip not in TIPS:
        raise ValueError('tip must be one of %r, got %r' % (TIPS, tip))

    lat = _LAT_PRE + _A5_Y
    dx, dz = _A5_X, _A5_Z

    if tip == 'tcp':
        # Rz(roll) applied to the TCP offset, which is expressed in the arm5 frame.
        c, s = cos(roll), sin(roll)
        dx += _TCP_X * c - _TCP_Y * s
        lat += _TCP_X * s + _TCP_Y * c
        dz += _TCP_Z

    return dx, dz, lat


def fk(joints, tip='tcp'):
    """Forward kinematics.

    joints: five angles in radians, ordered as JOINT_NAMES.
    Returns (x, y, z, phi, roll) with the tool point in base_link metres, phi the
    tilt from vertical, and roll passed straight through.

    Deliberately computed from the same planar decomposition the IK inverts, but
    the round-trip test cross-checks it against move_group's /compute_fk, which is
    an independent implementation reading the URDF directly.
    """
    t1, t2, t3, t4, t5 = joints
    dx, dz, lat = _last_segment(t5, tip)

    phi = t2 + t3 + t4
    l3 = hypot(dx, dz)
    delta = atan2(dx, dz)

    # Radial/vertical position within the plane swept by theta1.
    r = L1 * sin(t2) + L2 * sin(t2 + t3) + l3 * sin(phi + delta)
    z = Z0 + L1 * cos(t2) + L2 * cos(t2 + t3) + l3 * cos(phi + delta)

    # Rotate (r, lat) out of the plane and into base_link.
    c, s = cos(t1), sin(t1)
    return (r * c - lat * s, r * s + lat * c, z, phi, t5)


def ik(x, y, z, phi, roll=0.0, elbow='up', tip='tcp', strict=False):
    """Inverse kinematics. Exact, closed-form.

    Returns five joint angles (radians, ordered as JOINT_NAMES), or None if the
    target is out of reach or the solution violates a joint limit. Pass
    strict=True to raise Unreachable with a diagnosis instead of returning None.

    elbow selects between the two law-of-cosines branches: 'up' bends the elbow
    away from the base, 'down' toward it. Both are checked by ik_best().

    KNOWN LIMITATION, and it is exactly characterised: this solves only poses with
    a non-negative in-plane radius, i.e. the tool on the same side of the base axis
    as theta1 points. Configurations folded back THROUGH the base axis (negative r)
    are valid on the real arm but are not recovered here, because theta1 is derived
    from atan2(y, x) and a negative radius aliases onto a yaw flipped by pi.

    Measured over 4000 random valid joint vectors: 1988 round-trip exactly and 2012
    fail, and every single failure is a negative-r configuration -- zero failures
    from any other cause. The forward working volume, which is all this arm is used
    for, is entirely r > 0, so this is a non-issue in practice. Documented so the
    ~50% figure is not mistaken for a bug later.
    """
    if elbow not in ELBOW_BRANCHES:
        raise ValueError('elbow must be one of %r, got %r'
                         % (ELBOW_BRANCHES, elbow))

    dx, dz, lat = _last_segment(roll, tip)
    l3 = hypot(dx, dz)
    delta = atan2(dx, dz)

    def fail(msg):
        if strict:
            raise Unreachable(msg)
        return None

    # --- theta1: undo the lateral offset ------------------------------------
    # The tool sits `lat` to one side of the theta1 plane, so the plane must be
    # aimed slightly off the bearing to the target.
    rho = hypot(x, y)
    if rho < abs(lat):
        return fail('target is %.4f m from the base axis, inside the %.4f m '
                    'lateral tool offset -- no theta1 can reach it'
                    % (rho, abs(lat)))
    r = sqrt(rho * rho - lat * lat)
    t1 = atan2(y, x) - atan2(lat, r)
    lo, hi = JOINT_LIMITS[0]
    if not lo <= t1 <= hi:
        return fail('arm1_Joint = %.3f rad (%.1f deg) is outside its %.1f..%.1f '
                    'deg range' % (t1, degrees(t1), degrees(lo), degrees(hi)))

    # --- wrist centre (the arm4 origin) -------------------------------------
    rw = r - l3 * sin(phi + delta)
    zw = z - Z0 - l3 * cos(phi + delta)

    # --- planar 2R for theta2, theta3 ---------------------------------------
    d2 = rw * rw + zw * zw
    d = sqrt(d2)
    if d > L1 + L2 + _REACH_EPS:
        return fail('wrist centre is %.4f m from the shoulder, beyond the '
                    '%.4f m the upper arm can span' % (d, L1 + L2))
    if d < abs(L1 - L2) - _REACH_EPS:
        return fail('wrist centre is %.4f m from the shoulder, inside the '
                    '%.4f m minimum' % (d, abs(L1 - L2)))

    cos_t3 = (d2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    cos_t3 = max(-1.0, min(1.0, cos_t3))       # guard fp drift at full stretch
    t3 = acos_signed(cos_t3, elbow)

    t2 = atan2(rw, zw) - atan2(L2 * sin(t3), L1 + L2 * cos(t3))
    t4 = phi - t2 - t3
    t5 = roll

    joints = [t1, t2, t3, t4, t5]
    for name, q, (lo, hi) in zip(JOINT_NAMES, joints, JOINT_LIMITS):
        if not lo <= q <= hi:
            return fail('%s = %.3f rad (%.1f deg) is outside its %.1f..%.1f deg '
                        'range (elbow=%r)'
                        % (name, q, degrees(q), degrees(lo), degrees(hi), elbow))
    return joints


def acos_signed(c, elbow):
    """Elbow branch selection. 'up' takes the negative root, which bends the
    elbow away from the base -- the posture that keeps the forearm clear of the
    chassis behind the arm."""
    from math import acos
    a = acos(c)
    return -a if elbow == 'up' else a


def ik_best(x, y, z, phi, roll=0.0, seed=None, tip='tcp'):
    """Try both elbow branches, return the valid solution closest to `seed`.

    Seeding matters for Cartesian segments: without it, consecutive waypoints can
    flip between branches and command a violent reconfiguration mid-move.
    """
    candidates = [s for s in (ik(x, y, z, phi, roll, e, tip)
                              for e in ELBOW_BRANCHES) if s is not None]
    if not candidates:
        return None
    if seed is None:
        return candidates[0]
    return min(candidates,
               key=lambda s: sum((a - b) ** 2 for a, b in zip(s, seed)))


def reach_limits(tip='tcp', roll=0.0):
    """Envelope summary, for sanity checks and error messages."""
    dx, dz, lat = _last_segment(roll, tip)
    l3 = hypot(dx, dz)
    return {
        'shoulder_height': Z0,
        'upper_arm': L1 + L2,
        'tool_length': l3,
        'max_reach_from_shoulder': L1 + L2 + l3,
        'max_height': Z0 + L1 + L2 + l3,
        'lateral_offset': lat,
        'yaw_range': JOINT_LIMITS[0],       # arm1 is asymmetric and wider
        'yaw_sector': JOINT_LIMITS[0][1] - JOINT_LIMITS[0][0],
    }


def describe(x, y, z, phi, roll=0.0, tip='tcp'):
    """Human-readable reason a pose is or is not reachable. For CLI diagnostics."""
    for elbow in ELBOW_BRANCHES:
        try:
            ik(x, y, z, phi, roll, elbow, tip, strict=True)
            return 'reachable (elbow=%s)' % elbow
        except Unreachable as exc:
            last = str(exc)
    return 'unreachable: %s' % last
