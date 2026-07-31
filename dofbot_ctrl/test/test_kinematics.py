#!/usr/bin/env python3
# coding: utf-8
"""
Offline tests for dofbot_kinematics: no ROS, no hardware, no move_group.

    pytest src/dofbot_ros2/dofbot_ctrl/test/test_kinematics.py

Three kinds of check, in increasing order of what they can catch:

1. Self-consistency -- FK -> IK -> FK round-trips. Catches algebra errors in the
   inversion, but not a mistranscribed URDF constant, since both directions read
   the same constants.

2. Independence -- the module's planar decomposition is compared against a
   generic 4x4 homogeneous-transform chain built by PARSING dofbot.urdf here in
   the test. That is a different algorithm reading a different source, so it does
   catch a wrong offset, a dropped y term or a flipped axis. It is the offline
   half of Verification step 2; the /compute_fk comparison against move_group is
   the online half and covers the same ground with a third implementation.

3. Behaviour -- limits, unreachable poses, elbow branches, seeding.

The regression joint vectors come from the Yahboom nano MoveIt lessons. Those are
valid for this robot: the nano's dofbot.urdf is byte-for-byte identical to ours.
"""

import os
from math import atan2, cos, degrees, hypot, isclose, pi, radians, sin

import pytest

from dofbot_ctrl import dofbot_kinematics as K
from dofbot_ctrl.dofbot_kinematics import Unreachable


# --------------------------------------------------------------------------
# An independent FK, built from the URDF rather than from the module's constants
# --------------------------------------------------------------------------

_URDF = os.path.join(os.path.dirname(__file__), '..', '..',
                     'dofbot_description', 'urdf', 'dofbot.urdf')

# base_link -> TCP, in order. Which joint each entry is driven by (None = fixed).
_CHAIN = [('arm1_Joint', 0), ('arm2_Joint', 1), ('arm3_Joint', 2),
          ('arm4_Joint', 3), ('arm5_Joint', 4), ('Gripping_Joint', None)]


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def _rpy_xyz(rpy, xyz):
    """Fixed-axis roll-pitch-yaw (URDF convention: R = Rz*Ry*Rx) plus translation."""
    cr, sr = cos(rpy[0]), sin(rpy[0])
    cp, sp = cos(rpy[1]), sin(rpy[1])
    cy, sy = cos(rpy[2]), sin(rpy[2])
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp, cp * sr, cp * cr, xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _axis_rot(axis, q):
    """Rotation of q about an arbitrary unit axis (Rodrigues), as a 4x4."""
    ax, ay, az = axis
    n = hypot(hypot(ax, ay), az)
    ax, ay, az = ax / n, ay / n, az / n
    c, s, t = cos(q), sin(q), 1.0 - cos(q)
    return [
        [t * ax * ax + c, t * ax * ay - s * az, t * ax * az + s * ay, 0.0],
        [t * ax * ay + s * az, t * ay * ay + c, t * ay * az - s * ax, 0.0],
        [t * ax * az - s * ay, t * ay * az + s * ax, t * az * az + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _load_urdf_chain():
    """Parse dofbot.urdf into (origin_matrix, axis, joint_index) per chain link."""
    import xml.etree.ElementTree as ET

    joints = {j.get('name'): j for j in ET.parse(_URDF).getroot().findall('joint')}
    out = []
    for name, idx in _CHAIN:
        j = joints[name]
        origin = j.find('origin')
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
        axis_el = j.find('axis')
        axis = ([float(v) for v in axis_el.get('xyz').split()]
                if axis_el is not None and idx is not None else None)
        out.append((_rpy_xyz(rpy, xyz), axis, idx))
    return out


_CHAIN_CACHE = _load_urdf_chain() if os.path.exists(_URDF) else None

needs_urdf = pytest.mark.skipif(
    _CHAIN_CACHE is None,
    reason='dofbot_description/urdf/dofbot.urdf not found next to this package')


def urdf_fk(joints, tip='tcp'):
    """TCP (or arm5 origin) position in base_link, by generic transform chain."""
    t = [[float(i == j) for j in range(4)] for i in range(4)]
    for origin, axis, idx in _CHAIN_CACHE:
        if tip == 'arm5' and axis is None:
            break                       # stop before the fixed TCP offset
        t = _matmul(t, origin)
        if axis is not None:
            t = _matmul(t, _axis_rot(axis, joints[idx]))
        if tip == 'arm5' and idx == 4:
            break                       # arm5 origin, before its own roll
    return (t[0][3], t[1][3], t[2][3])


# --------------------------------------------------------------------------
# grids and helpers
# --------------------------------------------------------------------------

LIM = K.JOINT_LIMIT


def joint_grid(n=5):
    """All n**5 combinations of n angles spanning the joint range.

    The span stops a whisker short of the stops on purpose. ik() rejects any
    solution with abs(q) > JOINT_LIMIT and applies no epsilon, so a state sitting
    exactly ON a limit round-trips to 1.5700000000000003 and is rejected -- an
    artefact of testing an infinitely thin set, not a defect worth an epsilon in
    the solver. Poses that need a joint exactly at its stop are not poses to plan
    for anyway.
    """
    span = 0.999 * LIM
    vals = [-span + 2.0 * span * i / (n - 1) for i in range(n)]
    for a in vals:
        for b in vals:
            for c in vals:
                for d in vals:
                    for e in vals:
                        yield [a, b, c, d, e]


def random_joints(count, seed=20260730):
    import random
    rng = random.Random(seed)
    return [[rng.uniform(-LIM, LIM) for _ in range(5)] for _ in range(count)]


def in_plane_radius(joints, tip='tcp'):
    """Signed radius in the theta1 half-plane. See the negative-r test below.

    Projecting the tool point back onto the theta1 direction cancels the lateral
    offset exactly (x = r*cos t1 - lat*sin t1, y = r*sin t1 + lat*cos t1), so
    this recovers the sign that ik() cannot see from (x, y) alone.
    """
    x, y = K.fk(joints, tip)[:2]
    c, s = cos(joints[0]), sin(joints[0])
    return x * c + y * s


def max_abs_diff(a, b):
    return max(abs(p - q) for p, q in zip(a, b))


# --------------------------------------------------------------------------
# 1. FK against the URDF -- catches transcription errors
# --------------------------------------------------------------------------

@needs_urdf
@pytest.mark.parametrize('tip', ['tcp', 'arm5'])
def test_fk_matches_urdf_transform_chain(tip):
    worst = 0.0
    for joints in joint_grid(4):
        ours = K.fk(joints, tip)[:3]
        theirs = urdf_fk(joints, tip)
        worst = max(worst, max_abs_diff(ours, theirs))
    assert worst < 1e-12, 'FK disagrees with the URDF chain by %.3g m' % worst


@needs_urdf
def test_fk_matches_urdf_on_random_states():
    worst = max(max_abs_diff(K.fk(j)[:3], urdf_fk(j)) for j in random_joints(500))
    assert worst < 1e-12


# --------------------------------------------------------------------------
# 2. FK -> IK -> FK round-trip
# --------------------------------------------------------------------------

def test_roundtrip_over_joint_grid():
    """Every forward-workspace configuration must invert exactly.

    Configurations with a negative in-plane radius are excluded: they are real
    poses on the real arm, but ik() derives theta1 from atan2(y, x) and so
    aliases them onto a yaw flipped by pi. Documented in the ik() docstring;
    test_negative_radius_is_the_only_failure_mode pins it down as the *only*
    excluded class.
    """
    tested = 0
    worst_j = worst_p = 0.0
    for joints in joint_grid(5):
        if in_plane_radius(joints) <= 1e-3:
            continue
        x, y, z, phi, roll = K.fk(joints)
        got = K.ik_best(x, y, z, phi, roll, seed=joints)
        assert got is not None, 'no IK for %r (from FK of a valid state)' % (joints,)
        tested += 1
        worst_j = max(worst_j, max_abs_diff(got, joints))
        worst_p = max(worst_p, max_abs_diff(K.fk(got)[:3], (x, y, z)))
    assert tested > 1000, 'grid degenerated to %d cases' % tested
    # Position is the tolerance that matters and it holds to machine precision.
    # Joint angles are allowed a thousand times more slack because the grid
    # includes fully-extended elbows, where theta2/theta3 are ill-conditioned:
    # near that singularity a nanometre of position error is worth ~1e-7 rad of
    # joint error. It shows up nowhere else.
    assert worst_p < 1e-9, 'position round-trip off by %.3g m' % worst_p
    assert worst_j < 1e-6, 'joint round-trip off by %.3g rad' % worst_j


def test_roundtrip_random_states():
    tested = 0
    worst = 0.0
    for joints in random_joints(2000):
        if in_plane_radius(joints) <= 1e-3:
            continue
        x, y, z, phi, roll = K.fk(joints)
        got = K.ik_best(x, y, z, phi, roll, seed=joints)
        assert got is not None
        tested += 1
        worst = max(worst, max_abs_diff(K.fk(got)[:3], (x, y, z)))
    assert tested > 500
    assert worst < 1e-9


@pytest.mark.parametrize('tip', ['tcp', 'arm5'])
def test_roundtrip_honours_tip(tip):
    """'tcp' and 'arm5' must each invert their own point, not each other's."""
    for joints in random_joints(300, seed=7):
        if in_plane_radius(joints, tip) <= 1e-3:
            continue
        x, y, z, phi, roll = K.fk(joints, tip)
        got = K.ik_best(x, y, z, phi, roll, seed=joints, tip=tip)
        assert got is not None
        assert max_abs_diff(K.fk(got, tip)[:3], (x, y, z)) < 1e-9


def test_tips_differ_by_the_tool_length():
    """A sanity check that the tip switch is doing something real.

    The separation is the full length of the Gripping_Joint offset vector, i.e.
    slightly more than its 0.068091 z component, because of the -0.00265 x term.
    """
    expected = hypot(hypot(0.00265, 9.7552e-05), 0.068091)
    for joints in ([0.3, 0.4, -0.5, 0.2, 0.0], [-0.2, 0.1, 0.9, -0.4, 1.1]):
        tcp = K.fk(joints, 'tcp')[:3]
        arm5 = K.fk(joints, 'arm5')[:3]
        sep = hypot(hypot(tcp[0] - arm5[0], tcp[1] - arm5[1]), tcp[2] - arm5[2])
        assert isclose(sep, expected, abs_tol=1e-9)


def test_negative_radius_is_the_only_failure_mode():
    """Pin the documented limitation: no OTHER class of valid state fails IK."""
    failures = 0
    for joints in random_joints(4000, seed=99):
        x, y, z, phi, roll = K.fk(joints)
        got = K.ik_best(x, y, z, phi, roll, seed=joints)
        recovered = got is not None and max_abs_diff(got, joints) < 1e-9
        if not recovered:
            failures += 1
            assert in_plane_radius(joints) < 1e-3, (
                'IK failed on a positive-radius state %r -- that is a real bug, '
                'not the documented aliasing' % (joints,))
    assert failures > 0, 'sample never exercised the negative-radius branch'


# --------------------------------------------------------------------------
# 3. Behaviour: limits, unreachable, branches, seeding
# --------------------------------------------------------------------------

def test_rejects_pose_beyond_reach():
    lim = K.reach_limits()
    far = lim['max_reach_from_shoulder'] + 0.05
    assert K.ik(far, 0.0, K.Z0, pi / 2) is None
    assert K.ik_best(far, 0.0, K.Z0, pi / 2) is None
    with pytest.raises(Unreachable, match='beyond'):
        K.ik(far, 0.0, K.Z0, pi / 2, strict=True)


def test_rejects_pose_inside_the_shoulder():
    """Tool point folded back onto the shoulder: wrist centre unreachably close.

    L1 == L2 here, so the 2R minimum is 0 and the inner limit is never the
    binding one; what actually rejects this is the joint limits.
    """
    assert K.ik(0.02, 0.0, K.Z0, 0.0) is None
    with pytest.raises(Unreachable):
        K.ik(0.02, 0.0, K.Z0, 0.0, strict=True)


def test_rejects_target_on_the_base_axis():
    """Inside the lateral tool offset there is no theta1 that can aim the arm."""
    lat = abs(K.reach_limits()['lateral_offset'])
    assert K.ik(0.0, 0.0, 0.30, 0.0) is None
    with pytest.raises(Unreachable, match='lateral'):
        K.ik(lat / 2, 0.0, 0.30, 0.0, strict=True)


def test_rejects_yaw_beyond_limit():
    """Straight back (-x) is geometrically fine but needs theta1 = pi."""
    reach = 0.20
    assert K.ik(-reach, 0.0, 0.20, pi / 2) is None
    with pytest.raises(Unreachable, match='arm1_Joint|theta1'):
        K.ik(-reach, 0.0, 0.20, pi / 2, strict=True)


def test_rejects_solution_violating_a_wrist_limit():
    """A reachable point whose required phi drives theta4 past its stop.

    Found by construction: take a valid state, then demand the same position
    with a phi far from the one it was generated at.
    """
    joints = [0.0, 0.6, -0.3, 0.4, 0.0]
    x, y, z, phi, _ = K.fk(joints)
    assert K.ik(x, y, z, phi) is not None
    bad = [K.ik(x, y, z, p) for p in (phi - 1.4, phi + 1.4)]
    assert any(s is None for s in bad), 'expected a joint-limit rejection'


def test_invalid_arguments_raise():
    with pytest.raises(ValueError, match='elbow'):
        K.ik(0.2, 0.0, 0.2, 1.0, elbow='sideways')
    with pytest.raises(ValueError, match='tip'):
        K.fk([0, 0, 0, 0, 0], tip='fingertip')


# A pose both elbow branches can reach. Most of this arm's workspace admits only
# one, because the +-1.57 stops on every joint cut the 'down' branch away over
# most of the forward volume; found by sweep.
TWO_BRANCH_POSE = (0.25, 0.05, 0.11, 2.2)


def test_elbow_branches_are_distinct_and_both_exact():
    """Where both branches exist they must be different postures, same point."""
    x, y, z, phi = TWO_BRANCH_POSE
    up = K.ik(x, y, z, phi, elbow='up')
    down = K.ik(x, y, z, phi, elbow='down')
    assert up is not None and down is not None
    assert up[2] < 0.0 < down[2], 'branch sign convention changed'
    assert max_abs_diff(up, down) > 0.1, 'branches collapsed onto each other'
    for s in (up, down):
        assert max_abs_diff(K.fk(s)[:3], (x, y, z)) < 1e-12
        assert isclose(K.fk(s)[3], phi, abs_tol=1e-12)


def test_ik_best_returns_the_branch_nearest_the_seed():
    x, y, z, phi = TWO_BRANCH_POSE
    up = K.ik(x, y, z, phi, elbow='up')
    down = K.ik(x, y, z, phi, elbow='down')
    assert K.ik_best(x, y, z, phi, seed=up) == up
    assert K.ik_best(x, y, z, phi, seed=down) == down


def test_ik_best_keeps_a_cartesian_segment_on_one_branch():
    """The reason seeding exists: no branch flip mid-segment.

    A 170 mm vertical lift at x = 0.26, phi = 1.9 -- about the longest
    straight-line move this arm has at a fixed pitch, and the shape of the lift
    in the pick sequence. Re-seeding from the previous waypoint must never
    produce a discontinuous jump, which is exactly what cartesian_move relies on.
    """
    n = 40
    seed = None
    prev = None
    for i in range(n + 1):
        z = 0.02 + 0.17 * i / n
        s = K.ik_best(0.26, 0.0, z, 1.9, seed=seed)
        assert s is not None, 'segment left the workspace at z=%.3f' % z
        if prev is not None:
            assert max_abs_diff(s, prev) < 0.15, (
                'branch flip between waypoints %d and %d' % (i - 1, i))
        seed = prev = s


def test_ik_best_without_a_seed_still_solves():
    assert K.ik_best(*TWO_BRANCH_POSE) is not None


def test_roll_passes_through_and_shifts_the_tcp_slightly():
    """The TCP x-offset sits after the theta5 roll, so roll moves the tool by up
    to ~2.65 mm. Documented as ignored in practice; pinned here so a future
    change cannot silently drop the modelling."""
    base = K.fk([0.0, 0.5, -0.2, 0.3, 0.0])
    rolled = K.fk([0.0, 0.5, -0.2, 0.3, pi / 2])
    assert rolled[4] == pytest.approx(pi / 2)
    shift = hypot(rolled[0] - base[0], rolled[1] - base[1])
    assert 0.002 < shift < 0.004


def test_reach_limits_are_self_consistent():
    lim = K.reach_limits()
    assert lim['shoulder_height'] == pytest.approx(0.1255)
    assert lim['upper_arm'] == pytest.approx(K.L1 + K.L2)
    assert lim['tool_length'] == pytest.approx(0.146319, abs=1e-6)
    assert lim['max_height'] == pytest.approx(
        lim['shoulder_height'] + lim['max_reach_from_shoulder'])
    # max_height is a true bound: nothing exceeds it, and the all-zeros pose sits
    # just under it, because reaching it exactly needs the tool tilted by the
    # -0.0328 rad delta that the -0.00215/-0.00265 x-offsets introduce.
    top = K.fk([0, 0, 0, 0, 0])[2]
    assert 0.0 < lim['max_height'] - top < 1e-4
    assert max(K.fk(j)[2] for j in random_joints(500, seed=3)) < lim['max_height']
    # Both tool lengths are the FULL offset vector, so each is a shade longer
    # than its z component alone (0.146319 > 0.146240, 0.078179 > 0.078149).
    assert K.reach_limits('arm5')['tool_length'] == pytest.approx(
        hypot(0.00215, 0.078149), abs=1e-9)


def test_describe_explains_both_outcomes():
    assert K.describe(*TWO_BRANCH_POSE).startswith('reachable')
    unreachable = K.describe(0.9, 0.0, 0.2, 1.2)
    assert unreachable.startswith('unreachable')
    assert 'beyond' in unreachable, 'lost the diagnosis, kept only the verdict'


# --------------------------------------------------------------------------
# 4. Regression vectors from the nano MoveIt lessons (same URDF as ours)
# --------------------------------------------------------------------------

# fk() output for each, frozen the first time the module passed the URDF
# cross-check above. If one of these moves, a constant changed.
LESSON_STATES = {
    # Yahboom "Forward Kinematics Design" / "Scene Design"
    'fk_design': ([0.0, 0.52, 0.786, 0.20, 0.0],
                  (0.266751155, 0.000702552, 0.233340829, 1.506, 0.0)),
    # "Trajectory Planning", the three sequential targets
    'traj_1': ([1.57, -1.00, -0.61, 0.20, 0.0],
               (-0.000939558, -0.297623578, 0.185692428, -1.41, 0.0)),
    'traj_2': ([0.0, 0.0, 0.0, 0.0, 0.0],
               (-0.004800000, 0.000702552, 0.437440000, 0.0, 0.0)),
    'traj_3': ([-1.16, -0.50, -0.81, -0.79, 1.57],
               (-0.099660595, 0.223679890, 0.143799967, -2.10, 1.57)),
    # our own SRDF 'init' -- kept as a vector even though the named state is
    # being replaced, because it is a useful non-trivial posture
    'srdf_init': ([0.0, -1.57, 1.57, 1.57, 0.0],
                  (0.063386158, 0.000702552, 0.213332429, 1.57, 0.0)),
}


@pytest.mark.parametrize('name', sorted(LESSON_STATES))
def test_lesson_state_fk_is_stable(name):
    joints, expected = LESSON_STATES[name]
    assert K.fk(joints) == pytest.approx(expected, abs=1e-9)


@needs_urdf
@pytest.mark.parametrize('name', sorted(LESSON_STATES))
def test_lesson_state_fk_matches_urdf(name):
    joints, _ = LESSON_STATES[name]
    assert max_abs_diff(K.fk(joints)[:3], urdf_fk(joints)) < 1e-12


@pytest.mark.parametrize('name', sorted(LESSON_STATES))
def test_lesson_state_inverts(name):
    """Every lesson state that lies in the forward workspace must invert."""
    joints, _ = LESSON_STATES[name]
    if in_plane_radius(joints) <= 1e-3:
        pytest.skip('negative in-plane radius; see the ik() docstring')
    x, y, z, phi, roll = K.fk(joints)
    got = K.ik_best(x, y, z, phi, roll, seed=joints)
    assert got is not None
    assert max_abs_diff(got, joints) < 1e-9


# The pose target from the lessons' Inverse Kinematics page. Its quaternion is
# unusable (unnormalised, and over-constrained for a 5-DOF arm), so only the
# position is tested -- swept over phi, which is the free parameter our
# formulation exposes and theirs does not.
LESSON_IK_TARGET = (0.111138, 0.028503, 0.311743)


def test_lesson_ik_target_is_reachable_over_a_band_of_pitches():
    x, y, z = LESSON_IK_TARGET
    solved = [radians(d) for d in range(0, 181)
              if K.ik_best(x, y, z, radians(d)) is not None]
    assert solved, 'the lessons\' IK target came out unreachable'
    # A contiguous band around ~70 deg from vertical; nothing near top-down.
    assert 1.0 < min(solved) < 1.2
    assert 1.4 < max(solved) < 1.5
    assert len(solved) == round(degrees(max(solved) - min(solved))) + 1, \
        'expected one contiguous band of feasible pitches'


def test_lesson_ik_target_solution_is_exact():
    x, y, z = LESSON_IK_TARGET
    phi = 1.3963            # 80 deg from vertical -- the Pro's grasp pitch
    s = K.ik_best(x, y, z, phi)
    assert s is not None
    assert max_abs_diff(K.fk(s)[:3], (x, y, z)) < 1e-12
    assert K.fk(s)[3] == pytest.approx(phi, abs=1e-12)
    assert all(abs(q) <= K.JOINT_LIMIT for q in s)
    # theta1 is fixed by the target bearing alone, independent of phi -- less the
    # ~6 mrad the 0.6 mm lateral tool offset costs at this radius.
    lat = K.reach_limits()['lateral_offset']
    r = (hypot(x, y) ** 2 - lat ** 2) ** 0.5
    assert s[0] == pytest.approx(atan2(y, x) - atan2(lat, r), abs=1e-12)
    assert K.ik_best(x, y, z, 1.2)[0] == pytest.approx(s[0], abs=1e-12)
