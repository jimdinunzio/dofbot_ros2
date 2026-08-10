#!/usr/bin/env python3
# coding: utf-8
"""
Offline tests for the object/gripper model: gripper.py, graspable.py and the
pure geometry in scene_objects.py.

    pytest src/dofbot_ros2/dofbot_ctrl/test/test_grasp_model.py

The held_pose expectations are not invented. They were cross-checked against
move_group: attaching a 120 mm cylinder gripped 30 mm up from its base, at
phi = 2.6, and reading the attached object back out of /get_planning_scene gave
Gripping_point_Link-relative (-0.025707, -1.1e-07, -0.015465) with quaternion
(0.87049, ~0, -0.49219, ~0). Both agree with the arm5_Link pose asserted below
transformed through Gripping_Joint's rpy="3.1416 -1.5708 0", to five decimals.
So this file freezes a MoveIt-confirmed result, not a self-consistent one.

scene_objects imports moveit_client, which imports rclpy and the MoveIt
messages. That is an import, not a connection -- no node, no ROS graph -- so
these still run without a robot. The tests skip cleanly if the messages are not
on the path at all (e.g. pytest outside a sourced workspace).
"""

import contextlib
import importlib
import os
from math import cos, hypot, isclose, sin

import pytest

from dofbot_ctrl import gripper
from dofbot_ctrl import dofbot_kinematics as kin
from dofbot_ctrl.graspable import CATALOGUE, GraspableObject, ObjectError, get

scene_objects = pytest.importorskip(
    'dofbot_ctrl.scene_objects',
    reason='needs moveit_msgs on the path; source the workspace')


# ------------------------------------------------------------------- gripper
#
# The arm has two grippers and they swap, so most of these run twice. The
# `profile` fixture reloads gripper.py with DOFBOT_GRIPPER set; because
# graspable holds a reference to the module OBJECT and reload() mutates it in
# place, the catalogue sees the swap too, which is exactly the behaviour under
# test. The fixture restores the default afterwards so ordering cannot leak.

@contextlib.contextmanager
def as_profile(name):
    """Run a block with `name` bolted on, then put the default back.

    reload() mutates the single module OBJECT rather than making a second one,
    so the two profiles cannot be held side by side -- a test that wants both
    has to enter this twice and keep the numbers, not the module. That is also
    why GripperError must be caught as ValueError below: each reload defines a
    fresh class object, and the one captured before the reload is not it.
    """
    old = os.environ.get(gripper.ENV_VAR)
    os.environ[gripper.ENV_VAR] = name
    try:
        yield importlib.reload(gripper)
    finally:
        if old is None:
            os.environ.pop(gripper.ENV_VAR, None)
        else:
            os.environ[gripper.ENV_VAR] = old
        importlib.reload(gripper)


@pytest.fixture(params=['stock', 'extended'])
def profile(request):
    with as_profile(request.param) as g:
        yield g


def test_extended_is_the_default_profile():
    """What is bolted on the arm today, so an unset variable must mean it."""
    assert gripper.PROFILE == 'extended'
    assert gripper.DEFAULT_PROFILE == 'extended'


def test_an_unknown_profile_refuses_to_load():
    """Silently guessing would plan to the wrong fingertips.

    The URDF cannot raise and resolves anything non-'stock' to the extended
    fingers, so the Python side has to be the one that stops.
    """
    # ValueError, not GripperError: the reload that raises also rebinds the
    # class, so the exception is an instance of one this scope has never seen.
    with pytest.raises(ValueError, match='not a gripper'):
        with as_profile('exteded'):                      # a plausible typo
            pass
    assert gripper.PROFILE == 'extended'                 # and it recovered


def test_endpoints_are_the_measured_ones(profile):
    """Stock 0-60 mm; extended 50-105 mm and never shuts."""
    expected = {'stock': (0.0, 0.060), 'extended': (0.050, 0.105)}
    shut, wide = expected[profile.PROFILE]
    assert profile.jaw_width_for(profile.OPEN_ANGLE) == pytest.approx(wide)
    assert profile.jaw_width_for(profile.CLOSE_ANGLE) == pytest.approx(shut)
    assert profile.MAX_WIDTH == pytest.approx(wide)
    assert profile.MIN_WIDTH == pytest.approx(shut)
    assert profile.CLOSE_ANGLE == pytest.approx(1.5708)
    # OPEN_ANGLE is NOT the joint's lower limit on the extended fingers. That
    # linkage is over-centre and its widest point sits at 0.034 rad, measured on
    # hardware; commanding the limit of 0 opens slightly less. The stock table
    # has never been swept finely enough to know whether it does the same.
    assert profile.OPEN_ANGLE == pytest.approx(
        {'stock': 0.0, 'extended': 0.034}[profile.PROFILE])


def _sweep(g, step=1):
    """Every whole mm this profile will actually accept."""
    lo = int(round(g.SAFE_MIN_WIDTH * 1e3))
    hi = int(round(g.SAFE_MAX_WIDTH * 1e3))
    return [mm / 1000.0 for mm in range(lo, hi + 1, step)]


def test_width_and_angle_invert_each_other(profile):
    for w in _sweep(profile):
        assert profile.jaw_width_for(profile.jaw_angle_for(w)) == pytest.approx(
            w, abs=1e-9)


def test_wider_object_means_a_smaller_angle(profile):
    """Angle increases as the jaws close, so it must fall as width rises."""
    angles = [profile.jaw_angle_for(w) for w in _sweep(profile, step=5)]
    assert all(a > b for a, b in zip(angles, angles[1:]))


def test_rejects_what_it_cannot_open_to(profile):
    with pytest.raises(profile.GripperError):
        profile.jaw_angle_for(profile.SAFE_MAX_WIDTH + 1e-4)
    with pytest.raises(profile.GripperError):
        profile.jaw_angle_for(profile.SAFE_MIN_WIDTH - 1e-4)
    with pytest.raises(profile.GripperError, match='negative'):
        profile.jaw_angle_for(-0.001)
    assert not profile.fits(profile.SAFE_MAX_WIDTH + 1e-4)
    assert profile.fits(profile.SAFE_MIN_WIDTH)
    assert profile.fits(profile.SAFE_MAX_WIDTH)


def test_the_can_and_the_block_swap_places_with_the_fingers():
    """The headline trade. Neither profile takes both; each takes one.

    66 mm will not go into the stock 60 mm jaws, and a 30 mm block falls
    straight through fingers that stop 50 mm apart.
    """
    with as_profile('stock') as g:
        assert not g.fits(0.066) and g.fits(0.030)
        with pytest.raises(ValueError, match='66.0 mm'):
            g.jaw_angle_for(0.066)
    with as_profile('extended') as g:
        assert g.fits(0.066) and not g.fits(0.030)
        with pytest.raises(ValueError, match='pass straight between'):
            g.jaw_angle_for(0.030)


def test_a_rejection_names_the_other_gripper(profile):
    """An object the other fingers would take is a screwdriver away, not a dead
    end, and the error has to say so or the answer looks like 'impossible'."""
    unreachable = {'stock': 0.066, 'extended': 0.030}[profile.PROFILE]
    with pytest.raises(profile.GripperError) as exc:
        profile.jaw_angle_for(unreachable)
    assert profile.OTHER_PROFILE in str(exc.value)
    assert 'DOFBOT_GRIPPER=%s' % profile.OTHER_PROFILE in str(exc.value)


def test_grip_angle_squeezes_past_contact(profile):
    """The commanded angle must be tighter than exact contact, or nothing grips."""
    w = profile.SAFE_MIN_WIDTH + 0.010
    assert profile.grip_angle_for(w) > profile.jaw_angle_for(w)
    # Never past the mechanical stop, however hard it is asked to squeeze.
    assert profile.grip_angle_for(w, squeeze=1.0) <= profile.CLOSE_ANGLE
    # The squeeze overshoot must not be mistaken for an object that is too
    # small: an object just inside the limit is still a legal grasp.
    assert profile.grip_angle_for(profile.SAFE_MIN_WIDTH) <= profile.CLOSE_ANGLE
    with pytest.raises(profile.GripperError):
        profile.grip_angle_for(profile.SAFE_MIN_WIDTH - 1e-4)


def test_tip_offset_is_positive_and_grows_as_the_jaws_close(profile):
    """The fingers reach PAST Gripping_point_Link, further the tighter they shut.

    Sign and direction both matter and neither is observable from a passing pick
    -- get either wrong and the arm reaches through the object instead of
    pinching it at the tips, which only shows up when you look at RViz.
    """
    assert profile.tip_offset_for(profile.MAX_WIDTH) > 0.0
    offsets = [profile.tip_offset_for(w) for w in _sweep(profile, step=5)]
    assert all(a > b for a, b in zip(offsets, offsets[1:])), \
        'tip offset must fall as the object gets wider (jaws more open)'


def test_tip_offsets_are_the_measured_ones():
    """These are the hover distances, and drift here is the arm quietly driving
    deeper into every object it picks up.

    Stock comes from the arm5 -> Rlink1 -> Rlink2 chain onto the stock finger
    mesh and is probably ~5 mm long; see the note on _STOCK.

    BOTH ends of extended are touch-offs against the base plate's top face
    (z = 3.0 mm): jaws shut, tf z = 92.0 -> 89.0 mm; jaws open, tf z = 63.0 with
    a 2.6 deg tilt -> 60.1 mm. These two owe nothing to any model, so they are
    the assertions that must not drift. Everything between them is the mesh's
    curve affine-fitted onto these ends.
    """
    with as_profile('stock') as g:
        assert g.tip_offset_for(0.030) == pytest.approx(0.0330, abs=1e-4)
    with as_profile('extended') as g:
        assert g.tip_offset_for(g.MIN_WIDTH) == pytest.approx(0.0890, abs=1e-4)
        assert g.tip_offset_for(g.MAX_WIDTH) == pytest.approx(0.0601, abs=1e-4)
        # the measured swing, which no rigid extension may change
        assert g.tip_offset_for(g.MIN_WIDTH) - g.tip_offset_for(g.MAX_WIDTH) \
            == pytest.approx(0.0289, abs=2e-4)
        assert g.tip_offset_for(0.066) == pytest.approx(0.0882, abs=1e-4)


def test_the_extension_lengthens_the_finger_by_a_constant():
    """It bolts onto the end of the stock finger pointing the same way, so it
    lengthens the reach without changing the shape of the curve. The constant
    itself is not asserted to a tight value -- only the shut end is measured, and
    the rise depends on a stock baseline that is itself mesh-derived. What is
    worth pinning is that the two profiles stay parallel: if they ever stop
    differing by a constant, someone has mixed up the meshes.

    Do not tighten this into an exact figure derived from the meshes. The shut
    end is anchored to a touch-off precisely because a mesh-derived constant here
    can be self-consistent and still badly wrong.
    """
    stock_rows, _ = gripper._STOCK
    ext_rows, _ = gripper._EXTENDED
    for rows in (stock_rows, ext_rows):
        assert rows[-1][0] == pytest.approx(1.5708)
    assert stock_rows[0][0] == 0.0
    assert ext_rows[0][0] == pytest.approx(0.034)   # the over-centre peak
    open_rise = ext_rows[0][2] - stock_rows[0][2]
    shut_rise = ext_rows[-1][2] - stock_rows[-1][2]
    assert open_rise == pytest.approx(shut_rise, abs=2e-3)
    assert 0.040 < shut_rise < 0.055


def test_only_the_measured_profile_claims_to_be_calibrated():
    """Extended widths came off calipers; the stock ones are still a two-point
    stub with the tip offsets derived from the URDF."""
    with as_profile('extended') as g:
        assert g.CALIBRATED is True
        assert 'UNCALIBRATED' not in g.describe(0.066)
    with as_profile('stock') as g:
        assert g.CALIBRATED is False
        assert 'UNCALIBRATED' in g.describe(0.030)


def test_the_measured_span_curve_is_not_a_straight_line():
    """Why the full sweep was worth taking. A two-point fit between 105 mm and
    50 mm is ~8 mm out mid-travel, which is a whole object's worth of error.

    The curve is also the flatter half of an over-centre linkage whose span
    PEAKS at 0 rad and falls away on both sides; see joint_map. Everything here
    assumes 0 is that peak, so span falls monotonically across this table.
    """
    mid = 1.5708 / 2.0
    linear = 0.105 + (0.050 - 0.105) * (mid / 1.5708)
    with as_profile('extended') as g:
        assert abs(g.jaw_width_for(mid) - linear) > 0.008
        widths = [g.jaw_width_for(a / 100.0) for a in range(0, 158, 5)]
        assert all(a >= b for a, b in zip(widths, widths[1:])), \
            'span must fall monotonically from the over-centre peak at 0 rad'


def test_the_can_is_held_where_the_robot_actually_held_it():
    """Ground truth: the can was gripped on hardware and the servo settled here.

    DEFAULT_SQUEEZE was calibrated in the pre-re-key coordinates and was not
    adjusted afterwards, so this passing is a real check rather than a tautology:
    a squeeze fitted in the old frame still lands on the same PHYSICAL grip in
    the new one, which is what a coordinate change should leave alone.
    """
    with as_profile('extended') as g:
        assert g.grip_angle_for(0.066) == pytest.approx(1.411, abs=0.003)
        # The squeeze is real travel past contact, not a rounding artefact.
        assert g.grip_angle_for(0.066) > g.jaw_angle_for(0.066) + 0.04
def _advance(g, width):
    """How much deeper than the fingertip the throat lookup drives the TCP."""
    return g.tip_offset_for(width) - g.throat_offset_for(width)


def _clamp_width(g):
    """The width at which the advance reaches zero: half the object exactly
    fills the finger, leaving the clearance. Wider than this and there is
    nothing to advance."""
    return 2.0 * (g.FINGER_DEPTH - g.BACK_STOP_CLEARANCE)


def _unclamped_widths(g):
    """Two grippable widths strictly below the clamp threshold, derived so the
    formula tests keep probing the live branch whatever the constants become.

    Skips rather than lies if the fitted profile has no such width -- with the
    clearance set high enough, every object it can hold is already clamped, and
    a test that quietly asserted nothing would be worse than one that says so.
    """
    top = min(_clamp_width(g), g.SAFE_MAX_WIDTH)
    lo, hi = g.SAFE_MIN_WIDTH, top - 1e-4
    if hi <= lo:
        pytest.skip('no unclamped width on the %s profile at a %.0f mm '
                    'clearance' % (g.PROFILE, g.BACK_STOP_CLEARANCE * 1e3))
    return (lo, (lo + hi) / 2.0)


def test_the_object_is_seated_against_the_back_stop():
    """The advance is FINGER_DEPTH - width/2 - BACK_STOP_CLEARANCE: the face,
    less the half of the object that hangs behind the contact line, less the
    clearance. Not FINGER_DEPTH (that ignores the object's own radius, and put
    the arm 35 mm too deep on hardware) and not a fixed observed gap.

    Every expected value is computed from FINGER_DEPTH and BACK_STOP_CLEARANCE,
    and the widths it probes are derived from the clamp threshold rather than
    typed in, so retuning either constant leaves this test still testing the
    formula instead of failing on a stale literal.
    """
    with as_profile('extended') as g:
        for w in _unclamped_widths(g):
            room = g.FINGER_DEPTH - w / 2.0 - g.BACK_STOP_CLEARANCE
            assert _advance(g, w) == pytest.approx(room)
            # The object reaches its own radius PLUS the advance into the
            # finger, and that is what has to fit -- clearance left over.
            assert w / 2.0 + _advance(g, w) == pytest.approx(
                g.FINGER_DEPTH - g.BACK_STOP_CLEARANCE)


def test_the_can_clamps_to_the_fingertip_at_the_current_clearance():
    """The can is wider than the clearance leaves room for, so it gets no
    advance at all and is held exactly where it was before throat_offset_for
    existed.

    Recorded rather than tidied away, because it is surprising: the depth
    machinery is live and correct and yet does nothing for the object it was
    written for. That is a property of the hand-set BACK_STOP_CLEARANCE, not of
    the model -- lower it and the can moves in. Stated as the inequality that
    causes it, so this test explains itself at whatever the constants become.
    """
    can = get('soda_can')
    with as_profile('extended') as g:
        assert g.FINGER_DEPTH - can.grasp_width / 2.0 <= g.BACK_STOP_CLEARANCE
        assert _advance(g, can.grasp_width) == 0.0
        assert g.throat_offset_for(can.grasp_width) == g.tip_offset_for(
            can.grasp_width)


def test_the_advance_tracks_the_object_width():
    """The property both earlier attempts lacked. A narrow object leaves more
    room behind its contact line, so it can be driven further in; a fixed nudge
    under-uses that and drives a wide object into the stop."""
    with as_profile('extended') as g:
        assert _advance(g, g.SAFE_MIN_WIDTH) > _advance(g, g.SAFE_MAX_WIDTH)


def test_a_wide_object_is_never_pulled_back_out():
    """At or past the clamp threshold the object is already against the stop
    with its contact at the fingertip. The answer is the fingertip -- an
    unclamped subtraction would go NEGATIVE and hover further out than
    tip_offset_for."""
    with as_profile('extended') as g:
        flush = _clamp_width(g)
        for w in (flush, (flush + g.SAFE_MAX_WIDTH) / 2.0, g.SAFE_MAX_WIDTH):
            assert g.throat_offset_for(w) == g.tip_offset_for(w)
        assert _advance(g, flush) == pytest.approx(0.0)


def test_an_unmeasured_finger_falls_back_to_the_tip():
    """Nobody has measured the stock finger, and a guess there drives the arm
    into the object. Absent the measurement the answer is the old, safe one."""
    with as_profile('stock') as g:
        assert g.FINGER_DEPTH is None
        mid = (g.SAFE_MIN_WIDTH + g.SAFE_MAX_WIDTH) / 2.0
        for w in (g.SAFE_MIN_WIDTH, mid, g.SAFE_MAX_WIDTH):
            assert g.throat_offset_for(w) == g.tip_offset_for(w)


def test_squeeze_never_drives_past_the_stop(profile):
    """A generous squeeze on a nearly-shut object must clamp, not overrun. With
    the extended fingers the stop is 50 mm and objects start at 53, so this is
    only 3 mm of margin and the clamp does real work."""
    assert profile.grip_angle_for(profile.SAFE_MIN_WIDTH) <= profile.CLOSE_ANGLE
    assert profile.grip_angle_for(profile.SAFE_MAX_WIDTH) <= profile.CLOSE_ANGLE


# ----------------------------------------------------------------- catalogue

def test_the_catalogue_follows_the_fitted_gripper():
    """Which objects are graspable is a property of what is bolted on, so
    fits_gripper() has to be a live question and not a stored verdict. The goal
    object and the development object each work under exactly one profile."""
    can, block = get('soda_can'), get('test_block')
    assert can.fits_gripper() and can.check() is can          # default: extended
    assert not block.fits_gripper()
    with pytest.raises(ObjectError, match='pass straight between'):
        block.check()


def test_the_catalogue_follows_the_other_gripper_too():
    can, block = get('soda_can'), get('test_block')
    with as_profile('stock'):
        assert block.fits_gripper() and block.check() is block
        assert not can.fits_gripper()
        with pytest.raises(ObjectError) as exc:
            can.check()
        assert '66.0 mm' in str(exc.value) and '60.0 mm' in str(exc.value)


def test_defaults_fill_in_and_are_consistent():
    obj = GraspableObject(name='x', shape='box', width=0.02, height=0.05)
    assert obj.depth == 0.02
    assert obj.grasp_width == 0.02
    assert obj.grasp_height == 0.025          # mid-height
    assert obj.centre_offset() == 0.0
    assert obj.box_size == (0.02, 0.02, 0.05)


def test_grasp_dimensions_are_separate_from_bounding_ones():
    """The whole reason the fields are split: gripping a neck, not the body."""
    neck = GraspableObject(name='necked', shape='cylinder', width=0.066,
                           height=0.122, grasp_width=0.053, grasp_height=0.110)
    assert neck.radius == 0.033              # collision geometry: the full body
    assert neck.grasp_width == 0.053         # what the jaws must open to
    assert neck.centre_offset() == pytest.approx(0.061 - 0.110)
    assert neck.fits_gripper()               # the neck fits where the body does not


def test_bad_objects_are_refused_at_construction():
    with pytest.raises(ObjectError, match='shape'):
        GraspableObject(name='x', shape='sphere', width=0.02, height=0.02)
    with pytest.raises(ObjectError, match='depth'):
        GraspableObject(name='x', shape='cylinder', width=0.02, height=0.02,
                        depth=0.03)
    with pytest.raises(ObjectError, match='grasp_height'):
        GraspableObject(name='x', shape='box', width=0.02, height=0.02,
                        grasp_height=0.05)
    with pytest.raises(ObjectError, match='cylinder, not a box'):
        get('soda_can').box_size
    with pytest.raises(ObjectError, match='box, not a cylinder'):
        get('test_block').radius
    with pytest.raises(ObjectError, match='unknown object'):
        get('banana')


def test_catalogue_entries_all_construct():
    for name, obj in CATALOGUE.items():
        assert obj.name == name
        assert obj.height > 0 and obj.width > 0


# ------------------------------------------------------- scene_objects maths

def test_tool_offset_is_read_out_of_the_fk_not_retyped():
    assert scene_objects.TOOL_OFFSET == pytest.approx(
        (-0.00265, 9.7552e-05, 0.068091), abs=1e-9)


def test_grasp_point_is_the_contact_point_not_the_tcp():
    """grasp_point is pure object geometry: no gripper term in it.

    The gripper's contribution is the hover, applied separately with back_off,
    because it is a distance along the TOOL axis rather than a vertical drop --
    and because pick_place searches for it instead of trusting the uncalibrated
    table.
    """
    tall = GraspableObject(name='t', shape='cylinder', width=0.04, height=0.12,
                           grasp_height=0.03)
    assert scene_objects.grasp_point(tall, 0.2, 0.0, 0.10) == pytest.approx(
        (0.2, 0.0, 0.07))


def test_back_off_moves_along_the_tool_axis():
    phi, d = 2.6, 0.02
    p = scene_objects.back_off((0.22, 0.0, 0.0), phi, d)
    assert hypot(hypot(p[0] - 0.22, p[1]), p[2]) == pytest.approx(d)
    assert p[2] > 0.0                      # downward pitch -> backing off rises
    assert p[0] < 0.22                     # and pulls in
    assert scene_objects.back_off((0.22, 0.0, 0.0), phi, 0.0) == pytest.approx(
        (0.22, 0.0, 0.0))


def test_grasp_point_drops_from_the_centre_to_the_grip_height():
    tall = GraspableObject(name='t', shape='cylinder', width=0.04, height=0.12,
                           grasp_height=0.03)
    # centre at z = 0.10 -> base at 0.04 -> grip 30 mm up from there = 0.07
    assert scene_objects.grasp_point(tall, 0.2, 0.0, 0.10) == pytest.approx(
        (0.2, 0.0, 0.07))
    # An object gripped at its middle is gripped at its centre.
    block = get('test_block')
    assert scene_objects.grasp_point(block, 0.2, 0.0, 0.03) == pytest.approx(
        (0.2, 0.0, 0.03))


def test_standoff_backs_off_along_the_tool_axis():
    block = get('test_block')
    phi, d = 2.6, 0.08
    g = scene_objects.grasp_point(block, 0.22, 0.0, 0.03)
    p = scene_objects.standoff_point(block, 0.22, 0.0, 0.03, phi, d)
    # exactly `d` away, and higher and closer in -- never lower
    assert hypot(hypot(p[0] - g[0], p[1] - g[1]), p[2] - g[2]) == pytest.approx(d)
    assert p[2] > g[2]
    assert hypot(p[0], p[1]) < hypot(g[0], g[1])


def test_standoff_follows_the_target_bearing():
    """Off-axis targets must back off along their own radial, not along +x."""
    block = get('test_block')
    p = scene_objects.standoff_point(block, 0.0, 0.22, 0.03, 2.6, 0.08)
    assert p[0] == pytest.approx(0.0, abs=1e-9)
    assert 0.0 < p[1] < 0.22


# What move_group reported for the case in the module docstring, read back out
# of /get_planning_scene: the attached object's pose relative to
# Gripping_point_Link, after MoveIt did the frame conversion itself.
MOVEIT_HELD_POSE = (-0.025706606, -1.1361323e-07, -0.015465136,
                    0.870489667, -1.8079027e-06, -0.492186693, 3.1974871e-06)


def _into_tool_frame(pose):
    """arm5_Link pose -> Gripping_point_Link pose.

    Gripping_Joint sits at TOOL_OFFSET with rpy="3.1416 -1.5708 0", which is the
    rotation matrix [[0,0,1],[0,-1,0],[1,0,0]] -- its own inverse. So the change
    of frame is a translation followed by that permutation, both ways.
    """
    r = ((0.0, 0.0, 1.0), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0))
    d = [p - o for p, o in zip(pose[:3], scene_objects.TOOL_OFFSET)]
    position = tuple(sum(r[i][k] * d[k] for k in range(3)) for i in range(3))
    # Quaternion for r itself: a 180 deg turn about the (1, 0, 1)/sqrt(2) axis.
    h = 2.0 ** -0.5
    rq = (h, 0.0, h, 0.0)
    # q_tool = conj(rq) * q_arm5
    cx, cy, cz, cw = -rq[0], -rq[1], -rq[2], rq[3]
    x, y, z, w = pose[3:]
    return position + (
        cw * x + cx * w + cy * z - cz * y,
        cw * y - cx * z + cy * w + cz * x,
        cw * z + cx * y - cy * x + cz * w,
        cw * w - cx * x - cy * y - cz * z)


def test_held_pose_matches_what_moveit_computed():
    """The cross-check, not just the freeze.

    held_pose is derived in arm5_Link; move_group re-expressed the same attach
    in Gripping_point_Link. Converting ours through the URDF's Gripping_Joint
    must reproduce MoveIt's numbers -- which is what makes this file evidence
    rather than a self-consistent tautology.
    """
    tall = GraspableObject(name='t', shape='cylinder', width=0.04, height=0.12,
                           grasp_height=0.03, symmetric=True)
    assert tall.centre_offset() == pytest.approx(0.03)
    ours = _into_tool_frame(scene_objects.held_pose(tall, 2.6, 0.0))
    # Quaternions double-cover, so accept q or -q. Decide the sign on the
    # LARGEST component: this rotation is a half turn, so w is ~0 and its sign
    # carries no information.
    ref = MOVEIT_HELD_POSE[3:]
    big = max(range(4), key=lambda i: abs(ref[i]))
    flip = -1.0 if ours[3 + big] * ref[big] < 0 else 1.0
    assert ours[:3] == pytest.approx(MOVEIT_HELD_POSE[:3], abs=1e-6)
    # The orientation is a shade looser on purpose. The URDF writes
    # rpy="3.1416 -1.5708 0", which is 7.4 urad off pi and 3.7 urad off pi/2;
    # move_group used those rounded numbers while _into_tool_frame uses the
    # exact right angles, and that difference is worth ~3e-6 per quaternion
    # component. Anything above ~1e-5 would be a real disagreement.
    assert tuple(flip * c for c in ours[3:]) == pytest.approx(
        MOVEIT_HELD_POSE[3:], abs=1e-5)


def test_held_pose_reduces_to_the_vendor_formula():
    """set_scene.cpp's special case falls out of the general form.

    Theirs grips an object at its very top with the object's axis along the tool
    axis, and places it half a height back from the TCP. Feed the same
    conditions in -- grasp_height == height, phi == 0 -- and the general
    derivation must produce exactly that.
    """
    h = 0.03
    obj = GraspableObject(name='v', shape='cylinder', width=0.02, height=h,
                          grasp_height=h - 1e-9)
    x, y, z, qx, qy, qz, qw = scene_objects.held_pose(obj, 0.0, 0.0)
    assert z == pytest.approx(scene_objects.TOOL_OFFSET[2] - h / 2.0, abs=1e-6)
    assert (qx, qy, qz, qw) == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_held_object_stays_world_upright_at_the_grasp():
    """The object's axis must come out vertical when the arm is at the grasp.

    This is the property the whole derivation exists for, so check it against
    the FK rather than against the formula: rotate the object's local +z by the
    held quaternion, then by the wrist's orientation, and it must be world +z.
    """
    obj = GraspableObject(name='t', shape='cylinder', width=0.04, height=0.12,
                          grasp_height=0.03)
    for phi in (1.2, 1.9, 2.6):
        for roll in (0.0, 0.6, -1.1):
            _, _, _, qx, qy, qz, qw = scene_objects.held_pose(obj, phi, roll)
            # object +z in the arm5 frame, from the quaternion
            ax = 2.0 * (qx * qz + qw * qy)
            ay = 2.0 * (qy * qz - qw * qx)
            az = 1.0 - 2.0 * (qx * qx + qy * qy)
            # arm5 -> base is Rz(theta1) Ry(phi) Rz(roll); theta1 only spins the
            # half-plane, so check in the plane: undo Rz(roll) then Ry(phi).
            c, s = cos(roll), sin(roll)
            px, py = ax * c - ay * s, ax * s + ay * c
            up_x = px * cos(phi) + az * sin(phi)
            up_z = -px * sin(phi) + az * cos(phi)
            assert isclose(up_x, 0.0, abs_tol=1e-9)
            assert isclose(py, 0.0, abs_tol=1e-9)
            assert isclose(up_z, 1.0, abs_tol=1e-9)


def test_held_pose_sits_on_the_tool_axis_when_gripped_at_mid_height():
    """centre_offset == 0 puts the object centre on the contact point."""
    block = get('test_block')
    assert block.centre_offset() == 0.0
    assert scene_objects.held_pose(block, 2.6, 0.4)[:3] == pytest.approx(
        scene_objects.TOOL_OFFSET, abs=1e-12)
    # ...and a hover pushes it that far further out along the tool axis, which
    # is +z in arm5_Link, not along world z.
    hovered = scene_objects.held_pose(block, 2.6, 0.4, 0.02)[:3]
    assert hovered[2] - scene_objects.TOOL_OFFSET[2] == pytest.approx(0.02)
    assert hovered[:2] == pytest.approx(scene_objects.TOOL_OFFSET[:2], abs=1e-12)


def test_a_full_floor_grasp_geometry_is_self_consistent():
    """grasp_point -> IK -> FK must land back on the grasp point."""
    block = get('test_block')
    phi = 2.6
    g = scene_objects.grasp_point(block, 0.22, 0.0, 0.025) + (phi,)
    joints = kin.ik_best(*g)
    assert joints is not None, kin.describe(*g)
    assert kin.fk(joints)[:3] == pytest.approx(g[:3], abs=1e-9)
    assert kin.fk(joints)[3] == pytest.approx(phi, abs=1e-12)
    # and the standoff is on the same tool axis, further out
    p = scene_objects.standoff_point(block, 0.22, 0.0, 0.025, phi, 0.08) + (phi,)
    assert kin.ik_best(*p) is not None, kin.describe(*p)
    assert p[2] - g[2] == pytest.approx(-0.08 * cos(phi))


def test_symmetric_flag_is_carried_not_ignored():
    assert get('soda_can').symmetric is True
    assert get('test_block').symmetric is False
