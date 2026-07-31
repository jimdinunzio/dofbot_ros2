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

from math import cos, hypot, isclose, sin

import pytest

from dofbot_ctrl import gripper
from dofbot_ctrl import dofbot_kinematics as kin
from dofbot_ctrl.graspable import CATALOGUE, GraspableObject, ObjectError, get

scene_objects = pytest.importorskip(
    'dofbot_ctrl.scene_objects',
    reason='needs moveit_msgs on the path; source the workspace')


# ------------------------------------------------------------------- gripper

def test_endpoints_are_the_measured_ones():
    """0 mm shut, 60 mm open, matching the SRDF open/close states."""
    assert gripper.jaw_width_for(gripper.OPEN_ANGLE) == pytest.approx(0.060)
    assert gripper.jaw_width_for(gripper.CLOSE_ANGLE) == pytest.approx(0.0)
    assert gripper.jaw_angle_for(0.060 - gripper.CLEARANCE) < gripper.CLOSE_ANGLE
    assert gripper.MAX_WIDTH == pytest.approx(0.060)


def test_width_and_angle_invert_each_other():
    for mm in range(0, 58):
        w = mm / 1000.0
        assert gripper.jaw_width_for(gripper.jaw_angle_for(w)) == pytest.approx(
            w, abs=1e-9)


def test_wider_object_means_a_smaller_angle():
    """Angle increases as the jaws close, so it must fall as width rises."""
    angles = [gripper.jaw_angle_for(mm / 1000.0) for mm in range(0, 58, 5)]
    assert all(a > b for a, b in zip(angles, angles[1:]))


def test_rejects_what_it_cannot_open_to():
    with pytest.raises(gripper.GripperError, match='66.0 mm'):
        gripper.jaw_angle_for(0.066)          # the can, on the body
    with pytest.raises(gripper.GripperError):
        gripper.jaw_angle_for(gripper.SAFE_MAX_WIDTH + 1e-4)
    with pytest.raises(gripper.GripperError, match='negative'):
        gripper.jaw_angle_for(-0.001)
    assert not gripper.fits(0.066)
    assert gripper.fits(0.030)


def test_grip_angle_squeezes_past_contact():
    """The commanded angle must be tighter than exact contact, or nothing grips."""
    contact = gripper.jaw_angle_for(0.030)
    assert gripper.grip_angle_for(0.030) > contact
    assert gripper.grip_angle_for(0.030) == pytest.approx(
        gripper.jaw_angle_for(0.030 - gripper.DEFAULT_SQUEEZE))
    # A nearly-shut object must not ask for an angle past the stop.
    assert gripper.grip_angle_for(0.001) <= gripper.CLOSE_ANGLE


def test_tip_offset_is_flagged_uncalibrated_not_silently_zero():
    assert gripper.CALIBRATED is False
    assert gripper.tip_offset_for(0.030) == 0.0
    assert 'UNCALIBRATED' in gripper.describe(0.030)


# ----------------------------------------------------------------- catalogue

def test_can_is_rejected_by_this_gripper_with_a_reason():
    """The plan's headline constraint, pinned: 66 mm will not go in 60 mm."""
    can = get('soda_can')
    assert not can.fits_gripper()
    with pytest.raises(ObjectError) as exc:
        can.check()
    assert '66.0 mm' in str(exc.value) and '60.0 mm' in str(exc.value)
    assert 'gripper wider' in str(exc.value)         # the catalogue note


def test_block_is_accepted():
    block = get('test_block')
    assert block.fits_gripper()
    assert block.check() is block


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
