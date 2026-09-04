#!/usr/bin/env python3
# coding: utf-8
"""
The wave gesture has to be a gesture MoveIt will actually execute.

    pytest src/dofbot_ros2/dofbot_ctrl/test/test_wave_poses.py

Two things have already gone wrong here, and both are cheap to catch:

  LIMITS. The poses started life as servo angles hand-posed on the real arm,
  bounded by the SERVO's travel rather than the model's. One of them converted
  to arm3_Joint = 1.606 rad against a +-1.570 URDF limit -- 2 degrees out,
  invisible on the arm, and a goal move_joints rejects before OMPL is asked.

  THE CHASSIS. Those poses folded the arm BACK over the base, which was fine
  when the model was the arm alone. Once chassis.xacro made the robot body a
  real link, the swing failed with `arm4_Link/chassis_link`. The gesture was
  rebuilt around one forward-leaning shape so that no link ever reaches behind
  the shoulder -- that is the property these tests pin, because it is what makes
  yawing the arm safe rather than a thing that has to be re-checked by hand.

Whether a pose COLLIDES is still a question for the live scene, and wave_arm
asks it at run time with /check_state_validity. These tests are the reachability
and geometry half, which needs nothing running.
"""

import math
import os
import re

import pytest

from dofbot_ctrl import dofbot_kinematics as kin

wave_arm = pytest.importorskip(
    'dofbot_ctrl.wave_arm',
    reason='needs rclpy and moveit_msgs on the path; source the workspace')
moveit_client = pytest.importorskip('dofbot_ctrl.moveit_client')

POSES = ('RAISED', 'WAVE_A', 'WAVE_B')
LEGS = [('RAISED', 'WAVE_A'), ('WAVE_A', 'WAVE_B'),
        ('WAVE_B', 'WAVE_A'), ('WAVE_A', 'RAISED')]
YAW, ROLL = 0, 4                    # indices into a five-joint pose

_XACRO = os.path.join(os.path.dirname(__file__), '..', '..',
                      'dofbot_description', 'urdf', 'chassis.xacro')


def _chassis_geometry():
    """(x of the axis, radius, z of the base, height) from chassis.xacro.

    The defaults live in the macro's params list as `name:=value`. Read rather
    than retyped: these are measurements of the real robot and they have already
    changed once.
    """
    with open(_XACRO) as fh:
        text = fh.read()
    def param(name):
        match = re.search(r'\b%s:=([0-9.]+)' % name, text)
        assert match, 'no %s default in chassis.xacro' % name
        return float(match.group(1))
    return -param('back_offset'), param('radius'), param('z_bottom'), param('height')


def pose(name):
    return getattr(wave_arm, name)


def link_radii(joints):
    """In-plane radius of each link origin, out from the shoulder yaw axis.

    The same planar decomposition kin.fk sums, stopped at each joint instead of
    only at the tool. Negative means that link reaches BEHIND the shoulder,
    which is where the chassis is.
    """
    _, t2, t3, t4, t5 = joints
    out = [0.0, kin.L1 * math.sin(t2)]
    out.append(out[-1] + kin.L2 * math.sin(t2 + t3))
    for tip in ('arm5', 'tcp'):
        dx, dz, _lat = kin._last_segment(t5, tip)
        out.append(out[2] + math.hypot(dx, dz)
                   * math.sin(t2 + t3 + t4 + math.atan2(dx, dz)))
    return out


@pytest.mark.parametrize('name', POSES)
def test_wave_poses_are_inside_the_joint_limits(name):
    joints = pose(name)
    assert len(joints) == len(kin.JOINT_NAMES)
    for joint, q, (lo, hi) in zip(kin.JOINT_NAMES, joints, kin.JOINT_LIMITS):
        assert lo <= q <= hi, '%s/%s = %.4f not in %.4f..%.4f' % (
            name, joint, q, lo, hi)


# chassis.xacro: a z-axis cylinder behind the arm, and the thing the original
# gesture swung into. Read from the xacro rather than retyped so a chassis that
# is re-measured moves this test with it.
CHASSIS_X, CHASSIS_R, CHASSIS_Z_BOTTOM, CHASSIS_H = _chassis_geometry()

# The pose that failed in the planning scene had its nearest link 26 mm from the
# cylinder CENTRELINE, which the link meshes more than fill. This is a
# centreline figure too, so it is not a collision proof -- /check_state_validity
# at run time is that. It is a floor: three times the margin of the one that is
# known to fail, and enough that the gesture is not relying on luck.
MIN_CLEARANCE = 0.075


def link_points(joints):
    """Shoulder, elbow, wrist, arm5 and tool origins in base_link.

    The same planar decomposition kin.fk sums, stopped at each joint instead of
    only at the tool, then yawed out of the plane.
    """
    t1, t2, t3, t4, t5 = joints
    rz = [(0.0, kin.Z0)]
    r = kin.L1 * math.sin(t2)
    z = kin.Z0 + kin.L1 * math.cos(t2)
    rz.append((r, z))
    r += kin.L2 * math.sin(t2 + t3)
    z += kin.L2 * math.cos(t2 + t3)
    rz.append((r, z))
    for tip in ('arm5', 'tcp'):
        dx, dz, _lat = kin._last_segment(t5, tip)
        l3, delta = math.hypot(dx, dz), math.atan2(dx, dz)
        rz.append((rz[2][0] + l3 * math.sin(t2 + t3 + t4 + delta),
                   rz[2][1] + l3 * math.cos(t2 + t3 + t4 + delta)))
    return [(r * math.cos(t1), r * math.sin(t1), z) for r, z in rz]


def chassis_clearance(joints, samples=40):
    """Least distance from any link centreline to the chassis cylinder."""
    points = link_points(joints)
    best = float('inf')
    z_top = CHASSIS_Z_BOTTOM + CHASSIS_H
    for p, q in zip(points, points[1:]):
        for i in range(samples + 1):
            t = i / samples
            x, y, z = (a + (b - a) * t for a, b in zip(p, q))
            radial = math.hypot(x - CHASSIS_X, y) - CHASSIS_R
            if CHASSIS_Z_BOTTOM <= z <= z_top:
                d = radial
            elif radial <= 0.0:
                d = CHASSIS_Z_BOTTOM - z if z < CHASSIS_Z_BOTTOM else z - z_top
            else:
                dz = CHASSIS_Z_BOTTOM - z if z < CHASSIS_Z_BOTTOM else z - z_top
                d = math.hypot(radial, dz)
            best = min(best, d)
    return best


@pytest.mark.parametrize('pair', LEGS)
def test_the_whole_swing_clears_the_chassis(pair):
    """The bug this gesture was rebuilt around.

    Checking the poses alone would not have caught it: the original's endpoints
    passed and it was waypoint 4 of 17 along a leg that hit `arm4_Link`. So walk
    the legs, not just their ends.
    """
    start, end = (pose(n) for n in pair)
    for joints in [start] + wave_arm.interpolate(start, end):
        clear = chassis_clearance(joints)
        assert clear >= MIN_CLEARANCE, (
            '%s -> %s passes %.3f m from the chassis centreline at yaw %.1f deg'
            % (pair[0], pair[1], clear, math.degrees(joints[YAW])))


@pytest.mark.parametrize('name', POSES)
def test_shoulder_and_elbow_hold_still(name):
    """Only arm1, arm4 and arm5 animate: the base swings, the wrist pitches and
    rolls. The shoulder and elbow are what hold the hand overhead and out of the
    chassis, so a pose that moves them is one whose clearance has to be argued
    about rather than inherited."""
    assert pose(name)[1:3] == wave_arm.BASE_PITCH[:2]


def test_the_wrist_flicks_rather_than_sweeping_rigidly():
    """The hand tips one way at one end of the swing and the other way at the
    other. Without it the gesture is the base rotating under a fixed arm."""
    a, b = pose('WAVE_A'), pose('WAVE_B')
    assert a[3] - wave_arm.BASE_PITCH[2] == pytest.approx(wave_arm.PITCH_FLICK)
    assert wave_arm.BASE_PITCH[2] - b[3] == pytest.approx(wave_arm.PITCH_FLICK)


def test_the_swing_is_symmetric_about_the_raised_pose():
    """RAISED is the centre of the swing, not a fourth pose -- the arm passes
    through it mid-wave, and the entry and exit legs are then the same size."""
    assert pose('RAISED')[YAW] == 0.0
    assert pose('WAVE_A')[YAW] == -pose('WAVE_B')[YAW] == wave_arm.SWING
    assert pose('WAVE_A')[ROLL] == -pose('WAVE_B')[ROLL] == wave_arm.FLICK


def test_the_swing_is_wide_enough_to_read_as_a_wave():
    swing = max(abs(a - b) for a, b in zip(pose('WAVE_A'), pose('WAVE_B')))
    assert swing > wave_arm.CHECK_STEP


def test_the_tool_is_held_up():
    """A greeting is made above the robot, not out over the pick zone."""
    for name in POSES:
        z = kin.fk(pose(name))[2]
        assert z > kin.Z0 + 0.2, '%s holds the tool at only %.3f m' % (name, z)


def test_the_default_finish_is_a_real_named_state():
    """--finish is validated against NAMED_STATES, so a default outside it
    would make a plain `wave_arm` with no arguments unrunnable."""
    assert wave_arm.DEFAULT_FINISH in moveit_client.NAMED_STATES


@pytest.mark.parametrize('pair', LEGS)
def test_interpolate_steps_no_further_than_check_step(pair):
    """Every waypoint on a leg is validated, so the gaps between them are the
    resolution at which the path is checked at all."""
    start, end = (pose(n) for n in pair)
    points = wave_arm.interpolate(start, end)
    assert points[-1] == pytest.approx(end)
    prev = start
    for point in points:
        assert max(abs(a - b) for a, b in zip(point, prev)) <= (
            wave_arm.CHECK_STEP + 1e-9)
        prev = point
