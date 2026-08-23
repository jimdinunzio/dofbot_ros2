#!/usr/bin/env python3
# coding: utf-8
"""
The named arm states exist twice: as group_states in dofbot_description.srdf,
which is the list RViz's MoveIt panel offers, and as NAMED_STATES in
moveit_client.py, which is what pick_place and move_to_state plan against.

    pytest src/dofbot_ros2/dofbot_ctrl/test/test_named_states.py

Two copies of the same numbers drift, and drift silently: retuning 'carry' and
'over_trash' on the arm changed the Python and left the SRDF alone, so the RViz
dropdown sent the arm to one posture and a pick sent it to another. Nothing
caught it, because nothing compared them. This does.

Every value here is read from one side and asserted against the other. There
are no pose literals in this file on purpose -- a transcription check that
transcribes the numbers a third time is one more copy to drift.
"""

import os
import xml.etree.ElementTree as ET

import pytest

from dofbot_ctrl import dofbot_kinematics as kin

moveit_client = pytest.importorskip(
    'dofbot_ctrl.moveit_client',
    reason='needs moveit_msgs on the path; source the workspace')

_SRDF = os.path.join(os.path.dirname(__file__), '..', '..',
                     'dofbot_moveit', 'config', 'dofbot_description.srdf')

needs_srdf = pytest.mark.skipif(
    not os.path.exists(_SRDF),
    reason='dofbot_moveit/config/dofbot_description.srdf not found next to '
           'this package')


def srdf_states():
    """{name: {joint: value}} for every group_state on the arm group."""
    root = ET.parse(_SRDF).getroot()
    return {state.get('name'): {j.get('name'): float(j.get('value'))
                                for j in state.findall('joint')}
            for state in root.findall('group_state')
            if state.get('group') == moveit_client.ARM_GROUP}


@needs_srdf
def test_srdf_and_named_states_have_the_same_names():
    assert sorted(srdf_states()) == sorted(moveit_client.NAMED_STATES)


@needs_srdf
@pytest.mark.parametrize('name', sorted(moveit_client.NAMED_STATES))
def test_srdf_matches_named_states(name):
    """The two copies agree joint by joint, in ARM_JOINT_NAMES order."""
    srdf = srdf_states()[name]
    ours = moveit_client.NAMED_STATES[name]
    for joint, q in zip(moveit_client.ARM_JOINT_NAMES, ours):
        assert srdf[joint] == pytest.approx(q, abs=1e-9), '%s/%s' % (name, joint)


@needs_srdf
@pytest.mark.parametrize('name', sorted(moveit_client.NAMED_STATES))
def test_srdf_names_every_arm_joint(name):
    """A group_state may legally omit joints; ours must not.

    An omitted joint is left at whatever the state it is planned FROM had, so a
    partial group_state is a pose that depends on where the arm already was.
    """
    assert (sorted(srdf_states()[name])
            == sorted(moveit_client.ARM_JOINT_NAMES))


@pytest.mark.parametrize('name', sorted(moveit_client.NAMED_STATES))
def test_named_states_are_inside_the_joint_limits(name):
    """Reachability only. Whether a state COLLIDES needs the planning scene,
    which is what `pick_place --check-states` asks move_group for."""
    joints = moveit_client.NAMED_STATES[name]
    assert len(joints) == len(kin.JOINT_NAMES)
    for joint, q, (lo, hi) in zip(kin.JOINT_NAMES, joints, kin.JOINT_LIMITS):
        assert lo <= q <= hi, '%s/%s = %.4f not in %.4f..%.4f' % (
            name, joint, q, lo, hi)
