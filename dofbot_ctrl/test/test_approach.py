#!/usr/bin/env python3
# coding: utf-8
"""
The approach search: which pitch, grip height and standoff a pick chooses.

WHAT THESE ARE ABOUT is the difference between "solvable" and "workable". A
grasp near the inner edge of the working ring solves and is still useless: the
arm arrives there folded up against a joint stop, with no room to back out
along a line, no room to lift, and nothing left to absorb the error in where the
object really is. So these check joint margin and the length of the approach,
not just whether ik_best returned something.

Written as RELATIONS against the module's own constants rather than against
remembered numbers -- the near edge is derived by scanning, not pinned, so
re-tuning min_standoff moves the expectation with it.

pick_place imports rclpy and the MoveIt messages. That is an import, not a
connection -- the harness below never builds a Node and never touches the ROS
graph -- so these run without a robot, and skip cleanly outside a sourced
workspace.
"""

import types
from math import pi

import pytest

from dofbot_ctrl import dofbot_kinematics as kin
from dofbot_ctrl.graspable import get

pick_place = pytest.importorskip(
    'dofbot_ctrl.pick_place',
    reason='needs rclpy and moveit_msgs on the path; source the workspace')
scene_objects = pytest.importorskip('dofbot_ctrl.scene_objects')

# The can standing on the floor, the case the whole sequence exists for. Its
# centre is half its height up, which is what pick() is given.
CAN = get('soda_can')
FLOOR_Z = CAN.height / 2.0


class _Sweep(pick_place.PickPlace):
    """_feasible_approach with the ROS parts stubbed out.

    Deliberately NOT calling Node.__init__: the search is pure kinematics apart
    from one /check_state_validity call per ranked survivor, and stubbing that
    to "always valid" isolates what these tests are about -- reach and joint
    limits -- from what the planning scene happens to contain.
    """

    def __init__(self, valid=True, **override):
        self._params = dict(
            grasp_pitch=2.2, pitch_min=pi / 2.0, pitch_max=pi, pitch_step=0.01,
            standoff=0.08, min_standoff=0.02, standoff_step=0.005,
            grasp_height_step=0.005, lift=0.10, min_lift=0.03, min_margin=0.02)
        self._params.update(override)
        self.logged = []
        self.mc = types.SimpleNamespace(
            check_state=lambda joints, grip: (valid, [] if valid else ['stub']))

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._params[name])

    def get_logger(self):
        sweep = self

        def record(level):
            return lambda msg: sweep.logged.append((level, msg))

        return types.SimpleNamespace(info=record('info'), warn=record('warn'),
                                     error=record('error'))


def solve(x, **override):
    """(phi, pre, grasp, lift, hover, height) or None if nothing works."""
    try:
        return _Sweep(**override)._feasible_approach(CAN, x, 0.0, FLOOR_Z)
    except pick_place.MoveItError:
        return None


def near_edge(step=0.005, **override):
    """The closest x, to `step`, at which a whole approach is workable."""
    x = 0.15
    while x < 0.40:
        if solve(x, **override) is not None:
            return x
        x += step
    return None


# ------------------------------------------------------- _reachable_standoff


def test_the_standoff_is_measured_and_stays_inside_its_bounds():
    sweep = _Sweep()
    want = sweep._params['standoff']
    floor = sweep._params['min_standoff']
    hover = sweep._hover(CAN)
    for x in (0.22, 0.26, 0.30):
        for phi in (2.1, 2.2, 2.3):
            contact = scene_objects.grasp_point(CAN, x, 0.0, FLOOR_Z)
            grasp = scene_objects.back_off(contact, phi, hover) + (phi,)
            if kin.ik_best(*grasp) is None:
                continue
            got = sweep._reachable_standoff(contact, grasp, phi, hover, want,
                                            floor, sweep._params['standoff_step'])
            if got is None:
                continue
            reach, pre, margin = got
            # never more than asked for, never less than the floor
            assert floor - 1e-9 <= reach <= want + 1e-9
            # and the pose it returns is the one it measured
            assert kin.ik_best(*pre) is not None
            assert pre[3] == phi
            assert margin > 0.0


def test_the_standoff_shortens_as_the_target_comes_closer():
    """Room for the approach runs out gradually as the target comes in."""
    sweep = _Sweep()
    reaches = []
    for x in (0.22, 0.25, 0.28, 0.31):
        got = solve(x)
        assert got is not None, x
        # recover the standoff from the poses: pre and grasp differ by exactly
        # that much along the tool axis
        pre, grasp = got[1], got[2]
        reaches.append(round(sum((a - b) ** 2
                                 for a, b in zip(pre[:3], grasp[:3])) ** 0.5, 6))
    assert reaches == sorted(reaches), reaches
    assert reaches[-1] == pytest.approx(sweep._params['standoff'], abs=1e-6)


def test_no_standoff_is_reported_rather_than_a_short_one():
    sweep = _Sweep()
    hover = sweep._hover(CAN)
    # straight up out of the base: the grasp may solve, but there is nowhere to
    # back off to, and the answer must be None rather than something below the
    # floor that a caller would then use.
    contact = (0.02, 0.0, 0.05)
    phi = 2.2
    grasp = scene_objects.back_off(contact, phi, hover) + (phi,)
    assert sweep._reachable_standoff(contact, grasp, phi, hover, 0.08, 0.02,
                                     0.005) is None


# ------------------------------------------------------------ the near edge


def test_measuring_the_standoff_reaches_closer_than_demanding_it():
    """Measuring the standoff is worth several centimetres of near reach.

    Derived both ways rather than pinned: forcing min_standoff up to the full
    standoff is exactly "demand the whole 80 mm", so both edges are measured by
    the same code on the same day.
    """
    sweep = _Sweep()
    fixed = near_edge(min_standoff=sweep._params['standoff'])
    measured = near_edge()
    assert measured is not None and fixed is not None
    assert measured < fixed
    # and the gain is the interesting part, not a rounding artefact
    assert fixed - measured > 0.03


def test_the_near_edge_is_set_by_the_standoff_floor():
    """Lowering the floor must buy reach; raising it must cost reach.

    This is what says the near edge is a POLICY choice about how short an
    approach is acceptable, not a hard fact about the arm.
    """
    tight = near_edge(min_standoff=0.01)
    normal = near_edge()
    loose = near_edge(min_standoff=0.05)
    assert tight < normal < loose


def test_below_the_near_edge_the_grasp_still_solves():
    """Just inside the edge the arm can still STRIKE the grasp -- it cannot back
    out of it. This is why the failure has to name the standoff: a message that
    blames reach sends you to move the base closer, the wrong direction.
    """
    sweep = _Sweep()
    edge = near_edge()
    inside = edge - 0.02
    assert solve(inside) is None
    hover = sweep._hover(CAN)
    struck = [
        phi for phi in (2.1 + 0.01 * i for i in range(30))
        for contact in [scene_objects.grasp_point(CAN, inside, 0.0, FLOOR_Z)]
        if kin.ik_best(*(scene_objects.back_off(contact, phi, hover) + (phi,)))
        is not None]
    assert struck, 'no grasp at all inside the edge -- premise gone'


def test_the_failure_names_the_stage_that_ran_out():
    edge = near_edge()
    with pytest.raises(pick_place.MoveItError) as exc:
        _Sweep()._feasible_approach(CAN, edge - 0.02, 0.0, FLOOR_Z)
    said = str(exc.value)
    assert 'the grasp solves' in said
    assert 'straight-line approach' in said
    # and it points the right way
    assert 'FURTHER AWAY' in said
    assert 'move the base closer' not in said


def test_a_target_genuinely_out_of_reach_still_says_so():
    limits = kin.reach_limits()
    far = limits['max_reach_from_shoulder'] * 2.0
    with pytest.raises(pick_place.MoveItError, match='move the base closer'):
        _Sweep()._feasible_approach(CAN, far, 0.0, FLOOR_Z)


# ---------------------------------------------------------- what it prefers


def test_the_proven_grip_height_wins_wherever_it_works():
    """The reason the score is quantised: a margin difference of a thousandth of
    a radian is inside the arm's own backlash and must not outvote the height
    proven on hardware. Across the working range the nominal wins nearly always.
    """
    chosen = [solve(x)[5] for x in (0.21, 0.22, 0.24, 0.25, 0.26, 0.30, 0.33)
              if solve(x) is not None]
    assert chosen
    nominal = sum(1 for h in chosen if h == CAN.grasp_height)
    assert nominal >= len(chosen) - 1, chosen


def test_a_chosen_height_is_always_one_the_object_offers():
    for x in (0.21, 0.24, 0.28, 0.33):
        got = solve(x)
        if got is None:
            continue
        assert got[5] in CAN.grasp_heights()


def test_the_choice_has_joint_room_across_the_working_range():
    """'Solvable' is not 'workable'. Every pose in the returned approach has to
    be off its stop, or there is nothing left to absorb the pose error."""
    for x in (0.22, 0.24, 0.26, 0.28, 0.30):
        got = solve(x)
        assert got is not None, x
        _phi, pre, grasp, _lift, _hover, _height = got
        for pose in (pre, grasp):
            joints = kin.ik_best(*pose)
            assert joints is not None
            assert pick_place._joint_margin(joints) > 0.0
            assert kin.in_limits(joints)


def test_a_thin_posture_is_reported_rather_than_hidden():
    sweep = _Sweep(min_margin=1.0)         # nothing can satisfy this
    sweep._feasible_approach(CAN, 0.26, 0.0, FLOOR_Z)
    warned = [m for level, m in sweep.logged if level == 'warn']
    assert any('joint stop' in m for m in warned), warned


def test_the_lift_is_reported_but_never_required():
    """The lift is not what clears the floor -- move_named('carry') raises the
    object on its own -- so a pick with no straight-up lift at all must still be
    offered, and said."""
    got = solve(0.35)
    assert got is not None
    lift = got[3]
    assert lift >= 0.0
    sweep = _Sweep(min_lift=1.0)           # more than any pose can give
    assert sweep._feasible_approach(CAN, 0.26, 0.0, FLOOR_Z) is not None
    warned = [m for level, m in sweep.logged if level == 'warn']
    assert any('lift' in m for m in warned), warned


def test_the_returned_grasp_is_the_contact_point_backed_off_by_the_hover():
    """The one relation the whole sequence rests on: the TCP target is NOT the
    grasp point. Getting the sign wrong drives the arm into the object."""
    got = solve(0.26)
    assert got is not None
    phi, _pre, grasp, _lift, hover, height = got
    at_height = pick_place.replace(CAN, grasp_height=height)
    contact = scene_objects.grasp_point(at_height, 0.26, 0.0, FLOOR_Z)
    assert grasp[:3] == pytest.approx(
        scene_objects.back_off(contact, phi, hover))
    # backed off TOWARD the wrist, so the TCP is short of the contact point
    assert (grasp[0] ** 2 + grasp[1] ** 2) ** 0.5 < (
        contact[0] ** 2 + contact[1] ** 2) ** 0.5


def test_a_collision_at_every_candidate_is_reported_as_a_collision():
    with pytest.raises(pick_place.MoveItError, match='collide'):
        _Sweep(valid=False)._feasible_approach(CAN, 0.26, 0.0, FLOOR_Z)


def test_the_scene_is_not_asked_more_than_the_cap():
    calls = []
    sweep = _Sweep(valid=False)
    sweep.mc = types.SimpleNamespace(
        check_state=lambda joints, grip: (calls.append(joints), (False, ['x']))[1])
    with pytest.raises(pick_place.MoveItError):
        sweep._feasible_approach(CAN, 0.26, 0.0, FLOOR_Z)
    assert len(calls) <= pick_place.MAX_STATE_CHECKS
