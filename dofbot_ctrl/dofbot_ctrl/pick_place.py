#!/usr/bin/env python3
# coding: utf-8
"""
Pick and place, driven by coordinates rather than by RViz.

    # a soda can standing on the floor: (x, y, z) is its CENTRE, so z is half
    # the catalogue height. The extensions want it further out than the stock
    # jaws did -- see the reach note below.
    ros2 run dofbot_ctrl pick_place -- 0.30 0.0 0.061
    ros2 run dofbot_ctrl pick_place -- --plan-only 0.30 0.0 0.061
    # the test block needs the stock jaws; the extensions cannot close on it
    DOFBOT_GRIPPER=stock ros2 run dofbot_ctrl pick_place -- \
        --object test_block 0.22 0.0 0.015
    ros2 run dofbot_ctrl pick_place -- --check-states

`pick(x, y, z)` is the whole point of this file, and the seam the perception
layer will attach to. (x, y, z) is the CENTRE of the object in base_link -- so
for something resting on the floor, z is half the catalogue height, NOT 0. A
depth camera returns a point on the near surface, so whoever calls this steps
along the view ray by the object radius first -- deliberately, once, in the
caller, instead of Yahboom's blind +0.02/+0.01/+0.01 fudge constants which
partly encode that correction and partly encode calibration error.

SEQUENCE
--------
    move_named('ready') + open_gripper
    add the object to the planning scene
    move_pose  to a pre-grasp standoff back along the tool axis  (OMPL plans it)
    cartesian_move straight in to the grasp                      (we plan it)
    close_gripper to the object's grasp width
    attach, with every end-effector link named as a touch link
    cartesian_move straight up, clear of the floor
    move_named('carry')

then place(): over_trash -> open -> detach + remove -> carry.

GRASP PITCH
-----------
phi is the tool tilt from vertical (see dofbot_kinematics): 0 is straight up,
pi is straight down. It is a parameter because the right value depends on the
object and where it sits, and the reachable band is narrow -- at a grasp point
on the floor, only phi in roughly 2.4-3.0 works at all, and only part of that
leaves room for an 80 mm straight-line approach.

The vendor numbers do NOT transfer as-is, and this is why: theirs is the angle
between the tool and the TABLE, ours is measured from vertical, and their IK
solves to the arm5 rotation centre rather than the TCP. `pitch = 1.04` in the
nano code and `1.3963` in the Pro's grasp_desktop.py are therefore not phi
values. The default here was derived from our own reach instead -- see
_feasible_approach, which sweeps outward from the configured pitch and reports
which one it used, so tuning on hardware is a matter of reading the log.

HOVER: THE TCP IS NOT WHERE THE JAWS GRIP
----------------------------------------
The object is held BETWEEN THE FINGERTIPS, and the fingertips are nowhere near
Gripping_point_Link. That frame is fixed to the wrist, while the fingers reach
PAST it and by a varying amount, because the four-bar swings them outward along
the tool axis as it closes.

So the TCP target is the grasp point pulled back along the tool axis by
gripper.tip_offset_for(width). Get this wrong in the optimistic direction and
the arm drives itself into the object up to the knuckles rather than pinching it
at the tips -- which is exactly what it looks like in RViz.

Which gripper is fitted is set by DOFBOT_GRIPPER, and dofbot.urdf reads the same
variable. Nothing in this file needs to know which one it is, but if those two
disagree the hover is wrong by the whole length of the extensions.

LONGER FINGERS MOVE THE WORKING RING OUT, THEY DO NOT SHRINK IT
---------------------------------------------------------------
Worth knowing before chasing an unreachable target the wrong way. The hover backs
the TCP off ALONG THE TOOL AXIS, and at a steep grasp pitch that direction is
inward and upward, not outward. What limits these grasps is therefore the arm's
MINIMUM radius, its inability to fold up tight, and a longer finger pushes the
wrist further into it.

So the reachable band for the can translates rather than narrowing. Swept
offline against ik_best, it keeps its width and moves outward by roughly the
increase in hover:

    stock jaws        object x = 0.17 .. 0.28 m
    extended fingers  object x = 0.25 .. 0.36 m

The near edge is the one that bites: a can comfortably reachable on the stock
jaws can be unreachable at ANY pitch with the extensions on. When a grasp will
not solve, the fix is usually to move the base FURTHER AWAY, which is the
opposite of the instinct. _feasible_approach reports the pitches it tried, so
the log tells you which edge you are against.

grasp_height is not sensitive over that sweep; the band barely moves across the
usable range of grip heights. (Reachability only. Whether a tall can fouls the
wrist once attached is a collision question, and only move_group answers it.)

Related, and the same mistake in a different place: never collision-check the
COMMANDED jaw angle. grip_angle_for() deliberately asks for narrower than the
object so the servo loads up against it, but the object stops the jaws at its
own width, so the commanded angle is a pose the gripper never occupies while
holding anything. Check jaw_angle_for(width).
"""

import argparse
import sys
from math import atan2, degrees, hypot, pi

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from dofbot_ctrl import dofbot_kinematics as kin
from dofbot_ctrl import graspable, gripper, scene_objects
from dofbot_ctrl.moveit_client import (GRIPPER_LINKS, NAMED_STATES,
                                       DofbotMoveIt, MoveItError)


class PickPlace(Node):

    def __init__(self):
        super().__init__('pick_place')

        self.declare_parameter('object', 'soda_can')
        self.declare_parameter('standoff', 0.08)      # pre-grasp, m along tool
        self.declare_parameter('grasp_pitch', 2.6)    # phi, rad from vertical
        # Sweep the whole range that can reach DOWN onto a supported object:
        # pi/2 is a horizontal tool, pi is straight down. Anything shallower
        # points the tool upwards and cannot grasp something standing on a
        # surface. Candidates are tried nearest-to-grasp_pitch first.
        self.declare_parameter('pitch_min', pi / 2.0)
        self.declare_parameter('pitch_max', pi)
        # Must be fine enough not to step OVER the feasible band. At a
        # floor-level grasp that band can be a couple of hundredths of a radian
        # wide, and a coarse step straddles it and reports "no workable
        # approach" for a pose the arm reaches comfortably. Stepping finer is
        # nearly free: candidates are screened by analytic IK first, and only
        # survivors cost a /check_state_validity round trip.
        self.declare_parameter('pitch_step', 0.01)
        self.declare_parameter('lift', 0.10)          # straight-up retreat, m
        self.declare_parameter('min_lift', 0.03)      # enough to clear the floor

        self.mc = DofbotMoveIt(self)
        self.held = None            # the object currently attached, if any

    # ------------------------------------------------------------------ setup

    def _object(self, name=None):
        obj = graspable.get(name or self.get_parameter('object').value)
        obj.check()
        return obj

    def _reachable_lift(self, grasp, want, step=0.005):
        """How far straight up the tool can go from `grasp`, at most `want`.

        The lift has to be measured, not assumed. At a steep grasp pitch the
        reachable band in z is only a few centimetres deep, so a fixed retreat
        is unreachable about as often as not. Taking what is available and
        reporting it beats failing the whole pick over the last few mm of a
        move whose only job is to clear the floor.
        """
        best = 0.0
        d = step
        while d <= want + 1e-9:
            if kin.ik_best(grasp[0], grasp[1], grasp[2] + d, grasp[3]) is None:
                break
            best = d
            d += step
        return best

    def _hover(self, obj):
        """How far back along the tool axis the TCP sits from the grasp point.

        The object is held BETWEEN THE FINGERTIPS, and the fingertips are not at
        Gripping_point_Link. That frame is fixed to the wrist while the fingers
        reach past it, by more as they close, because the four-bar swings them
        outward along the tool axis. So the arm must stop the TCP short of the
        object by exactly that much, which is gripper.tip_offset_for() at the
        opening the object holds the jaws to.

        This is a lookup, not a search. Do not replace it with a search for
        the smallest collision-free backoff: that finds the DEEPEST reach which
        does not trip a contact, burying the arm in the object up to the
        knuckles instead of pinching it at the tips. "Not colliding" is not the
        same as "gripping".
        """
        return gripper.tip_offset_for(obj.grasp_width)

    def _feasible_approach(self, obj, x, y, z):
        """Choose a grasp pitch and hover that support the whole sequence.

        Returns (phi, pre_grasp, grasp, lift, hover) -- poses as (x, y, z, phi),
        distances in metres. `grasp` is the TCP target, already backed off from
        the contact point by `hover`.

        Starts at the configured pitch and works outwards, because the feasible
        band at floor level is only a few tenths of a radian wide and shifts
        with the object's radius and grip height. Each candidate is checked at
        the standoff, the grasp, the MIDPOINT between them, and for enough
        straight-up travel to clear the floor. The midpoint is not redundant:
        the reachable set is not convex, so two good endpoints do not imply a
        good line between them.

        The grasp is checked with the jaws at jaw_angle_for(grasp_width), NOT at
        the commanded squeeze angle: the object stops the jaws where it is wide,
        so the squeeze angle is a pose the gripper never occupies while holding
        anything, and checking it rejects perfectly good grasps.
        """
        preferred = float(self.get_parameter('grasp_pitch').value)
        lo = float(self.get_parameter('pitch_min').value)
        hi = float(self.get_parameter('pitch_max').value)
        step = float(self.get_parameter('pitch_step').value)
        standoff = float(self.get_parameter('standoff').value)
        want_lift = float(self.get_parameter('lift').value)
        min_lift = float(self.get_parameter('min_lift').value)
        grip = gripper.jaw_angle_for(obj.grasp_width)

        # The whole range, tried nearest the preferred pitch first, so a target
        # that works at 2.6 still gets 2.6 and one that only works at 2.33 gets
        # found instead of being reported unreachable.
        n = int(round((hi - lo) / step))
        candidates = sorted((lo + i * step for i in range(n + 1)),
                            key=lambda p: abs(p - preferred))

        contact = scene_objects.grasp_point(obj, x, y, z)
        hover = self._hover(obj)
        why = {}
        for phi in candidates:
            grasp = scene_objects.back_off(contact, phi, hover) + (phi,)
            pre = scene_objects.back_off(contact, phi, hover + standoff) + (phi,)
            mid = tuple((a + b) / 2.0 for a, b in zip(pre[:3], grasp[:3])) + (phi,)
            bad = next((p for p in (grasp, pre, mid)
                        if kin.ik_best(*p) is None), None)
            if bad is not None:
                why[round(phi, 3)] = kin.describe(*bad)
                continue
            # Check the grasp with the jaws where the OBJECT stops them, not at
            # the commanded squeeze angle -- see the docstring.
            valid, contacts = self.mc.check_state(kin.ik_best(*grasp), grip)
            if not valid:
                why[round(phi, 3)] = 'at the grasp: %s' % contacts
                continue
            lift = self._reachable_lift(grasp, want_lift)
            if lift < min_lift:
                why[round(phi, 3)] = ('only %.0f mm of straight-up lift, need '
                                      '%.0f' % (lift * 1e3, min_lift * 1e3))
                continue
            if phi != preferred:
                self.get_logger().warn(
                    'grasp_pitch %.2f does not work here; using %.2f rad'
                    % (preferred, phi))
            if lift < want_lift:
                self.get_logger().warn(
                    'lift limited to %.0f mm by reach (wanted %.0f)'
                    % (lift * 1e3, want_lift * 1e3))
            self.get_logger().info(
                'TCP held %.1f mm back from the grasp point, so the FINGERTIPS '
                'land on it (gripper.tip_offset_for(%.3f))'
                % (hover * 1e3, obj.grasp_width))
            return phi, pre, grasp, lift, hover

        # Say whether this is a reach problem or a posture problem, because the
        # two want opposite responses: drive closer, versus try another pitch.
        limits = kin.reach_limits()
        span = hypot(hypot(contact[0], contact[1]), contact[2] - kin.Z0)
        bearing = degrees(atan2(contact[1], contact[0]))
        yaw_lo, yaw_hi = (degrees(a) for a in limits['yaw_range'])
        if span > limits['max_reach_from_shoulder']:
            reason = ('the grasp point is %.3f m from the shoulder, past the '
                      '%.3f m the arm can span -- move the base closer'
                      % (span, limits['max_reach_from_shoulder']))
        elif not yaw_lo <= bearing <= yaw_hi:
            reason = ('bearing %.0f deg is outside arm1_Joint\'s %.0f..%.0f deg '
                      'sector -- turn the base' % (bearing, yaw_lo, yaw_hi))
        else:
            reason = ('within reach (%.3f m of %.3f, bearing %.0f deg) but no '
                      'posture works. Nearest attempt, phi=%.2f: %s'
                      % (span, limits['max_reach_from_shoulder'], bearing,
                         preferred, why.get(round(preferred, 3), '?')))
        raise MoveItError(
            'no workable approach to (%.3f, %.3f, %.3f) for %s: %s '
            '(swept phi %.2f..%.2f in %.3f rad steps, %d candidates)'
            % (x, y, z, obj.name, reason, lo, hi, step, len(candidates)))

    # ------------------------------------------------------------------- pick

    def pick(self, x, y, z, name=None, plan_only=False):
        """Pick the object whose CENTRE is at (x, y, z) in base_link."""
        obj = self._object(name)
        phi, pre, grasp, lift, hover = self._feasible_approach(obj, x, y, z)
        standoff = float(self.get_parameter('standoff').value)

        self.get_logger().info(
            'pick %s at (%.3f, %.3f, %.3f): grasp TCP (%.3f, %.3f, %.3f) '
            'phi=%.2f, %.0f mm standoff, %s'
            % (obj.name, x, y, z, grasp[0], grasp[1], grasp[2], phi,
               standoff * 1e3, gripper.describe(obj.grasp_width)))

        if plan_only:
            return self._report_plan(obj, pre, grasp, phi, lift, hover)

        self.mc.move_named('ready')
        self.mc.open_gripper()

        scene_objects.add(self.mc, obj, x, y, z)
        # With the object in the scene, the last few centimetres of the approach
        # are "in collision" by definition -- the jaws have to be around it.
        # Allow just that pair so the floor and the chassis stay checked.
        self.mc.allow_collisions(obj.name, GRIPPER_LINKS, True)

        self.mc.move_pose(*pre)
        self.mc.cartesian_move(grasp)

        self.mc.set_gripper(gripper.grip_angle_for(obj.grasp_width))

        # theta5 is whatever the approach left it at; for a symmetric object it
        # does not matter, and for anything else it is the caller's to set.
        roll = self.mc.current_joints()[4]
        scene_objects.attach(self.mc, obj, phi, roll, hover)
        self.held = obj
        # The attached object's touch_links now cover finger contact, so the
        # manual allowance can go; leaving it would also let the object pass
        # through the fingers after it is put back down.
        self.mc.allow_collisions(obj.name, GRIPPER_LINKS, False)

        self.mc.cartesian_move((grasp[0], grasp[1], grasp[2] + lift, phi))
        self.mc.move_named('carry')
        self.get_logger().info('picked %s' % obj.name)
        return obj

    def _report_plan(self, obj, pre, grasp, phi, lift, hover):
        """Everything pick() would do, checked but not commanded."""
        steps = [('pre-grasp', pre), ('grasp', grasp),
                 ('lift', (grasp[0], grasp[1], grasp[2] + lift, phi))]
        ok = True
        for label, pose in steps:
            joints = kin.ik_best(*pose)
            if joints is None:
                self.get_logger().error(
                    '%-10s (%.3f, %.3f, %.3f) phi=%.2f: %s'
                    % (label, pose[0], pose[1], pose[2], pose[3],
                       kin.describe(*pose)))
                ok = False
                continue
            valid = self.mc.state_valid(joints, verbose=True)
            ok = ok and valid
            self.get_logger().info(
                '%-10s (%.3f, %.3f, %.3f) phi=%.2f -> %s  %s'
                % (label, pose[0], pose[1], pose[2], pose[3],
                   ['%.3f' % q for q in joints],
                   'ok' if valid else 'IN COLLISION'))
        held = scene_objects.held_pose(obj, phi, 0.0, hover)
        self.get_logger().info(
            'held pose in arm5_Link: (%.4f, %.4f, %.4f) quat (%.3f, %.3f, '
            '%.3f, %.3f)' % held)
        return ok

    # ------------------------------------------------------------------ place

    def place(self, state='over_trash'):
        """Release whatever is held over the named drop pose."""
        if self.held is None:
            self.get_logger().warn('place() with nothing held -- releasing anyway')
        self.mc.move_named(state)
        self.mc.open_gripper()
        if self.held is not None:
            self.mc.detach(self.held.name)
            self.mc.remove(self.held.name)
            self.held = None
        self.mc.move_named('carry')
        self.get_logger().info('placed')

    # ---------------------------------------------------------------- utility

    def check_states(self):
        """Validate every NAMED_STATES entry against the live planning scene.

        This is what stops a named pose from being eyeballed in RViz: 'init' was
        wrong for exactly that reason, and it stayed wrong until the chassis
        became a real link.
        """
        worst = True
        for name in sorted(NAMED_STATES):
            joints = NAMED_STATES[name]
            over = [n for n, q, (lo, hi)
                    in zip(kin.JOINT_NAMES, joints, kin.JOINT_LIMITS)
                    if not lo <= q <= hi]
            valid = self.mc.state_valid(joints, verbose=True)
            x, y, z, phi, roll = kin.fk(joints)
            self.get_logger().info(
                '%-11s %-7s tcp=(%.3f, %.3f, %.3f) phi=%.3f%s'
                % (name, 'ok' if valid else 'INVALID', x, y, z, phi,
                   '  OVER LIMIT: %s' % over if over else ''))
            worst = worst and valid and not over
        return worst


def main(args=None):
    parser = argparse.ArgumentParser(
        prog='pick_place', description=__doc__.split('\n\n')[0])
    parser.add_argument('x', type=float, nargs='?',
                        help='object centre x in base_link, metres')
    parser.add_argument('y', type=float, nargs='?')
    parser.add_argument('z', type=float, nargs='?')
    parser.add_argument('--object', default=None,
                        help='catalogue entry (default: the `object` parameter)')
    parser.add_argument('--plan-only', action='store_true',
                        help='solve and collision-check the sequence, move nothing')
    parser.add_argument('--check-states', action='store_true',
                        help='validate every named state and exit')
    parser.add_argument('--no-place', action='store_true',
                        help='pick and carry, but do not go on to place')
    cli = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init(args=args)
    node = PickPlace()
    status = 0
    try:
        if cli.check_states:
            status = 0 if node.check_states() else 1
        elif cli.x is None or cli.y is None or cli.z is None:
            parser.print_usage(sys.stderr)
            node.get_logger().error('give an object position: x y z (metres, '
                                    'base_link, object CENTRE)')
            status = 2
        else:
            ok = node.pick(cli.x, cli.y, cli.z, cli.object,
                           plan_only=cli.plan_only)
            if cli.plan_only:
                status = 0 if ok else 1
            elif not cli.no_place:
                node.place()
    except (MoveItError, graspable.ObjectError, gripper.GripperError) as exc:
        node.get_logger().error(str(exc))
        status = 1
    except KeyboardInterrupt:
        status = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == '__main__':
    sys.exit(main())
