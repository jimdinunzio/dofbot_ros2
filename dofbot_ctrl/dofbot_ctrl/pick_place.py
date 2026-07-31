#!/usr/bin/env python3
# coding: utf-8
"""
Pick and place, driven by coordinates rather than by RViz.

    ros2 run dofbot_ctrl pick_place -- 0.22 0.0 0.0
    ros2 run dofbot_ctrl pick_place -- --object soda_can 0.22 0.0 0.046
    ros2 run dofbot_ctrl pick_place -- --plan-only 0.22 0.0 0.0
    ros2 run dofbot_ctrl pick_place -- --check-states

`pick(x, y, z)` is the whole point of this file, and the seam the perception
layer will attach to. (x, y, z) is the CENTRE of the object in base_link. A
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
Gripping_point_Link is a fixed frame 68.09 mm from the wrist, but the jaw faces
are mesh geometry that reaches past it, by an amount that CHANGES with the
opening: the four-bar swings the fingers outward along the tool axis as it
closes, so Rlink2_Link's origin travels from z = 0.398 wide open to z = 0.429
fully shut while the TCP frame stays at 0.437.

So the arm does not drive the TCP onto the object -- it stops short and lets the
closing motion reach down and in. `_hover` finds how far short, by backing the
TCP off along the tool axis until the state is clear with the jaws at the angle
the object stops them at. At x = 0.22, phi = 2.6, a 30 mm block on the floor
wants about 20 mm of hover.

That distance is exactly gripper.tip_offset_for(), which is 0.0 because it has
never been measured with calipers. Nothing depends on that zero: the search
finds the value against the live scene and logs it, so calibrating later turns a
search into a check. Note it is also the reason a squeeze angle must not be
collision-checked -- the object holds the jaws open at its own width, so the
commanded angle past contact is a pose the gripper never occupies.
"""

import argparse
import sys

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

        self.declare_parameter('object', 'test_block')
        self.declare_parameter('standoff', 0.08)      # pre-grasp, m along tool
        self.declare_parameter('grasp_pitch', 2.6)    # phi, rad from vertical
        self.declare_parameter('pitch_search', 0.6)   # how far to sweep, rad
        self.declare_parameter('pitch_step', 0.05)
        self.declare_parameter('lift', 0.10)          # straight-up retreat, m
        self.declare_parameter('min_lift', 0.03)      # enough to clear the floor
        self.declare_parameter('max_hover', 0.05)     # tip-offset search limit, m

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
        reachable band in z is only a few centimetres deep -- from a 30 mm block
        at x = 0.22 with phi = 2.6, straight up runs out at 95 mm -- so a fixed
        100 mm retreat is unreachable about as often as not. Taking what is
        available and reporting it beats failing the whole pick over the last
        5 mm of a move whose only job is to clear the floor.
        """
        best = 0.0
        d = step
        while d <= want + 1e-9:
            if kin.ik_best(grasp[0], grasp[1], grasp[2] + d, grasp[3]) is None:
                break
            best = d
            d += step
        return best

    def _hover(self, contact, phi, grip, limit, step=0.002):
        """How far back along the tool axis the TCP must sit, at this pitch.

        Gripping_point_Link is a fixed frame 68.09 mm out, but the jaw faces are
        mesh geometry that reaches PAST it, and by an amount that changes with
        the opening -- the four-bar swings the fingers outward along the tool
        axis as it closes. Putting the TCP straight onto the contact point
        therefore drives the fingertips through whatever is behind it, which for
        an object on the floor is floor_link.

        So back the TCP off until the state is clear with the jaws at the angle
        the object actually stops them at, and return the distance. This is the
        "hover a little higher and let the closing motion reach down" the
        gripper is designed for, measured rather than guessed -- and the number
        it returns is what gripper.tip_offset_for() should eventually hold.

        Returns None if no hover up to `limit` works.
        """
        d = 0.0
        while d <= limit + 1e-9:
            tcp = scene_objects.back_off(contact, phi, d) + (phi,)
            joints = kin.ik_best(*tcp)
            if joints is not None and self.mc.check_state(joints, grip)[0]:
                return d
            d += step
        return None

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
        span = float(self.get_parameter('pitch_search').value)
        step = float(self.get_parameter('pitch_step').value)
        standoff = float(self.get_parameter('standoff').value)
        want_lift = float(self.get_parameter('lift').value)
        min_lift = float(self.get_parameter('min_lift').value)
        max_hover = float(self.get_parameter('max_hover').value)
        grip = gripper.jaw_angle_for(obj.grasp_width)

        candidates = [preferred]
        k = 1
        while k * step <= span:
            candidates += [preferred - k * step, preferred + k * step]
            k += 1

        contact = scene_objects.grasp_point(obj, x, y, z)
        why = {}
        for phi in candidates:
            hover = self._hover(contact, phi, grip, max_hover)
            if hover is None:
                tcp = contact + (phi,)
                why[round(phi, 3)] = (
                    kin.describe(*tcp) if kin.ik_best(*tcp) is None else
                    'no hover up to %.0f mm clears: %s'
                    % (max_hover * 1e3,
                       self.mc.check_state(kin.ik_best(*tcp), grip)[1]))
                continue
            grasp = scene_objects.back_off(contact, phi, hover) + (phi,)
            pre = scene_objects.back_off(contact, phi, hover + standoff) + (phi,)
            mid = tuple((a + b) / 2.0 for a, b in zip(pre[:3], grasp[:3])) + (phi,)
            bad = next((p for p in (pre, mid) if kin.ik_best(*p) is None), None)
            if bad is not None:
                why[round(phi, 3)] = kin.describe(*bad)
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
                'hover %.0f mm back from the contact point (gripper.'
                'tip_offset_for(%.3f) is %.0f mm; calibrate it and this search '
                'becomes a check)'
                % (hover * 1e3, obj.grasp_width,
                   gripper.tip_offset_for(obj.grasp_width) * 1e3))
            return phi, pre, grasp, lift, hover

        raise MoveItError(
            'no workable approach to (%.3f, %.3f, %.3f) for %s over pitches '
            '%.2f..%.2f. At the preferred %.2f: %s'
            % (x, y, z, obj.name, min(why), max(why), preferred,
               why[round(preferred, 3)]))

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
            over = [n for n, q in zip(kin.JOINT_NAMES, joints)
                    if abs(q) > kin.JOINT_LIMIT]
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
