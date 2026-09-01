#!/usr/bin/env python3
# coding: utf-8
"""
Pick and place, driven by coordinates rather than by RViz.

    # a soda can standing on the floor: (x, y, z) is its CENTRE, so z is half
    # the catalogue height. The extensions want it further out than the stock
    # jaws did -- see the reach note below. --pick ends holding it, at 'carry'
    ros2 run dofbot_ctrl pick_place -- --pick 0.30 0.0 0.061
    # then, whenever the caller has decided where it goes
    ros2 run dofbot_ctrl pick_place -- --place
    # solve and collision-check the pick without moving. Leaves the can standing
    # in the planning scene and drawn in RViz, so this is also how you park a
    # target to look at it.
    ros2 run dofbot_ctrl pick_place -- --plan-only 0.30 0.0 0.061
    # the test block needs the stock jaws; the extensions cannot close on it
    DOFBOT_GRIPPER=stock ros2 run dofbot_ctrl pick_place -- \
        --pick --object test_block 0.22 0.0 0.015
    ros2 run dofbot_ctrl pick_place -- --check-states
    # recover after a run that died partway: empty the scene, let go, go home
    ros2 run dofbot_ctrl pick_place -- --reset

ONE HALF PER INVOCATION. There is no combined form: --pick ends with the object
held at 'carry' and the process gone, and --place picks that up from the planning
scene whenever the caller is ready. The pause between them is the point -- it is
where something else decides where the object goes.

`pick(x, y, z)` is the whole point of this file, and the seam the perception
layer will attach to. (x, y, z) is the CENTRE of the object in base_link -- so
for something resting on the floor, z is somewhere within thecatalogue height, NOT 0. A
depth camera returns a point on the near surface, so whoever calls this steps
along the view ray by the object radius first -- deliberately, once, in the
caller.

SEQUENCE
--------
    add the object to the planning scene, and draw its mesh
    move_named('ready') + open_gripper
    move_pose  to a pre-grasp standoff back along the tool axis  (OMPL plans it)
               -- as long a standoff as the arm has room for, measured
    cartesian_move straight in to the grasp                      (we plan it)
    close_gripper to the object's grasp width
    attach, with every end-effector link named as a touch link
    cartesian_move straight up, as far as the arm has room for (may be nothing)
    move_named('carry')       -- THIS is what clears the floor; see min_lift

then place(): over_trash -> open -> detach + remove -> carry. It reads what is
attached from the planning scene, which is what lets the halves be two separate
`ros2 run` invocations (--pick, then --place) with no process in common.

WHAT _feasible_approach ACTUALLY CHOOSES
----------------------------------------
Three things, not one:

    phi           tool tilt from vertical (see dofbot_kinematics): 0 straight
                  up, pi straight down. Swept pitch_min..pitch_max
    grasp_height  where up the object the jaws close, within the band the
                  catalogue entry allows. Most objects offer no band at all
    standoff      how long the final straight-line approach is -- MEASURED,
                  not demanded

Candidates are RANKED, on how much room the tightest joint has left and on the
standoff they support, rather than taken first-fit. Near the inner edge of the
working ring that distinction is the whole game: solutions there exist but sit
hard against a joint stop, with nothing left to absorb the error in where the
object really is. grasp_pitch is a tie-break among comfortable postures, not a
starting point.

For a can on the floor the working band is phi 2.05..2.35, which is why the
default is 2.2.

HOVER: THE TCP IS NOT WHERE THE JAWS GRIP
----------------------------------------
The object is held BETWEEN THE FINGERS, and the fingers are nowhere near
Gripping_point_Link. That frame is fixed to the wrist, while they reach PAST it
and by a varying amount, because the four-bar swings them outward along the tool
axis as it closes.

So the TCP target is the grasp point pulled back along the tool axis by
gripper.throat_offset_for(width). Get this wrong in the optimistic direction and
the arm drives itself into the object up to the knuckles -- which is exactly
what it looks like in RViz.

HOW FAR DOWN THE FINGER, not just how far back. The finger has 40 mm of flat
front face and then a back stop, and where on that face the object lands is a
separate question from how far the TCP is held off. gripper.throat_offset_for()
answers it, and HALF THE OBJECT IS BEHIND THE CONTACT LINE is the term to keep
hold of: the faces touch a round object at its widest point, so a can aimed at
the fingertip already fills 33 of the 40 mm and only 7 mm is going spare. 

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

    stock jaws                       object x = 0.20 .. 0.28 m   hover 11.7 mm
    extended fingers                 object x = 0.26 .. 0.35 m   hover 88.2 mm

Seating an object deeper SPENDS hover, so it walks the band back in by whatever
it spends -- the same mechanism as the gripper swap, just driven by where on the
finger the object sits rather than by how long the finger is. The can spends
NOTHING at the current gripper.BACK_STOP_CLEARANCE (its advance clamps to zero,
see gripper.throat_offset_for), so the second row is both the fingertip figure
and the live one. 

THE HOVER IS NOT WHAT SETS THE NEAR EDGE, though. Both rows above are swept with
the standoff held at a fixed 80 mm, and that is what binds, not the reach: the
GRASP alone solves from x = 0.164 m. The standoff pose is the grasp pulled back
along the tool axis, and at phi ~ 2.2 that direction is up and INWARD, into the
same fold-up limit, so it runs out first and takes 6 cm of near reach with it.
_reachable_standoff measures it instead, which puts the extended-finger band at
0.20 .. 0.38 m with the near edge set by the min_standoff floor.

The near edge is still the one that bites: a can comfortably reachable on the
stock jaws can be unreachable at ANY pitch with the extensions on. When a grasp
will not solve, the fix is usually to move the base FURTHER AWAY, which is the
opposite of the instinct. _feasible_approach names the stage that ran out, so
the log tells you which edge you are against.

grasp_height barely moves the band -- a few millimetres across the whole usable
range -- but it does move the POSTURE, and near the inner edge that is the
difference between a solution with joint room to spare and one against a stop.
Hence graspable.grasp_heights: reachability is not what the height is being
traded for. (Whether a tall can fouls the wrist once attached is a collision
question, and only move_group answers it.)

Related, and the same mistake in a different place: never collision-check the
COMMANDED jaw angle. obj.grip_angle() deliberately asks for narrower than the
object so the servo loads up against it, but the object stops the jaws at its
own width, so the commanded angle is a pose the gripper never occupies while
holding anything. Check jaw_angle_for(width).
"""

import argparse
import sys
from dataclasses import replace
from math import atan2, degrees, hypot, pi

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from dofbot_ctrl import dofbot_kinematics as kin
from dofbot_ctrl import graspable, gripper, scene_markers, scene_objects
from dofbot_ctrl.moveit_client import (GRIPPER_LINKS, NAMED_STATES,
                                       DofbotMoveIt, MoveItError)

# How an approach is scored, and both halves are "enough is enough". Past
# MARGIN_ENOUGH the posture has joint room to spare; past STANDOFF_ENOUGH the
# straight-line approach is as long as it needs to be. Beyond either, more is
# not better, and the tie-breaks hand the choice back to the grip height and
# pitch that are known to work on hardware.
#
# QUANTISED, because both are measured against a machine with backlash. The
# steps are the smallest difference in each quantity that means anything: below
# them, a raw comparison lets sixth-of-a-degree noise outvote a real preference.
MARGIN_ENOUGH, MARGIN_STEP = 0.10, 0.02        # rad clear of the nearest stop
STANDOFF_ENOUGH, STANDOFF_STEP = 0.040, 0.010  # m of straight-line approach

# A cap on /check_state_validity round trips per pick. The IK screen is free and
# leaves hundreds of survivors ranked by score; the collision check is a service
# call. Ten is generous -- a correctly cleared scene passes on the first -- and
# the cap is what keeps a scene left full of stale objects from hammering
# move_group through the whole ranking.
MAX_STATE_CHECKS = 10

# Scene objects reset() must leave alone: they are standing fixtures owned by
# another node, not leftovers. Must match chassis_collision's `object_id`.
SCENE_FIXTURES = ('robot_chassis',)


def _joint_margin(joints):
    """How many radians the TIGHTEST joint has left before its own limit.

    The arm reaches its inner working edge by folding up, so near that edge
    every solution sits against a stop.

    Read from kin.JOINT_LIMITS per joint, never a symmetric constant: arm1's
    range is both wider and asymmetric.
    """
    return min(min(q - lo, hi - q)
               for q, (lo, hi) in zip(joints, kin.JOINT_LIMITS))


class PickPlace(Node):

    def __init__(self):
        super().__init__('pick_place')

        self.declare_parameter('object', 'soda_can')
        # The standoff WANTED; _reachable_standoff takes as much of it as the
        # arm has room for.
        self.declare_parameter('standoff', 0.08)      # pre-grasp, m along tool
        # ...and the shortest approach worth making. The last leg is a straight
        # line along the tool axis so the open jaws SLIDE OVER the object rather
        # than swinging into it, and an approach no longer than the error in the
        # object's position does not do that. Lowering this floor buys near
        # reach and spends the only thing that stops the jaws knocking the can
        # over: 10 mm reaches x = 0.19, 20 mm reaches x = 0.20.
        self.declare_parameter('min_standoff', 0.02)
        self.declare_parameter('standoff_step', 0.005)
        # phi, rad from vertical. A tie-break among comfortable postures, not a
        # starting point. 2.2 is the middle of where the can actually gets
        # picked: swept across 0.20..0.35 m the working band is 2.05..2.32 and
        # nothing solves above about 2.34.
        self.declare_parameter('grasp_pitch', 2.2)
        # Sweep the whole range that can reach DOWN onto a supported object:
        # pi/2 is a horizontal tool, pi is straight down. Anything shallower
        # points the tool upwards and cannot grasp something standing on a
        # surface.
        self.declare_parameter('pitch_min', pi / 2.0)
        self.declare_parameter('pitch_max', pi)
        # Must be fine enough not to step OVER the feasible band. At a
        # floor-level grasp that band can be a couple of hundredths of a radian
        # wide, and a coarse step straddles it and reports "no workable
        # approach" for a pose the arm reaches comfortably. Stepping finer is
        # nearly free: candidates are screened by analytic IK first, and only
        # survivors cost a /check_state_validity round trip.
        self.declare_parameter('pitch_step', 0.01)
        # How finely the grip height is traded. See graspable.grasp_heights --
        # most objects offer no range at all and this does nothing.
        self.declare_parameter('grasp_height_step', 0.005)
        self.declare_parameter('lift', 0.10)          # straight-up retreat, m
        # ADVISORY, NOT A GATE -- below this the pick proceeds and says so.
        # The lift is not what gets the object clear of the floor:
        # move_named('carry') raises it MONOTONICALLY from the grasp, measured
        # at x = 0.19, 0.20 and 0.24 as +27 mm of base height in the first tenth
        # of that move with no dip. The Cartesian lift is only the safest FIRST
        # few centimetres, worth taking when available.
        self.declare_parameter('min_lift', 0.03)
        # How much room the tightest joint should have left, radians. Advisory,
        # but the number to watch: the arm reaches both edges of its working
        # ring by running out of joint, so near either edge every solution sits
        # against a stop. 0.02 rad is about a degree, roughly one servo step.
        # Under it the pose is geometrically valid and practically unusable --
        # there is nothing left
        # to absorb the error in where the object actually is.
        self.declare_parameter('min_margin', 0.02)

        self.mc = DofbotMoveIt(self)
        # Cosmetic only. Every call below sits next to the scene_objects call it
        # mirrors, so the drawn can and the planned-against cylinder are put in
        # the same place by the same lines.
        self.markers = scene_markers.MeshMarkers(self)
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

    def _reachable_standoff(self, contact, grasp, phi, hover, want, floor,
                            step=0.005):
        """The longest pre-grasp standoff that solves, at most `want`.

        Returns (standoff, pre_grasp, margin), or None if not even `floor`
        fits. `margin` is the tightest joint margin across the pre-grasp pose
        and the midpoint between it and the grasp -- the midpoint is not
        redundant, because the reachable set is not convex and two good
        endpoints do not imply a good line between them.

        Measured, not assumed -- the same shape as _reachable_lift above, for
        the same reason. It is not a collision search: see the warning on
        gripper.throat_offset_for. What it measures is where the arm runs out of
        joint, which is a fact about the arm and does not move when the scene
        does.
        """
        d = want
        while d >= floor - 1e-9:
            pre = scene_objects.back_off(contact, phi, hover + d) + (phi,)
            mid = tuple((a + b) / 2.0
                        for a, b in zip(pre[:3], grasp[:3])) + (phi,)
            j_pre, j_mid = kin.ik_best(*pre), kin.ik_best(*mid)
            if j_pre is not None and j_mid is not None:
                return d, pre, min(_joint_margin(j_pre), _joint_margin(j_mid))
            d -= step
        return None

    def _hover(self, obj):
        """How far back along the tool axis the TCP sits from the grasp point.

        The object is held BETWEEN THE FINGERS, and the fingers are not at
        Gripping_point_Link. That frame is fixed to the wrist while they reach
        past it, by more as they close, because the four-bar swings them outward
        along the tool axis. So the arm must stop the TCP short of the object by
        exactly that much.

        WHERE ON THE FINGER is the other half, and it is not free. This used to
        be gripper.tip_offset_for(), which lands the CONTACT LINE on the
        fingertip; since the faces grip a round object at its widest point, half
        the object then hangs behind the tip and the rest of the finger goes
        unused. gripper.throat_offset_for() seats it against the back stop
        instead, working the advance out from the object's own radius. It is not
        what made the can pick work -- that was grasp_height -- and for the can
        it currently returns tip_offset_for unchanged.

        This is a lookup, not a search. Do not replace it with a search for
        the smallest collision-free backoff: that finds the DEEPEST reach which
        does not trip a contact, which is a different quantity that merely looks
        similar -- it moves whenever the scene does, and "not colliding" is not
        the same as "gripping".
        """
        return gripper.throat_offset_for(obj.grasp_width)

    def _feasible_approach(self, obj, x, y, z):
        """Choose a grip height, grasp pitch and standoff for the whole sequence.

        Returns (phi, pre_grasp, grasp, lift, hover, grasp_height) -- poses as
        (x, y, z, phi), distances in metres. `grasp` is the TCP target, already
        backed off from the contact point by `hover`. The caller must carry
        `grasp_height` forward, because it is not necessarily the object's
        nominal one and everything downstream measures from it.

        THREE THINGS ARE BEING CHOSEN, not one, and only the first is obvious:

          phi           the tool tilt, swept pitch_min..pitch_max
          grasp_height  where up the object the jaws close, from
                        obj.grasp_heights() -- usually a single value
          standoff      how long the final straight-line approach is, measured
                        by _reachable_standoff rather than demanded

        RANKED, NOT FIRST-MATCH, because near the inner edge of the working
        ring "solvable" and "workable" come apart: solutions there sit hard
        against a joint stop, with no room to approach along a line, no room to
        lift, and nothing left to absorb the error in where the object actually
        is. Candidates are scored on _joint_margin and on the standoff they
        support, both quantised (see MARGIN_ENOUGH), so once a posture is
        comfortable the tie-breaks return the choice to the preferred grip
        height and pitch.

        The IK screen is free, so every candidate gets one. Only the ranked
        survivors cost a /check_state_validity round trip, and only
        MAX_STATE_CHECKS of them.

        The grasp is checked with the jaws at jaw_angle_for(grasp_width), NOT at
        the commanded squeeze angle: the object stops the jaws where it is wide,
        so the squeeze angle is a pose the gripper never occupies while holding
        anything, and checking it rejects perfectly good grasps.

        The lift is measured and reported but NOT required -- see the min_lift
        parameter for why the move to 'carry' is what clears the floor.
        """
        preferred = float(self.get_parameter('grasp_pitch').value)
        lo = float(self.get_parameter('pitch_min').value)
        hi = float(self.get_parameter('pitch_max').value)
        step = float(self.get_parameter('pitch_step').value)
        standoff = float(self.get_parameter('standoff').value)
        min_standoff = float(self.get_parameter('min_standoff').value)
        standoff_step = float(self.get_parameter('standoff_step').value)
        height_step = float(self.get_parameter('grasp_height_step').value)
        want_lift = float(self.get_parameter('lift').value)
        min_lift = float(self.get_parameter('min_lift').value)
        min_margin = float(self.get_parameter('min_margin').value)
        grip = gripper.jaw_angle_for(obj.grasp_width)
        hover = self._hover(obj)

        n = int(round((hi - lo) / step))
        pitches = [lo + i * step for i in range(n + 1)]
        heights = obj.grasp_heights(height_step)

        scored = []
        solved_grasp = []       # (height, phi) that got that far, for diagnosis
        for height in heights:
            at_height = replace(obj, grasp_height=height)
            contact = scene_objects.grasp_point(at_height, x, y, z)
            for phi in pitches:
                grasp = scene_objects.back_off(contact, phi, hover) + (phi,)
                j_grasp = kin.ik_best(*grasp)
                if j_grasp is None:
                    continue
                solved_grasp.append((height, phi, contact, grasp))
                got = self._reachable_standoff(contact, grasp, phi, hover,
                                               standoff, min_standoff,
                                               standoff_step)
                if got is None:
                    continue
                reach, pre, margin = got
                margin = min(margin, _joint_margin(j_grasp))
                lift = self._reachable_lift(grasp, want_lift)
                # The epsilon is not decoration: a margin landing exactly on
                # MARGIN_ENOUGH divides to 4.9999999 and would bucket one step
                # low, handing the choice to a candidate no better than it.
                # Boundary values belong in the bucket above.
                key = (int((min(margin, MARGIN_ENOUGH) + 1e-9) / MARGIN_STEP),
                       int((min(reach, STANDOFF_ENOUGH) + 1e-9) / STANDOFF_STEP),
                       -round(abs(height - obj.grasp_height), 6),
                       -round(abs(phi - preferred), 6))
                scored.append((key, height, phi, pre, grasp, lift, reach,
                               margin, j_grasp))
        scored.sort(key=lambda c: c[0], reverse=True)

        why = []
        for cand in scored[:MAX_STATE_CHECKS]:
            _, height, phi, pre, grasp, lift, reach, margin, j_grasp = cand
            valid, contacts = self.mc.check_state(j_grasp, grip)
            if not valid:
                why.append('phi=%.2f grip height %.0f mm: %s'
                           % (phi, height * 1e3, contacts))
                continue
            if height != obj.grasp_height:
                # Says "scored better", not "does not work": the preferred
                # height is a tie-break, so it may well have solved and simply
                # come second. Worth a warning either way, because the height is
                # the one number here that was proven on hardware.
                self.get_logger().warn(
                    'gripping %.0f mm up rather than the usual %.0f: it scored '
                    'better here, and %s allows %.0f..%.0f'
                    % (height * 1e3, obj.grasp_height * 1e3, obj.name,
                       obj.grasp_height_range[0] * 1e3,
                       obj.grasp_height_range[1] * 1e3))
            # One line with every number the choice was made on. phi and the
            # standoff are NOT warned about when they come in short of what was
            # asked for -- the pitch is a tie-break and the standoff is measured
            # by design, so a warning on either fires on nearly every pick and
            # means nothing. The warnings below are the ones to act on.
            self.get_logger().info(
                'approach: phi=%.2f (asked %.2f), grip %.0f mm up the object, '
                '%.0f mm straight-line approach (wanted %.0f), %.0f mm lift, '
                'tightest joint %.3f rad off its stop'
                % (phi, preferred, height * 1e3, reach * 1e3, standoff * 1e3,
                   lift * 1e3, margin))
            if reach <= min_standoff + 1e-9:
                self.get_logger().warn(
                    'the straight-line approach is down to its %.0f mm floor. '
                    'The jaws barely slide over the object before closing, so '
                    'an error in where it actually is has nothing to absorb it '
                    '-- this target is at the arm\'s inner edge'
                    % (min_standoff * 1e3))
            if margin < min_margin:
                self.get_logger().warn(
                    'this posture is %.3f rad off a joint stop, under the '
                    '%.3f rad worth having. It solves, but there is no room '
                    'left for pose error and the servo may already be against '
                    'its limit' % (margin, min_margin))
            if lift < min_lift:
                self.get_logger().warn(
                    'only %.0f mm of straight-up lift here (wanted %.0f). Not '
                    'fatal -- the move to carry raises the object on its own -- '
                    'but the first centimetres are not the controlled ones'
                    % (lift * 1e3, want_lift * 1e3))
            # Half the object is behind the contact line, so how far it reaches
            # into the finger is that half PLUS whatever the advance was. Worth
            # printing both: the advance is what changed, the reach is what has
            # to fit inside gripper.FINGER_DEPTH.
            advance = gripper.tip_offset_for(obj.grasp_width) - hover
            self.get_logger().info(
                'TCP held %.1f mm back from the grasp point: the fingertip '
                'lookup driven %.1f mm deeper, so %s reaches %.1f mm into a '
                '%s mm finger (gripper.throat_offset_for(%.3f))'
                % (hover * 1e3, advance * 1e3, obj.name,
                   (obj.grasp_width / 2.0 + advance) * 1e3,
                   '%.0f' % (gripper.FINGER_DEPTH * 1e3)
                   if gripper.FINGER_DEPTH is not None else 'an unmeasured',
                   obj.grasp_width))
            return phi, pre, grasp, lift, hover, height

        raise MoveItError(
            'no workable approach to (%.3f, %.3f, %.3f) for %s: %s (swept phi '
            '%.2f..%.2f in %.3f rad steps and grip height %s, %d candidates)'
            % (x, y, z, obj.name,
               self._why_not(obj, x, y, z, solved_grasp, scored, why,
                             hover, min_standoff, standoff_step, preferred),
               lo, hi, step,
               '%.0f..%.0f mm' % (min(heights) * 1e3, max(heights) * 1e3)
               if len(heights) > 1 else '%.0f mm' % (heights[0] * 1e3),
               len(pitches) * len(heights)))

    def _why_not(self, obj, x, y, z, solved_grasp, scored, why, hover,
                 min_standoff, standoff_step, preferred):
        """Name the stage that actually failed, in the order the sweep hit them."""
        limits = kin.reach_limits()
        contact = scene_objects.grasp_point(obj, x, y, z)
        span = hypot(hypot(contact[0], contact[1]), contact[2] - kin.Z0)
        bearing = degrees(atan2(contact[1], contact[0]))
        yaw_lo, yaw_hi = (degrees(a) for a in limits['yaw_range'])

        if not solved_grasp:
            # Nothing struck the grasp at all: a reach or a bearing problem,
            # and those want opposite responses -- drive closer, versus turn.
            if span > limits['max_reach_from_shoulder']:
                return ('the grasp point is %.3f m from the shoulder, past the '
                        '%.3f m the arm can span -- move the base closer'
                        % (span, limits['max_reach_from_shoulder']))
            if not yaw_lo <= bearing <= yaw_hi:
                return ('bearing %.0f deg is outside arm1_Joint\'s %.0f..%.0f '
                        'deg sector -- turn the base' % (bearing, yaw_lo, yaw_hi))
            return ('within reach (%.3f m of %.3f, bearing %.0f deg) but no '
                    'pitch or grip height strikes the grasp at all: %s'
                    % (span, limits['max_reach_from_shoulder'], bearing,
                       kin.describe(*(scene_objects.back_off(
                           contact, preferred, hover) + (preferred,)))))

        if not scored:
            # THE NEAR-EDGE CASE. The grasp solves; what does not is backing out
            # of it along the tool axis to make a straight-line approach,
            # because at these pitches that direction is up and inward, into the
            # fold-up limit. Measure the approach again with the floor removed,
            # so the report says how far short it fell and not merely that it
            # did.
            phis = [p for _h, p, _c, _g in solved_grasp]
            hs = [h for h, _p, _c, _g in solved_grasp]
            best = 0.0
            for _h, phi, cont, grasp in solved_grasp:
                got = self._reachable_standoff(cont, grasp, phi, hover,
                                               min_standoff, standoff_step,
                                               standoff_step)
                if got is not None:
                    best = max(best, got[0])
            return ('the grasp solves (phi %.2f..%.2f, grip height '
                    '%.0f..%.0f mm) but there is no room to back out of it for '
                    'a straight-line approach: the longest that clears the '
                    'joint limits is %.0f mm against the %.0f mm minimum. This '
                    'is the arm folded up tight, so the target is too CLOSE -- '
                    'move the base FURTHER AWAY.'
                    % (min(phis), max(phis), min(hs) * 1e3, max(hs) * 1e3,
                       best * 1e3, min_standoff * 1e3))

        return ('%d approaches solve kinematically but the best %d all collide: '
                '%s' % (len(scored), len(why), '; '.join(why) or 'no detail'))

    # ------------------------------------------------------------------- pick

    def _clear(self, obj):
        """Drop any copy of `obj` an earlier run left behind.

        MUST happen before _feasible_approach. The sweep collision-checks each
        candidate grasp against the LIVE scene, and a grasp puts the jaws around
        the object, so a stale copy of the very thing being picked rejects every
        candidate -- reported as "no workable approach ... but no posture works"
        for a target the arm reaches comfortably. The object is re-added after
        the sweep, together with the allowance that makes the grasp legal.

        Leftovers are the normal case, not an edge: --plan-only deliberately
        leaves the object standing so it can be looked at, --pick ends with
        it attached to the gripper, and any run that fails partway leaves it
        wherever it got to. Each `ros2 run` is a fresh process with no memory of
        the last one, so the scene is the only place that state lives.

        The scene is READ before anything is deleted. detach() and remove() are
        not no-ops on an object that is absent -- move_group returns
        success=False and the client raises -- so deleting blind fails on every
        clean run, which is most of them.

        Detach before remove: removing a WORLD object does not touch an
        ATTACHED one, and after --pick the can is attached. Detaching puts
        it back into the world, where remove() then takes it.
        """
        world, attached = self.mc.object_ids()
        if obj.name not in world and obj.name not in attached:
            self.markers.hide(obj)          # a drawing with nothing behind it
            return

        self.get_logger().info('clearing %r left in the scene by an earlier run'
                               % obj.name)
        if obj.name in attached:
            self.mc.detach(obj.name)
        self.mc.remove(obj.name)
        # A run that died between allowing and forbidding leaves the pair
        # allowed, which fails the other way -- a grasp validated against an
        # object the gripper is permitted to pass straight through. Only
        # reachable when something WAS there, which is exactly that case.
        self.mc.allow_collisions(obj.name, GRIPPER_LINKS, False)
        self.markers.hide(obj)

    def pick(self, x, y, z, name=None, plan_only=False):
        """Pick the object whose CENTRE is at (x, y, z) in base_link."""
        obj = self._object(name)
        self._clear(obj)
        phi, pre, grasp, lift, hover, height = self._feasible_approach(
            obj, x, y, z)

        # The sweep is allowed to move the grip height within the object's own
        # band, and EVERYTHING DOWNSTREAM MEASURES FROM IT: the collision
        # cylinder's placement, held_pose's centre offset, the drawn marker.
        # Substituting it into the catalogue entry here means each of those
        # keeps reading obj.grasp_height / obj.centre_offset() and cannot be
        # left behind holding the nominal figure. Frozen dataclass, so replace()
        # re-runs the validation rather than mutating past it.
        obj = replace(obj, grasp_height=height)

        self.get_logger().info(
            'pick %s at (%.3f, %.3f, %.3f): grasp TCP (%.3f, %.3f, %.3f) '
            'phi=%.2f, gripping %.0f mm up the object, %s'
            % (obj.name, x, y, z, grasp[0], grasp[1], grasp[2], phi,
               height * 1e3, gripper.describe(obj.grasp_width)))

        # Into the scene AFTER the approach search but BEFORE the plan_only
        # branch, and both halves of that matter.
        #
        # Not earlier: _feasible_approach would then reject every pitch, because
        # a grasp puts the jaws around the object and that is a collision until
        # it is allowed.
        #
        # Not later: --plan-only used to return above this line, so it drew no
        # can and checked its poses against a world that did not contain one.
        # That made the mode useless for the thing it is most wanted for --
        # parking the target and looking at where it actually sits relative to
        # the jaws -- and it also under-reported, since a link that fouls the
        # can is exactly the sort of thing a dry run should catch.
        scene_objects.add(self.mc, obj, x, y, z)
        self.markers.show(obj, x, y, z)

        if plan_only:
            # The same allowance the real sequence uses, so the dry run answers
            # the same question rather than flagging the grasp itself.
            self.mc.allow_collisions(obj.name, GRIPPER_LINKS, True)
            try:
                return self._report_plan(obj, pre, grasp, phi, lift, hover)
            finally:
                # The can stays in the scene to be looked at, but the ACM goes
                # back: nothing is holding it, so it should obstruct the gripper
                # like any other object until a real pick allows it.
                self.mc.allow_collisions(obj.name, GRIPPER_LINKS, False)

        # The can is already in the scene for these two, and deliberately still
        # forbidden -- the transit to 'ready' has no business passing through it.
        self.mc.move_named('ready')
        self.mc.open_gripper()

        # With the object in the scene, the last few centimetres of the approach
        # are "in collision" by definition -- the jaws have to be around it.
        # Allow just that pair so the floor and the chassis stay checked.
        self.mc.allow_collisions(obj.name, GRIPPER_LINKS, True)

        self.mc.move_pose(*pre)
        self.mc.cartesian_move(grasp)

        self.mc.set_gripper(obj.grip_angle())

        # theta5 is whatever the approach left it at; for a symmetric object it
        # does not matter, and for anything else it is the caller's to set.
        roll = self.mc.current_joints()[4]
        scene_objects.attach(self.mc, obj, phi, roll, hover)
        self.markers.hold(obj, phi, roll, hover)
        self.held = obj
        # The attached object's touch_links now cover finger contact, so the
        # manual allowance can go; leaving it would also let the object pass
        # through the fingers after it is put back down.
        self.mc.allow_collisions(obj.name, GRIPPER_LINKS, False)

        # The lift is whatever the arm had room for, and at the edges of the
        # working ring that can be nothing at all. Skip rather than command a
        # zero-length straight line: plan_cartesian floors itself at two
        # waypoints, so a zero move is a degenerate trajectory to the pose the
        # arm is already in. move_named('carry') below is what clears the floor
        # either way -- it raises the object monotonically from the grasp.
        if lift > 0.0:
            self.mc.cartesian_move((grasp[0], grasp[1], grasp[2] + lift, phi))
        else:
            self.get_logger().warn(
                'no straight-up lift available from this grasp; going straight '
                'to carry, which raises the object on its own')
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
                '%-10s (%.3f, %.3f, %.3f) phi=%.2f -> %s  %s  (%.3f rad off '
                'the nearest joint stop)'
                % (label, pose[0], pose[1], pose[2], pose[3],
                   ['%.3f' % q for q in joints],
                   'ok' if valid else 'IN COLLISION', _joint_margin(joints)))
        held = scene_objects.held_pose(obj, phi, 0.0, hover)
        self.get_logger().info(
            'held pose in arm5_Link: (%.4f, %.4f, %.4f) quat (%.3f, %.3f, '
            '%.3f, %.3f)' % held)
        return ok

    # ------------------------------------------------------------------ place

    def place(self, state='over_trash'):
        """Release whatever is held over the named drop pose.

        What is being carried is read from the PLANNING SCENE when this process
        did not do the picking -- which is the whole of `--place`. self.held
        only ever records what this process attached, and a --pick run ends
        in a different process from the one that finishes the job; the scene is
        where that state actually lives. Detaching an object nobody remembers is
        the difference between placing it and leaving it welded to the gripper
        for every plan that follows.
        """
        held = [self.held.name] if self.held is not None else []
        adopted = not held
        if adopted:
            _world, held = self.mc.object_ids()
            if len(held) > 1:
                # One gripper, so this is a scene left in a state no pick
                # produces. Release them all rather than pick a favourite.
                self.get_logger().warn('%d objects attached: %s'
                                       % (len(held), ', '.join(held)))
            for name in held:
                self.get_logger().info('placing %r, attached by an earlier run'
                                       % name)
        if not held:
            self.get_logger().warn('place() with nothing held -- releasing anyway')

        self.mc.move_named(state)
        self.mc.open_gripper()
        for name in held:
            self.mc.detach(name)
            self.mc.remove(name)
        if held:
            if adopted:
                # Marker ids are per process and handed out in first-drawn
                # order, so hide() here would delete whichever marker happens to
                # share the id. DELETEALL needs no ids -- see markers.clear.
                self.markers.clear()
            else:
                self.markers.hide(self.held)
            self.held = None
        self.mc.move_named('carry')
        self.get_logger().info('placed%s'
                               % (' ' + ', '.join(held) if held else ''))

    # ------------------------------------------------------------------ reset

    def reset(self, state='ready', force=False):
        """Recover from a run that died partway: clear, release, go home.

        A failed pick leaves the arm wherever it stopped AND the object wherever
        it got to in the planning scene -- attached to the gripper, if it died
        after the grasp. THE NEXT RUN CANNOT UNDO THAT ON ITS OWN. _clear()
        drops the stale copy, but the sequence then puts a fresh one back in the
        same place and asks for 'ready' with the jaws still around it, which is
        a START state in collision. move_group rejects the whole plan as
        INVALID_MOTION_PLAN -- immediately, before OMPL is asked for anything,
        which is what the near-instant failure tells you.

        Hence the order here: scene first, jaws second, motion last. Planning
        cannot get out of a hole the scene is holding it in.

        Opening DROPS whatever is held, in place, before the arm moves. That is
        deliberate -- the alternative is carrying the object to 'ready' with the
        scene no longer holding it, and dropping it from up there instead.
        """
        if state not in NAMED_STATES:
            raise MoveItError('unknown named state %r; have %s'
                              % (state, sorted(NAMED_STATES)))

        world, attached = self.mc.object_ids()
        for name in attached:
            self.get_logger().info('detaching %r' % name)
            self.mc.detach(name)
        if attached:
            # Detaching puts them back into the WORLD, where remove() takes
            # them; the list read above predates that.
            world, _ = self.mc.object_ids()
        for name in world:
            if name in SCENE_FIXTURES:
                continue
            self.get_logger().info('removing %r' % name)
            self.mc.remove(name)
            # A run that died between allowing and forbidding leaves the pair
            # allowed -- an object the gripper is permitted to pass straight
            # through. Same repair _clear makes, for objects this process never
            # knew about.
            self.mc.allow_collisions(name, GRIPPER_LINKS, False)
        self.markers.clear()
        self.held = None

        self.mc.open_gripper()

        here = self.mc.current_joints()
        valid, contacts = self.mc.check_state(here)
        if not valid:
            self.get_logger().warn(
                'still in collision with the scene cleared: %s. No planned '
                'move can start here; --force drives out blind.' % contacts)
        try:
            self.mc.move_named(state)
        except MoveItError as exc:
            if not force:
                raise
            self.get_logger().warn('%s -- forcing a blind move to %r'
                                   % (exc, state))
            self._blind_move(NAMED_STATES[state])
        self.get_logger().info('reset to %r' % state)

    def _blind_move(self, joints, steps=40):
        """Joint-space interpolation to `joints`, NOT COLLISION CHECKED.

        The escape hatch for a start state MoveIt will not plan out of. Every
        planned move validates its own first waypoint against the live scene, so
        a pose that really is in collision -- jaws in the floor, arm folded into
        the chassis -- cannot be planned out of at all. It can only be driven
        out of, which means going straight to the controller past move_group.

        Every joint moves monotonically, so the path stays inside the box the
        two endpoints span and is the shortest one between them. That is the
        whole of the safety argument, and it says nothing about what is in that
        box. Watch the arm, and have the power switch to hand.
        """
        here = self.mc.current_joints()
        points = [[a + (b - a) * (i + 1) / steps
                   for a, b in zip(here, joints)] for i in range(steps)]
        times = self.mc.time_parameterize(points, start=here)
        self.get_logger().warn('BLIND move over %.1f s, nothing checked'
                               % times[-1])
        self.mc.execute(points, times)

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
                        help='solve and collision-check the sequence, move '
                             'nothing; leaves the object in the scene and drawn')
    parser.add_argument('--check-states', action='store_true',
                        help='validate every named state and exit')
    parser.add_argument('--reset', nargs='?', const='ready', default=None,
                        metavar='STATE',
                        help='recovery: empty the planning scene, open the '
                             'gripper and move to STATE (default ready)')
    parser.add_argument('--force', action='store_true',
                        help='with --reset, drive out blind -- no collision '
                             'checking -- if MoveIt will not plan from where '
                             'the arm is')
    # One half per invocation, and one of them is required. There is no
    # combined form: --pick leaves the object held at 'carry' and the process
    # gone, and the pause before --place is where the caller decides where the
    # object goes. Mutually exclusive, so argparse rejects the pair itself.
    half = parser.add_mutually_exclusive_group()
    half.add_argument('--pick', action='store_true',
                      help='pick the object up and carry it, stopping short of '
                           'the place. Needs x y z')
    half.add_argument('--place', action='store_true',
                      help='place what the gripper is already holding, without '
                           'picking anything first. Takes no coordinates')
    cli = parser.parse_args(remove_ros_args(sys.argv)[1:])

    # All settled before rclpy.init, so a bad command line costs nothing and
    # says what the choices are rather than starting a node to find out.
    modes = (cli.check_states, cli.reset, cli.plan_only, cli.pick, cli.place)
    if not any(modes):
        parser.print_usage(sys.stderr)
        print('give a mode: --pick x y z, --place, --plan-only x y z, --reset '
              'or --check-states', file=sys.stderr)
        return 2
    if cli.place and (cli.x is not None or cli.y is not None
                      or cli.z is not None):
        print('--place takes no coordinates: it places what the gripper is '
              'already holding, wherever that came from', file=sys.stderr)
        return 2
    if cli.plan_only and cli.place:
        print('--plan-only is a dry run of the pick, and has nothing to say '
              'about --place', file=sys.stderr)
        return 2

    rclpy.init(args=args)
    node = PickPlace()
    status = 0
    try:
        if cli.check_states:
            status = 0 if node.check_states() else 1
        elif cli.reset:
            node.reset(cli.reset, force=cli.force)
        elif cli.place:
            node.place()
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
