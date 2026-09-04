#!/usr/bin/env python3
# coding: utf-8
"""
Wave the arm -- a greeting gesture, planned and executed like any other move.

    ros2 run dofbot_ctrl wave_arm
    ros2 run dofbot_ctrl wave_arm -- --waves 3
    ros2 run dofbot_ctrl wave_arm -- --seconds 2 --finish carry

Runs against the live pick_place stack: move_group plans the way in, and
moveit_bridge drives the servos. Nothing here opens the serial port, so the arm
stays enabled throughout and the wave is collision-checked against whatever the
planning scene holds -- unlike the pre-ROS wave_arm.py this replaces, which
wrote servo angles blind.

Three phases, because they want different machinery:

  1. INTO the gesture, from wherever the arm happens to be. An OMPL joint-space
     goal, because the start pose is unknown and the arm may have to come around
     something to reach RAISED.
  2. The gesture itself: RAISED -> A -> B -> A -> ... -> RAISED, sent as ONE
     timed trajectory. Planning each leg separately would stop the arm dead at
     every waypoint, and a wave that pauses at each end of its swing does not
     read as a wave. The path is known in advance and every waypoint along it is
     checked against the live scene before a byte is sent.
  3. Back to a named state to finish -- `init`, the folded stow pose, by
     default. `--finish ""` stops in the raised pose instead.

The gripper is left alone -- ARM_JOINT_NAMES is the five arm joints, and what
the jaws are holding is not the wave's business.
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from dofbot_ctrl.moveit_client import NAMED_STATES, DofbotMoveIt, MoveItError

# WHERE THE ARM HOLDS ITSELF. Shoulder, elbow and wrist pitch, from
# kin.ik_best(0.03, 0.0, 0.40, phi=0.0) on the elbow-forward branch: the tool
# 400 mm up and only 30 mm forward, pointing straight up. Hand held high, all
# but on the yaw axis.
#
# ON THE AXIS IS THE POINT. The wave is made by arm1, because arm1 is the only
# joint on this arm that moves the hand sideways at all -- arm2/3/4 are all
# pitch in one plane and arm5 is a roll. With the tool ON the yaw axis, that
# rotation twists the hand and sweeps the forearm while the hand itself stays
# overhead, which is what waving looks like. Hold the tool out at arm's length
# instead and the same rotation swings the whole arm through an arc, which
# looks like the robot turning to face somewhere else. The pre-ROS wave was an
# on-axis one (its tool moved 8 mm end to end) and this keeps that.
#
# WHAT CHANGED FROM THAT WAVE, AND WHY. It reached the pose by folding the arm
# BACK over the base. Fine when the model was the arm alone; against
# chassis.xacro it puts arm4_Link 26 mm from the chassis cylinder and the swing
# fails validation outright (`arm4_Link/chassis_link`). Here the arm bows
# FORWARD to the same overhead point, so the elbow is out over the pick zone
# and the gesture keeps ~0.10 m of clearance throughout -- near the most there
# is, the shoulder axis itself being 0.10 m from the cylinder.
BASE_PITCH = (0.66672, -0.86743, 0.20071)

# Yaw either side of centre. 40 degrees is wide enough to read across a room
# and well inside arm1's -109..+108 range.
SWING = 0.69813

# The two things that stop the swing being a rigid sweep. FLICK rolls the wrist
# with the swing, PITCH_FLICK tips the hand forward at one end and back at the
# other, so the hand rocks as it goes over rather than arriving flat. Together
# they take the hand from 37 mm of travel to 85 mm and cost 3 mm of clearance.
FLICK = 0.34907
PITCH_FLICK = 0.34907

# Centre of the swing and its two ends, in ARM_JOINT_NAMES order. RAISED is a
# pose the arm passes THROUGH mid-wave, not a fourth corner.
RAISED = (0.0,) + BASE_PITCH + (0.0,)
WAVE_A = ((SWING,) + BASE_PITCH[:2]
          + (BASE_PITCH[2] + PITCH_FLICK, FLICK))
WAVE_B = ((-SWING,) + BASE_PITCH[:2]
          + (BASE_PITCH[2] - PITCH_FLICK, -FLICK))

DEFAULT_FINISH = 'init'

# HOW LONG ONE WAVE TAKES, end to end -- the swing out, the two passes and the
# swing home, but not the planned moves in and out.
#
# A duration rather than a speed because that is the thing being chosen: a
# greeting has a pace, and 3 seconds is roughly the pace of the pre-ROS wave.
# moveit_client's own max_joint_speed of 30 deg/s would take 12 s and read as
# the arm being unwell. That 30 is set for a PICK, where the servos trailing
# their commanded position by moveit_bridge's 200 ms track_time_ms means the
# gripper arrives somewhere other than where the plan put it. A wave has nothing
# to arrive at; the same lag only softens the ends of the swing.
WAVE_SECONDS = 3.0

# Radians between waypoints along a leg. The legs are straight lines in joint
# space and the endpoints are known good, but a 1.4 rad base sweep can still
# pass through something the endpoints miss, so the line is sampled at roughly
# the resolution plan_cartesian uses for its own paths.
CHECK_STEP = math.radians(5.0)


def interpolate(start, end, step=CHECK_STEP):
    """Waypoints from `start` to `end`, no joint moving more than `step` between
    consecutive ones. Excludes `start`, includes `end`."""
    span = max(abs(a - b) for a, b in zip(start, end))
    count = max(1, int(math.ceil(span / step)))
    return [tuple(a + (b - a) * (i / count) for a, b in zip(start, end))
            for i in range(1, count + 1)]


def gesture_distance(waves=1):
    """Max-norm path length of the swings, in radians -- the quantity
    time_parameterize's speed cap applies to.

    Summing the legs' endpoints is exact rather than an approximation: each leg
    is a straight line in joint space, so every joint's share of it is constant
    and the dominating joint dominates every sub-step of it too.
    """
    legs = [(RAISED, WAVE_A)] + [(WAVE_A, WAVE_B), (WAVE_B, WAVE_A)] * waves
    legs.append((WAVE_A, RAISED))
    return sum(max(abs(a - b) for a, b in zip(*leg)) for leg in legs)


def speed_for(seconds, waves=1):
    """Peak joint speed (rad/s) that makes `waves` waves last `seconds` each.

    time_parameterize spends a quarter of the distance ramping up and a quarter
    ramping down, which works out at exactly 1.5 * distance / speed however long
    the path is -- so the speed a duration implies is closed form, not a search.
    """
    return 1.5 * gesture_distance(waves) / (seconds * waves)


class Waver(Node):

    def __init__(self):
        super().__init__('wave_arm')
        self.mc = DofbotMoveIt(self)

    def _leg(self, start, end, what):
        """One straight joint-space leg, validated against the live scene."""
        points = interpolate(start, end)
        for i, joints in enumerate(points):
            valid, contacts = self.mc.check_state(joints)
            if not valid:
                raise MoveItError('%s is blocked at waypoint %d/%d: %s'
                                  % (what, i + 1, len(points), contacts))
        return points

    def wave(self, waves=1, finish=DEFAULT_FINISH, seconds=WAVE_SECONDS):
        """Move into the gesture, wave `waves` times, then stow at `finish`.

        `seconds` is how long ONE wave takes, so the pace is the same whatever
        the count. Only the swings are timed here; the moves in and out are
        ordinary planned moves at move_group's own velocity scaling, which is
        what should be careful -- they start from wherever the arm was left.
        """
        self.get_logger().info('moving into the wave')
        self.mc.move_joints(RAISED)

        # However many waves are asked for, there are only four distinct lines
        # to validate -- the two swings and the two ends. Checking a waypoint is
        # a service round trip, so caching keeps --waves 10 as cheap to set up
        # as --waves 1.
        legs = {}

        def leg(start, end, what):
            key = (start, end)
            if key not in legs:
                legs[key] = self._leg(start, end, what)
            return legs[key]

        # One wave is A -> B -> back to A, so N of them is a continuous
        # oscillation between the two, entered and left through A.
        segments = [leg(RAISED, WAVE_A, 'the swing up')]
        for _ in range(waves):
            segments.append(leg(WAVE_A, WAVE_B, 'the swing across'))
            segments.append(leg(WAVE_B, WAVE_A, 'the swing back'))
        segments.append(leg(WAVE_A, RAISED, 'the return to raised'))

        # start=RAISED, not current_joints(): move_joints() has just put the arm
        # there, and the timing has to be measured from where the trajectory
        # begins or its first point is stamped too early to reach.
        speed = speed_for(seconds, waves)
        points, times = self.mc.merge(segments, start=RAISED, max_speed=speed)
        self.get_logger().info('waving %d time(s): %d waypoints over %.1f s '
                               'at %.0f deg/s peak'
                               % (waves, len(points), times[-1],
                                  math.degrees(speed)))
        self.mc.execute(points, times)

        if finish:
            self.get_logger().info('stowing at %r' % finish)
            self.mc.move_named(finish)


def main(args=None):
    parser = argparse.ArgumentParser(
        prog='wave_arm', description=__doc__.split('\n\n')[0])
    parser.add_argument('--waves', type=int, default=1,
                        help='back-and-forth waves (default: %(default)s)')
    parser.add_argument('--finish', default=DEFAULT_FINISH,
                        help='named state to stow at afterwards (%s), or "" to '
                             'stop in the raised pose (default: %%(default)s)'
                             % ', '.join(sorted(NAMED_STATES)))
    parser.add_argument('--seconds', type=float, default=WAVE_SECONDS,
                        help='how long ONE wave takes, so the pace holds '
                             'whatever --waves says (default: %(default)s)')
    cli = parser.parse_args(remove_ros_args(sys.argv)[1:])

    # Settled before rclpy.init, so a typo costs nothing and says what the
    # choices are rather than failing partway through a move.
    if cli.waves < 1:
        print('--waves must be at least 1', file=sys.stderr)
        return 2
    if cli.finish and cli.finish not in NAMED_STATES:
        print('unknown state %r; have %s'
              % (cli.finish, ', '.join(sorted(NAMED_STATES))), file=sys.stderr)
        return 2
    if cli.seconds <= 0:
        print('--seconds must be positive', file=sys.stderr)
        return 2

    rclpy.init(args=args)
    node = Waver()
    status = 0
    try:
        node.wave(cli.waves, cli.finish, cli.seconds)
    except MoveItError as exc:
        node.get_logger().error(str(exc))
        status = 1
    except KeyboardInterrupt:
        # The trajectory is already with the controller; this only stops us
        # waiting on it. Where the arm stops is whatever the bridge does next.
        status = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == '__main__':
    sys.exit(main())
