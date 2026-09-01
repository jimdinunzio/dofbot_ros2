#!/usr/bin/env python3
# coding: utf-8
"""
A Python interface to the running MoveIt stack, built from rclpy clients only.

There is no `moveit_commander` on Humble and no Python binding for MoveGroup-
Interface in this install, so everything here talks to move_group and the
controllers over their plain ROS interfaces:

    /move_action                                    MoveGroup       (action)
    /check_state_validity                           GetStateValidity (service)
    /apply_planning_scene                           ApplyPlanningScene (service)
    /get_planning_scene                             GetPlanningScene (service)
    /arm_group_controller/follow_joint_trajectory   FollowJointTrajectory
    /grip_group_controller/gripper_cmd              GripperCommand
    /joint_states                                   JointState       (topic)
    /moveit_bridge/get_parameters                   GetParameters    (service)

The last one is the odd one out and is deliberate: everything else here talks to
MoveIt, which is driving MOCK joints. set_gripper asks the bridge how long the
real servo takes, because that is the one place a mock joint's word is not good
enough -- see _gripper_settle().

TWO KINDS OF MOTION, AND WHY
----------------------------
Long transit moves (`move_joints`, `move_named`, `move_pose`) go to move_group as
JOINT-space goals, so OMPL still does collision-aware planning against the
chassis, the floor and anything in the planning scene. Only the goal is
joint-space -- the path is still planned.

Short straight-line moves (`cartesian_move`) are planned here instead of by
move_group. This is a convenience choice, NOT a workaround: measured on this
robot, /compute_cartesian_path returns fraction = 1.000 on the approach and lift
segments below, and /compute_ik solves 116/120 exactly-reachable pose goals at
the 5 ms timeout it shipped with. MoveIt's Cartesian planner works fine here.

What we get by doing it ourselves:

- The interface is (x, y, z, phi, roll). A MoveIt pose goal needs a full
  quaternion, and on a 5-DOF arm only the orientations on the reachable manifold
  have solutions -- so to ask MoveIt for a pose you must already know an
  achievable orientation, which means doing this arm's kinematics anyway.
- A rejected segment says which waypoint failed and why, geometrically, instead
  of returning a partial `fraction` the caller has to interpret. (Yahboom's own
  lesson retries 1000 times and then tests `fraction >= 0.0`, which is always
  true, so it "succeeds" on total failure.)
- Analytic IK is exact, picks the elbow branch deliberately, and costs
  microseconds, so seeding each waypoint from the last one is free and a segment
  cannot flip configuration halfway through.

BLOCKING, AND WHERE YOU MAY CALL IT FROM
----------------------------------------
Every method here is synchronous: it spins the node until the result arrives.
That makes a pick sequence read as straight-line code, which is the point. The
cost is that these methods MUST NOT be called from inside a subscription or
timer callback of the same node -- that would recurse into the executor. Call
them from main(), or from a thread that owns the spinning.
"""

from math import degrees, radians

import rclpy
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (AllowedCollisionEntry, AttachedCollisionObject,
                             CollisionObject, Constraints, JointConstraint,
                             MoveItErrorCodes, PlanningScene,
                             PlanningSceneComponents, RobotState)
from moveit_msgs.srv import (ApplyPlanningScene, GetPlanningScene,
                             GetStateValidity)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dofbot_ctrl import dofbot_kinematics as kin
from dofbot_ctrl import gripper
from dofbot_ctrl.joint_map import ARM_JOINT_NAMES, GRIPPER_JOINT_NAME

ARM_GROUP = 'arm_group'

# The six finger links, spelled as the URDF spells them.
FINGER_LINKS = ('Rlink1_Link', 'Rlink2_Link', 'Rlink3_Link',
                'Llink1_Link', 'Llink2_Link', 'Llink3_Link')

# The frame the gripper geometry is measured in, and the link an object is
# attached to. arm5_Link is the SRDF end_effector parent; Gripping_point_Link is
# the TCP frame hanging off it.
TOOL_LINK = 'Gripping_point_Link'
WRIST_LINK = 'arm5_Link'

# Everything a grasped object is allowed to touch: the whole end effector, not
# just the fingers. arm5_Link has to be in here, and measurably so -- its mesh
# runs to z = 0.0503 while the TCP sits at z = 0.0681, and the fingers are only
# ~50 mm long, so an object held between the jaws is 20 mm from the wrist body
# and anything with a corner (a 30 mm cube at a tilted grasp pitch) overlaps it.
# Leaving arm5_Link out fails the approach with `arm5_Link/<object>` and, after
# attaching, fails every subsequent plan. This is the SRDF end_effector group
# plus its parent link, which is the right boundary.
GRIPPER_LINKS = FINGER_LINKS + (TOOL_LINK, WRIST_LINK)

BASE_FRAME = 'base_link'

# The node that owns the real servos, and the two parameters holding how long
# it gives the jaws -- closing onto an object, and opening onto nothing. Asked
# for rather than copied: a second number here that drifted from the bridge's
# would silently reintroduce the truncation bug.
BRIDGE_NODE = 'moveit_bridge'
BRIDGE_GRIP_TIME = 'grip_time_ms'
BRIDGE_OPEN_TIME = 'open_time_ms'

# Named arm states, in ARM_JOINT_NAMES order. These mirror the group_states in
# dofbot_description.srdf -- change both together.
#
# 'up' and 'down' are the vendor's originals and are kept as they were.
#
# 'init' is Jim's, posed by hand on the real arm and read off with calibrate_zero
# as {1: 1.0, 2: 69.0, 3: -84.0, 4: -92.0, 5: -2.0} -- encoder median minus 90.
# Through joint_map.servo_to_urdf (which applies the per-servo zero offsets and
# the 2/3/4 sign flip) that is {1.00, -67.00, 86.00, 94.00, 0.00} degrees.
#
#   arm4 is clamped from 94.00 to 89.00 deg. Two reasons, and the arm cannot do
#   the measured value either way: the URDF limit is +-1.57 rad = 89.95 deg, and
#   the hardware maximum is +92.00 deg (servo 4 against its 0 stop, given its +2
#   zero offset). A reading of -2 means the joint was posed past its mechanical
#   stop by hand. 89.00 keeps a degree of margin off the stop. It costs 13 mm of
#   TCP height, 0.183 -> 0.196. Raising arm4_Joint's URDF limit to the hardware
#   range would recover it, at the price of asymmetric limits.
#
# The old vendor 'init' {0, -1.57, 1.57, 1.57, 0} is gone: /check_state_validity
# reports arm3_Link/chassis_link, which only became a collision once chassis_link
# and floor_link became real URDF links.
#
# 'ready' was solved with dofbot_kinematics; 'carry' and 'over_trash' are
# measured on the arm rather than derived. All three are checked against the
# live planning scene with /check_state_validity (`ros2 run dofbot_ctrl
# pick_place --check-states`). theta1 is set to exactly zero on the on-axis
# states rather than to the -0.007 the IK returns; that 0.6 mm lateral tool
# offset is not worth carrying in a named pose.
#
# 'carry' and 'over_trash' sit within 0.07 rad of the same phi: a held object
# keeps the tilt it was grasped at, so moving between them translates the load
# without tipping it. Both are on-axis, so that move no longer yaws it either.
NAMED_STATES = {
    #                theta1  theta2   theta3   theta4  theta5
    'up':         (0.0,     0.0,     0.0,     0.0,    0.0),
    'down':       (0.0,     1.57,    0.0,     0.0,    0.0),
    # folded and compact, TCP (0.091, 0.002, 0.178) phi=1.885 -- the rest/stow
    # pose, hand-posed on the real arm; see the note above on arm4
    'init':       (0.0175, -1.1694,  1.5010,  1.5533, 0.0),
    # observation: arm up and tucked back, TCP (0.100, 0, 0.322) phi=1.0, so the
    # chassis camera has a clear view of the floor in front
    'ready':      (0.0,    -0.7932,  1.2775,  0.5157, 0.0),
    # arm high and tucked, TCP (0.082, 0.001, 0.325) phi=1.08 -- safe to drive
    'carry':      (0.0,    -0.718,  0.840,  0.959, 0.0),
    # release pose, TCP (0.167, 0.001, 0.334) phi=1.15: straight in front and
    # high. The measured posture off the left front, rotated on-axis -- theta1
    # is the only joint that moved, so the height, the tilt and the joint room
    # are the ones that were measured on the arm.
    #
    # A PLACEHOLDER FOR A DETECTED BIN, not a surveyed one. It assumes a rim
    # below ~0.3 m, 0.167 m out in front, which is barely past the chassis; the
    # bin's real position is to come from perception, and place() will take
    # coordinates the way pick() does. Until then this is where it drops.
    'over_trash': (0.0,   0.229, -0.025,  0.942, 0.0),
}


class MoveItError(RuntimeError):
    """A MoveIt action or service reported failure."""


def _error_name(code):
    for name, value in MoveItErrorCodes.__dict__.items():
        if name.isupper() and not name.startswith('_') and value == code:
            return '%s (%d)' % (name, code)
    return str(code)


class DofbotMoveIt:
    """MoveIt + controller access for one node. See the module docstring."""

    def __init__(self, node, timeout=10.0):
        self.node = node
        self.timeout = timeout
        self.log = node.get_logger()

        # Geometry of every collision object we have added, kept so attach() can
        # re-express the same shape in a link frame without asking move_group
        # what it currently thinks the object is.
        self._objects = {}

        self._joint_state = None
        node.create_subscription(JointState, '/joint_states', self._on_js, 10)

        self._move = ActionClient(node, MoveGroup, '/move_action')
        self._traj = ActionClient(
            node, FollowJointTrajectory,
            '/arm_group_controller/follow_joint_trajectory')
        self._grip = ActionClient(
            node, GripperCommand, '/grip_group_controller/gripper_cmd')

        self._validity = node.create_client(GetStateValidity,
                                            '/check_state_validity')
        self._apply = node.create_client(ApplyPlanningScene,
                                         '/apply_planning_scene')
        self._scene = node.create_client(GetPlanningScene,
                                         '/get_planning_scene')
        # Not MoveIt's. The one question only the hardware bridge can answer.
        self._bridge = node.create_client(
            GetParameters, '/%s/get_parameters' % BRIDGE_NODE)
        self._no_bridge_logged = False

        # Motion tuning. max_joint_speed is not cosmetic: moveit_bridge writes to
        # the servos at 10 Hz and skips deltas under 0.5 servo-degrees, so a
        # trajectory faster than roughly 30 deg/s is one the real arm cannot
        # follow -- it will lag and arrive late.
        self._declare('max_joint_speed', radians(30.0))
        self._declare('cartesian_step', 0.005)      # m between waypoints
        self._declare('planning_time', 5.0)
        self._declare('planning_attempts', 10)
        self._declare('velocity_scaling', 0.2)
        self._declare('acceleration_scaling', 0.2)

    # ---------------------------------------------------------------- plumbing

    def _declare(self, name, default):
        if not self.node.has_parameter(name):
            self.node.declare_parameter(name, default)

    def param(self, name):
        return self.node.get_parameter(name).value

    def _on_js(self, msg):
        self._joint_state = msg

    def _spin_until(self, future, timeout=None, what='request'):
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout or self.timeout)
        if not future.done():
            raise MoveItError('%s timed out after %.1f s'
                              % (what, timeout or self.timeout))
        return future.result()

    def _call(self, client, request, what):
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise MoveItError('%s is not available -- is move_group running?'
                              % client.srv_name)
        return self._spin_until(client.call_async(request), what=what)

    def _send_goal(self, client, goal, what, timeout=None):
        """Send an action goal and block until the result. Returns the result."""
        if not client.wait_for_server(timeout_sec=self.timeout):
            raise MoveItError('%s action server is not available'
                              % client._action_name)
        handle = self._spin_until(client.send_goal_async(goal),
                                  what='%s goal' % what)
        if not handle.accepted:
            raise MoveItError('%s goal was rejected' % what)
        return self._spin_until(handle.get_result_async(),
                                timeout=timeout or 120.0,
                                what='%s result' % what).result

    # ------------------------------------------------------------------- state

    def current_joints(self, timeout=5.0, include_gripper=False):
        """The five arm joint angles (radians), from /joint_states.

        Blocks until a message that contains all of them has arrived.
        """
        names = (ARM_JOINT_NAMES + (GRIPPER_JOINT_NAME,) if include_gripper
                 else ARM_JOINT_NAMES)
        end = self.node.get_clock().now().nanoseconds + int(timeout * 1e9)
        while self.node.get_clock().now().nanoseconds < end:
            if self._joint_state is not None:
                by_name = dict(zip(self._joint_state.name,
                                   self._joint_state.position))
                if all(n in by_name for n in names):
                    return [by_name[n] for n in names]
            rclpy.spin_once(self.node, timeout_sec=0.1)
        raise MoveItError('no /joint_states message containing %r after %.1f s'
                          % (list(names), timeout))

    def current_pose(self, tip='tcp'):
        """(x, y, z, phi, roll) of the tool, by our own FK on the live joints."""
        return kin.fk(self.current_joints(), tip)

    def check_state(self, joints, grip=None):
        """(valid, contacts) for an arm configuration in the LIVE scene.

        Sent as a diff, so the current scene -- including anything attached to
        the gripper -- is what the configuration is checked against. The service
        always fills in the contact list, and naming the pair is most of the
        diagnostic value: "in collision" does not distinguish the object from
        the floor from the robot's own body, and the fix is different for each.

        `grip` overrides Rlink1_Joint instead of taking it from the live state.
        That matters more than it sounds: the four-bar swings the fingertips
        outwards and DOWN as it closes, so a pose that clears the floor with the
        jaws open does not necessarily clear it once they shut on something.
        """
        req = GetStateValidity.Request()
        req.group_name = ARM_GROUP
        req.robot_state = RobotState()
        req.robot_state.is_diff = True
        req.robot_state.joint_state.name = list(ARM_JOINT_NAMES)
        req.robot_state.joint_state.position = [float(q) for q in joints]
        if grip is not None:
            req.robot_state.joint_state.name.append(GRIPPER_JOINT_NAME)
            req.robot_state.joint_state.position.append(float(grip))
        res = self._call(self._validity, req, 'check_state_validity')
        pairs = sorted({(c.contact_body_1, c.contact_body_2)
                        for c in res.contacts})
        return res.valid, ', '.join('%s/%s' % p for p in pairs) or 'none reported'

    def state_valid(self, joints, verbose=False, grip=None):
        """Is this arm configuration collision-free in the live planning scene?"""
        valid, contacts = self.check_state(joints, grip)
        if verbose and not valid:
            self.log.warn('state invalid; contacts: %s' % contacts)
        return valid

    # ------------------------------------------------------- planned transit

    def move_joints(self, joints, check_goal=True):
        """Plan (OMPL, collision-aware) and execute to a joint-space goal.

        The goal is validated before OMPL is asked, purely for the error
        message: a colliding goal comes back from /move_action as
        `FAILURE (99999)`, which says nothing about what was hit, while
        /check_state_validity names the pair.
        """
        joints = [float(q) for q in joints]
        for name, q, (lo, hi) in zip(ARM_JOINT_NAMES, joints, kin.JOINT_LIMITS):
            if not lo <= q <= hi:
                raise MoveItError('%s = %.3f rad is outside its %.3f..%.3f range'
                                  % (name, q, lo, hi))
        if check_goal:
            valid, contacts = self.check_state(joints)
            if not valid:
                raise MoveItError('goal state is in collision: %s' % contacts)

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = ARM_GROUP
        req.num_planning_attempts = int(self.param('planning_attempts'))
        req.allowed_planning_time = float(self.param('planning_time'))
        req.max_velocity_scaling_factor = float(self.param('velocity_scaling'))
        req.max_acceleration_scaling_factor = float(
            self.param('acceleration_scaling'))
        req.start_state.is_diff = True          # plan from wherever we are now

        constraints = Constraints()
        for name, q in zip(ARM_JOINT_NAMES, joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = q
            jc.tolerance_above = 1e-4
            jc.tolerance_below = 1e-4
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints = [constraints]

        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        result = self._send_goal(self._move, goal, 'move_joints')
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise MoveItError('planning/execution failed: %s'
                              % _error_name(result.error_code.val))
        return result

    def move_named(self, name):
        """Move to one of NAMED_STATES."""
        if name not in NAMED_STATES:
            raise MoveItError('unknown named state %r; have %s'
                              % (name, sorted(NAMED_STATES)))
        self.log.info('move_named(%r)' % name)
        return self.move_joints(NAMED_STATES[name])

    def move_pose(self, x, y, z, phi, roll=0.0, tip='tcp'):
        """Plan to a tool pose. Exact analytic IK, then a joint-space goal.

        Not a MoveIt pose goal -- but not because KDL cannot solve it. The
        reason is the argument list: (x, y, z, phi, roll) is five numbers for
        five joints, whereas a pose goal needs a quaternion, and on this arm only
        the orientations on the reachable manifold have solutions. Solving here
        also means an unreachable request gets a geometric reason rather than a
        bare NO_IK_SOLUTION.
        """
        seed = self.current_joints()
        joints = kin.ik_best(x, y, z, phi, roll, seed=seed, tip=tip)
        if joints is None:
            raise MoveItError(
                'pose (%.3f, %.3f, %.3f) phi=%.3f roll=%.3f is %s'
                % (x, y, z, phi, roll, kin.describe(x, y, z, phi, roll, tip)))
        self.log.info('move_pose(%.3f, %.3f, %.3f, phi=%.3f) -> %s'
                      % (x, y, z, phi, ['%.3f' % q for q in joints]))
        return self.move_joints(joints)

    # ------------------------------------------------- straight-line segments

    def plan_cartesian(self, target, roll=None, start=None, step=None,
                       tip='tcp', validate=True):
        """Interpolate a straight line in (x, y, z, phi) and IK every waypoint.

        target: (x, y, z, phi) of the end of the segment.
        start:  same, defaults to the current tool pose.
        roll:   theta5 along the segment; defaults to the current roll.

        Returns the list of joint vectors, starting with the waypoint AFTER the
        start pose. Raises MoveItError naming the first waypoint that fails, so
        a rejected segment says where and why -- unlike a partial `fraction`.

        phi is interpolated linearly along with position. A pure translation is
        the common case (same phi at both ends), and then this is exactly a
        straight line in space at constant tool tilt.
        """
        seed = self.current_joints()
        if start is None:
            x0, y0, z0, phi0, roll0 = kin.fk(seed, tip)
        else:
            x0, y0, z0, phi0 = start
            roll0 = seed[4]
        if roll is None:
            roll = roll0
        x1, y1, z1, phi1 = target

        step = float(step if step is not None else self.param('cartesian_step'))
        distance = max(abs(x1 - x0), abs(y1 - y0), abs(z1 - z0))
        # Enough steps that neither the translation nor the tilt outruns `step`
        # (the tilt is converted at the tool length, so both are metres).
        arc = abs(phi1 - phi0) * kin.reach_limits(tip)['tool_length']
        n = max(2, int(round(max(distance, arc) / step)))

        points = []
        for i in range(1, n + 1):
            f = i / float(n)
            wx = x0 + (x1 - x0) * f
            wy = y0 + (y1 - y0) * f
            wz = z0 + (z1 - z0) * f
            wphi = phi0 + (phi1 - phi0) * f
            joints = kin.ik_best(wx, wy, wz, wphi, roll, seed=seed, tip=tip)
            if joints is None:
                raise MoveItError(
                    'straight-line segment rejected at waypoint %d/%d '
                    '(%.3f, %.3f, %.3f) phi=%.3f: %s'
                    % (i, n, wx, wy, wz, wphi,
                       kin.describe(wx, wy, wz, wphi, roll, tip)))
            if validate:
                valid, contacts = self.check_state(joints)
                if not valid:
                    raise MoveItError(
                        'straight-line segment rejected at waypoint %d/%d '
                        '(%.3f, %.3f, %.3f): collision between %s'
                        % (i, n, wx, wy, wz, contacts))
            points.append(joints)
            seed = joints          # carry the seed so the elbow cannot flip
        return points

    def time_parameterize(self, points, start=None, max_speed=None):
        """Trapezoidal time_from_start for a waypoint list.

        MoveIt's IterativeParabolicTimeParameterization has no Python binding on
        Humble, so this does the job directly. Distance is the max-norm over the
        joints, which is the quantity the speed cap applies to (no joint may
        exceed max_speed). The ramp keeps the real servos from being asked for a
        step change in velocity at either end.

        Returns times in seconds, one per point, strictly increasing.
        """
        speed = float(max_speed if max_speed is not None
                      else self.param('max_joint_speed'))
        prev = list(start if start is not None else self.current_joints())

        cumulative = []
        total = 0.0
        for p in points:
            total += max(abs(a - b) for a, b in zip(p, prev))
            cumulative.append(total)
            prev = p
        if total <= 0.0:
            return [0.1 * (i + 1) for i in range(len(points))]

        # Trapezoid: spend a quarter of the distance accelerating and a quarter
        # decelerating, cruising at `speed` in between. Degenerates to a pure
        # triangle profile for short moves, which is the ramp == total/2 case.
        ramp = min(0.25 * total, 0.5 * total)
        accel = speed * speed / (2.0 * ramp)
        t_ramp = speed / accel
        t_total = 2.0 * t_ramp + (total - 2.0 * ramp) / speed

        def when(s):
            if s <= ramp:
                return (2.0 * s / accel) ** 0.5
            if s >= total - ramp:
                return t_total - (2.0 * (total - s) / accel) ** 0.5
            return t_ramp + (s - ramp) / speed

        times = []
        last = 0.0
        for s in cumulative:
            t = max(when(s), last + 1e-3)   # strictly increasing, always
            times.append(t)
            last = t
        return times

    def execute(self, points, times, wait=True):
        """Send a joint-space trajectory straight to the arm controller.

        Bypasses move_group on purpose: the path has already been validated
        waypoint by waypoint against the live scene by plan_cartesian.
        """
        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINT_NAMES)
        for joints, t in zip(points, times):
            pt = JointTrajectoryPoint()
            pt.positions = [float(q) for q in joints]
            pt.velocities = [0.0] * len(ARM_JOINT_NAMES)
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        result = self._send_goal(self._traj, goal, 'follow_joint_trajectory',
                                 timeout=times[-1] + 30.0)
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise MoveItError('trajectory execution failed: %d (%s)'
                              % (result.error_code, result.error_string))
        return result

    def cartesian_move(self, target, roll=None, start=None, step=None,
                       tip='tcp', max_speed=None):
        """Straight-line tool move: plan, validate, time, execute.

        target/start/roll are as plan_cartesian. Returns the joint waypoints
        actually executed, so segments can be concatenated by the caller for a
        single continuous trajectory (see merge()).
        """
        here = self.current_joints()
        points = self.plan_cartesian(target, roll, start, step, tip)
        times = self.time_parameterize(points, start=here, max_speed=max_speed)
        self.log.info('cartesian_move to (%.3f, %.3f, %.3f) phi=%.3f: '
                      '%d waypoints over %.1f s'
                      % (target[0], target[1], target[2], target[3],
                         len(points), times[-1]))
        self.execute(points, times)
        return points

    def merge(self, segments, start=None, max_speed=None):
        """Concatenate several waypoint lists into one timed trajectory.

        Kept separate from cartesian_move so a multi-segment path can be sent as
        a single trajectory with a monotonically increasing time_from_start,
        rather than as several plan/execute round trips that stop dead between
        segments. Returns (points, times) ready for execute().

        pick_place does not use this yet -- its approach and lift are separated
        by the gripper closing, so there is nothing to smooth across. It is here
        for the case where consecutive segments really are continuous.
        """
        points = [p for seg in segments for p in seg]
        return points, self.time_parameterize(points, start=start,
                                              max_speed=max_speed)

    # ----------------------------------------------------------------- gripper

    def _sleep(self, seconds):
        """Block for `seconds` while still servicing this node's callbacks.

        The gripper wait is the only place here that waits on the clock rather
        than on a future, and it still has to keep /joint_states flowing --
        current_joints() reads exactly that, and the next thing a pick does
        after closing the jaws is read the arm's pose.
        """
        end = self.node.get_clock().now().nanoseconds + int(seconds * 1e9)
        while True:
            left = (end - self.node.get_clock().now().nanoseconds) / 1e9
            if left <= 0.0:
                return
            rclpy.spin_once(self.node, timeout_sec=left)

    def _gripper_settle(self, opening=False):
        """Seconds the real jaws still need after the action says they arrived.

        THE GRIPPER ACTION IS NOT A REPORT ABOUT THE HARDWARE. It finishes when
        the MOCK joint reaches the goal, which is the cycle after it is sent,
        while the servo is only just starting to move. Everything else here can
        ignore that gap, because an arm trajectory is re-sent every tick and the
        bridge just tracks it. The gripper is different: it is ONE write, and
        moveit_bridge hands the servo grip_time_ms to execute it over.

        Return before that elapses and the next arm write re-commands all six
        servos, replacing the slow move with a fast one -- so a 2000 ms close
        obeyed a lone action goal and did nothing at all inside a pick. It also
        means the arm would start lifting with the object not yet gripped.

        The duration is asked of the bridge, never copied, so the packet's time
        and the wait for it cannot drift apart. That is also why `opening`
        is passed rather than inferred here: the bridge sends the jaws open on
        a different, much shorter timer than it closes them, and this has to
        ask for whichever one it actually used.

        NO BRIDGE MEANS NO SERVOS -- simulation -- and nothing to wait for.
        """
        if not self._bridge.wait_for_service(timeout_sec=0.5):
            if not self._no_bridge_logged:
                self.log.info('no %s -- simulation, so not waiting on a servo'
                              % BRIDGE_NODE)
                self._no_bridge_logged = True
            return 0.0
        name = BRIDGE_OPEN_TIME if opening else BRIDGE_GRIP_TIME
        req = GetParameters.Request(names=[name])
        res = self._spin_until(self._bridge.call_async(req), timeout=2.0,
                               what='%s %s' % (BRIDGE_NODE, name))
        # An older bridge without the parameter answers PARAMETER_NOT_SET, which
        # is not an error here -- it means that bridge never slows the gripper,
        # so there is nothing extra to wait for.
        if not res.values or res.values[0].type != ParameterType.PARAMETER_INTEGER:
            return 0.0
        return res.values[0].integer_value / 1000.0

    def set_gripper(self, angle, max_effort=10.0):
        """Command Rlink1_Joint, in radians. 0 = open, ~1.5 = closed.

        Returns once the REAL jaws have finished moving, not merely once the
        action has -- see _gripper_settle() for why those are different and
        what goes wrong when the difference is ignored. Opening is much quicker
        than closing, so which one this is has to be worked out first.
        """
        # Direction, read BEFORE the goal is sent: the mock joint tracks the
        # command within a cycle, so afterwards the "current" angle is already
        # the one just asked for and every move looks stationary.
        opening = angle < self.current_joints(include_gripper=True)[-1]

        goal = GripperCommand.Goal()
        goal.command.position = float(angle)
        goal.command.max_effort = float(max_effort)
        self.log.info('set_gripper(%.3f rad = %.1f deg, %s)'
                      % (angle, degrees(angle),
                         'opening' if opening else 'closing'))
        # The gripper controller reports reached_goal=False when it stalls on an
        # object, which is the normal outcome of a successful grasp, so the
        # result is logged rather than raised on.
        result = self._send_goal(self._grip, goal, 'gripper_cmd', timeout=15.0)
        if not result.reached_goal:
            self.log.info('gripper stopped at %.3f rad (stalled -- expected '
                          'when it has hold of something)' % result.position)
        # Timed from HERE, with nothing deducted for the round trip: the bridge
        # samples at 10 Hz, so the servo has not necessarily been given the move
        # even by the time the action returns. Erring long costs a tenth of a
        # second; erring short lets the next arm write cut the close off, which
        # is the whole bug.
        self._sleep(self._gripper_settle(opening))
        return result

    def open_gripper(self):
        # gripper.OPEN_ANGLE, not 0.0. The linkage is over-centre and its span
        # peaks a little above the joint's lower limit; commanding 0 opens
        # slightly less wide. Matches the SRDF 'open' state.
        return self.set_gripper(gripper.OPEN_ANGLE)

    def close_gripper(self, angle=1.5):
        return self.set_gripper(angle)

    # ---------------------------------------------------------- planning scene

    def _apply_scene(self, scene, what):
        res = self._call(self._apply, ApplyPlanningScene.Request(scene=scene),
                         what)
        if not res.success:
            raise MoveItError('%s was refused by move_group' % what)
        return res

    def _add(self, object_id, primitives, poses, frame_id):
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.header.stamp = self.node.get_clock().now().to_msg()
        obj.id = object_id
        obj.primitives = primitives
        obj.primitive_poses = poses
        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        self._apply_scene(scene, 'add %r' % object_id)
        self._objects[object_id] = (primitives, poses)
        return obj

    @staticmethod
    def _pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)
        return pose

    def add_cylinder(self, object_id, x, y, z, height, radius,
                     frame_id=BASE_FRAME):
        """An upright cylinder centred at (x, y, z). Applied synchronously.

        Uses /apply_planning_scene rather than a /planning_scene topic diff (what
        chassis_collision does) because this one has to be in the scene before
        the very next plan, and a service returns success where a topic cannot.
        """
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.CYLINDER
        prim.dimensions = [float(height), float(radius)]
        return self._add(object_id, [prim], [self._pose(x, y, z)], frame_id)

    def add_box(self, object_id, x, y, z, size, frame_id=BASE_FRAME):
        """A box centred at (x, y, z); size is (dx, dy, dz)."""
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(d) for d in size]
        return self._add(object_id, [prim], [self._pose(x, y, z)], frame_id)

    def object_ids(self):
        """(world_ids, attached_ids) currently in the planning scene.

        Needed because remove() and detach() are NOT no-ops on something that
        is not there: move_group logs "Tried to remove world object ... but it
        does not exist" and returns success=False, which _apply_scene turns
        into a MoveItError. So a caller tidying up after a previous run has to
        look before it deletes.

        Asks for NAMES only -- WORLD_OBJECT_GEOMETRY would ship every vertex of
        every object back over the wire to answer "is it there?".
        """
        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        scene = self._call(self._scene, req, 'get_planning_scene').scene
        return ([o.id for o in scene.world.collision_objects],
                [a.object.id for a in
                 scene.robot_state.attached_collision_objects])

    def remove(self, object_id):
        obj = CollisionObject()
        obj.header.frame_id = BASE_FRAME
        obj.id = object_id
        obj.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        self._apply_scene(scene, 'remove %r' % object_id)
        self._objects.pop(object_id, None)

    def allow_collisions(self, object_id, links, allow=True):
        """Allow (or forbid) contact between one object and a set of links.

        Needed for the final approach: with the target in the planning scene,
        every waypoint that actually puts the jaws around it is "in collision",
        so a straight approach could not be validated at all. Allowing just
        object-vs-finger contact keeps the floor, the chassis and the rest of
        the arm checked, which is the part that matters.

        Sending an ACM in a scene diff REPLACES the matrix rather than merging
        into it, which would wipe every disable_collisions pair from the SRDF.
        So the current matrix is read back first and edited, not rebuilt.
        """
        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        acm = self._call(self._scene, req, 'get_planning_scene'
                         ).scene.allowed_collision_matrix

        names = list(acm.entry_names)
        rows = [list(entry.enabled) for entry in acm.entry_values]

        def index(name):
            if name not in names:
                names.append(name)
                for row in rows:
                    row.append(False)
                rows.append([False] * len(names))
            return names.index(name)

        i = index(object_id)
        for link in links:
            j = index(link)
            rows[i][j] = rows[j][i] = bool(allow)
        # index() grows earlier rows as it goes; make every row full width.
        width = len(names)
        for row in rows:
            row.extend([False] * (width - len(row)))

        acm.entry_names = names
        acm.entry_values = [AllowedCollisionEntry(enabled=row) for row in rows]

        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = acm
        return self._apply_scene(
            scene, '%s collisions between %r and the gripper'
                   % ('allow' if allow else 'forbid', object_id))

    def attach(self, object_id, link=TOOL_LINK, touch_links=GRIPPER_LINKS,
               pose=None, frame_id=None):
        """Attach a collision object to the gripper so it moves with the arm.

        Two modes:

        - pose=None: attach the object where it already is. move_group takes it
          out of the world and works out the link-relative transform itself.
        - pose=(x, y, z[, qx, qy, qz, qw]): re-place the object at that pose in
          `frame_id` (default: the link it is being attached to), reusing the
          geometry it was added with. This is the mode the pick sequence uses --
          once the jaws have closed, where the object IS is defined by the
          gripper, not by where it was believed to be beforehand. See
          scene_objects.held_pose, which produces it.

        `frame_id` may differ from `link`, and normally does: the pose is far
        easier to derive in arm5_Link (whose axes are the tool axes) than in
        Gripping_point_Link, which the URDF rotates by rpy="3.1416 -1.5708 0".

        Every link that touches the object must appear in touch_links, spelled
        exactly as the URDF spells it. MoveIt ignores touch links it does not
        recognise without complaining, so a name copy-pasted from another robot
        -- Yahboom's set_scene.cpp uses the Pro's "llink2"/"rlink2" against our
        Llink2_Link/Rlink2_Link -- leaves the held object colliding with the
        gripper that is holding it, and every later plan fails for no stated
        reason. GRIPPER_LINKS is the correct list for this robot.
        """
        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.touch_links = list(touch_links)
        aco.object.id = object_id
        aco.object.operation = CollisionObject.ADD

        if pose is None:
            # Empty geometry: move_group resolves it against the world object.
            aco.object.header.frame_id = link
        else:
            if object_id not in self._objects:
                raise MoveItError('attach(%r, pose=...) needs the geometry, but '
                                  'that object was not added through this client'
                                  % object_id)
            primitives, _ = self._objects[object_id]
            aco.object.header.frame_id = frame_id or link
            aco.object.primitives = primitives
            aco.object.primitive_poses = [self._pose(*pose)]

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [aco]
        self.log.info('attach(%r) to %s' % (object_id, link))
        return self._apply_scene(scene, 'attach %r' % object_id)

    def detach(self, object_id, link=TOOL_LINK):
        """Release the object back into the world (it stays a collision object).

        Follow with remove() if it should disappear from the scene entirely.
        """
        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.object.id = object_id
        aco.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [aco]
        self.log.info('detach(%r)' % object_id)
        return self._apply_scene(scene, 'detach %r' % object_id)
