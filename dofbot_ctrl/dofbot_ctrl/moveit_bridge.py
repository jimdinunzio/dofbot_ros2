#!/usr/bin/env python3
# coding: utf-8
"""
Drive the real arm to follow /joint_states -- the bridge that makes MoveIt (or
any /joint_states publisher) move the physical DOFBOT.

MoveIt plans and then executes on its controller, which publishes the joint
positions on /joint_states as the trajectory plays out. This node samples that
stream and writes the positions to the real servos through the calibrated
joint_map, so the hardware mirrors the planned motion.

Design, and why each piece is here:

- Timer sampling, not write-on-message. /joint_states arrives ~50 Hz; writing
  every message would flood the servo bus (each write is 6 addressed packets).
  Instead a timer fires at `rate` Hz, reads the LATEST joint state, and writes
  that. This decouples the write rate from the publish rate and keeps the bus
  calm. Between ticks we just remember the newest target.

- Move time ~= the sample period. Each servo write carries a duration; setting
  it near the tick period lets the servo interpolate smoothly from one sample to
  the next instead of snapping.

- The gripper gets its own move time, in its own packet. A jaw move arrives as
  ONE step: the gripper controller commands a position, mock hardware echoes it
  back in the same cycle, and nothing re-sends it. So the servo runs the whole
  travel at the duration it was handed -- that duration IS the jaw speed, and
  doubling it halves it. The arm cannot be slowed this way: its writes are
  reissued every tick, so each one is cut short by the next.

  Closing and opening get DIFFERENT durations. `grip_time_ms` is slow because a
  close lands on an object; `open_time_ms` is short because an open does not.

  AND THE ARM MUST NOT ADDRESS SERVO 6 WHILE THAT MOVE IS RUNNING. write6
  re-commands all six, so an arm write landing mid-close replaces the slow move
  with a track_time one and the jaws snap shut -- measured, and it is why a
  2000 ms close obeyed a lone action goal and did nothing in a full pick. While
  the move is in flight the arm is written as five individual packets instead.
  moveit_client also waits the duration out before moving on, which is the
  other half: this half keeps the promise even when something else is driving.

- Slow first move (sync). When this node starts, /joint_states reflects MoveIt's
  model pose, which may be far from where the real arm physically is. The first
  write eases the arm there over `sync_time_ms` so it doesn't hard-snap. After
  that, tracking writes use the short `track_time_ms`.

- Skip-if-unchanged. While the pose is constant (idle, or during planning before
  execution) the target doesn't change, so we don't write. The arm only moves
  during actual execution.

- Torque on at startup, so the servos can hold the commanded positions (a prior
  mirror/calibrate session may have left torque off).

- Optional encoder read-back (`encoder_rate`, default 0 = off). This node is
  the only one that may read the servos while it is running, because it owns
  the port -- so if anything is ever to compare what was commanded against what
  the arm did, it has to come from here. Readings go to /servo_states, NEVER to
  /joint_states: a second publisher there makes MoveIt's idea of the robot
  state flicker between two sources.

  APPLIED LIVE, like grip_time_ms: a parameter that is silently not the value
  in use is worse than no parameter. The publisher is always advertised and
  only the timer comes and goes, so `ros2 topic info /servo_states`
  distinguishes "switched off" from "no bridge running".

  OFF by default: reads and writes share one bus, one lock and one
  single-threaded executor, so a ~12ms sweep delays the write tick by that
  much. At the 10Hz tick there is 100ms of budget per cycle.

Owns /dev/ttyTHS1. Stop the mirror, gui_teleop, and calibrate_zero first --
nothing else may hold the port.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState

from Arm_Lib import Arm_Device

from dofbot_ctrl.joint_map import JOINT_NAMES, servo_to_urdf, urdf_to_servo
from dofbot_ctrl.serial_port import rival_warning

# A target within this many servo degrees of the last write counts as unchanged.
_STEP_DEG = 0.5

# Servo id of the gripper, and its slot in a JOINT_NAMES-ordered target.
_GRIP_ID = 6
_GRIP = _GRIP_ID - 1


class MoveItBridge(Node):

    def __init__(self):
        super().__init__('moveit_bridge')
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('rate', 10.0)           # servo write rate (Hz)
        self.declare_parameter('track_time_ms', 200)   # per-step servo move time
        self.declare_parameter('sync_time_ms', 2000)   # first (sync) move time
        # HOW FAST THE JAWS CLOSE, and the ONLY place the number lives. The
        # gripper's whole travel used to arrive as one track_time_ms write, so
        # twice that is half speed. There is no speed to scale anywhere above,
        # because GripperCommand carries a position and no velocity.
        #
        # Deliberately NOT also a launch argument. It was one, defaulting to
        # 400, and a launch default beats the default here -- so editing this
        # line changed nothing on a launched stack and the close stayed fast for
        # a value nobody had typed. One default, in the node that sends it.
        #
        # Read at every write rather than cached, so
        #     ros2 param set /moveit_bridge grip_time_ms 800
        # takes effect on the next close. This is the number that gets tuned
        # against a real object in the jaws, and relaunching to try a value
        # makes that a much slower loop than it needs to be.
        #
        # The servo's field is 12 bits of milliseconds (protocol 0x2A: time_H is
        # 0x00-0x0F), so anything past 4095 is not a slower close -- it is a
        # different move time entirely, wrapped.
        self.declare_parameter('grip_time_ms', 2000)   # jaw CLOSE move time
        # Jaw OPEN move time. Separate from grip_time_ms because the slow close
        # exists to stop the jaws slamming onto an object, and an open has no
        # object to protect -- three of the four seconds a pick spent waiting on
        # the gripper were spent opening onto nothing.
        #
        # Not track_time_ms either: the linkage is over-centre and its travel
        # ends at the open peak, so a full-speed open drives it into that stop.
        # 400 ms is a guess at "brisk but not slamming" and is the one number
        # here to try shorter, live:
        #     ros2 param set /moveit_bridge open_time_ms 250
        self.declare_parameter('open_time_ms', 400)

        # Hz to read the encoders back and publish them on /servo_states.
        # 0 disables it entirely, which is the default: see the docstring for
        # why this competes with the write tick for the bus.
        self.declare_parameter('encoder_rate', 0.0)

        port = self.get_parameter('port').value
        rate = self.get_parameter('rate').value
        self.track_time = int(self.get_parameter('track_time_ms').value)
        self.sync_time = int(self.get_parameter('sync_time_ms').value)
        encoder_rate = float(self.get_parameter('encoder_rate').value)

        self.port = port
        rivals = rival_warning(port)
        if rivals:
            self.get_logger().error(
                'Starting anyway, but expect corrupt traffic.%s' % rivals)

        self.arm = Arm_Device(com=port)

        # Prove the arm is actually there before trusting anything downstream.
        # This node only WRITES, so a powered-off arm produces no symptom at
        # all: RViz animates the planned motion perfectly while the real arm
        # sits dead, which is a genuinely confusing way to lose an hour. One
        # read is enough to tell the difference.
        if not any(self.arm.Arm_serial_servo_read(sid) is not None
                   for sid in range(1, 7)):
            self.get_logger().error(
                'NO REPLY FROM ANY SERVO on %s. The arm is almost certainly '
                'POWERED OFF -- check the switch and the battery. Other causes: '
                'another node holding the port (joint_state_mirror, gui_teleop, '
                'calibrate_zero), or the wrong port. Continuing anyway, but '
                'every command will be written into the void: MoveIt and RViz '
                'will look completely normal while the arm does not move.'
                % port)

        self.arm.Arm_serial_set_torque(1)  # ensure the arm can hold positions

        self.target = None        # latest servo command (deg) from /joint_states
        self.last_written = None
        self.synced = False
        # When the gripper's current move is due to finish (clock ns). Until
        # then servo 6 must not be addressed again -- see tick().
        self.grip_until = 0

        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.write_rate = rate
        self.encoder_rate = 0.0
        self.encoder_timer = None
        # Advertised unconditionally: a topic that exists and is silent is a
        # diagnosable state; one that was never advertised looks like a
        # crashed node.
        self.encoder_pub = self.create_publisher(JointState, '/servo_states', 10)
        self.add_on_set_parameters_callback(self._on_set_params)
        self._set_encoder_rate(encoder_rate)
        self.get_logger().info(
            'Following /joint_states -> servos at %.1f Hz; jaws close over '
            '%d ms and open over %d ms (both live via ros2 param set). The '
            'first move is a slow sync to the model pose -- support the arm.'
            % (rate, self.grip_time(), self.grip_time(opening=True)))

    def grip_time(self, opening=False):
        """The gripper's move time in ms, as it stands right now.

        Two numbers, because closing and opening are not the same job: a close
        lands on an object and a open does not.
        """
        name = 'open_time_ms' if opening else 'grip_time_ms'
        return int(self.get_parameter(name).value)

    def on_js(self, msg):
        """Store the newest pose as a servo command. Ignore partial messages
        (a publisher that doesn't include all six of our joints)."""
        by_name = dict(zip(msg.name, msg.position))
        if all(n in by_name for n in JOINT_NAMES):
            self.target = urdf_to_servo([by_name[n] for n in JOINT_NAMES])

    def tick(self):
        if self.target is None:
            return  # nothing received yet

        if not self.synced:
            self.get_logger().info('Syncing arm to model pose (servo deg): %s'
                                   % [round(a, 1) for a in self.target])
            self.arm.Arm_serial_servo_write6(*self.target, time=self.sync_time)
            self.last_written = self.target
            self.synced = True
            return

        moved = [abs(a - b) > _STEP_DEG
                 for a, b in zip(self.target, self.last_written)]
        if not any(moved):
            return  # unchanged -> don't hammer the bus

        now = self.get_clock().now().nanoseconds

        if moved[_GRIP]:
            # The gripper gets its own packet and its own duration. Logged
            # because this is the write whose duration is tuned by hand, and
            # "did the value I set reach a packet?" is otherwise unanswerable
            # from outside.
            # A low servo value is open (joint_map._GRIP_SERVO_RANGE), so a
            # target below the last one written is the jaws opening.
            opening = self.target[_GRIP] < self.last_written[_GRIP]
            ms = self.grip_time(opening)
            self.get_logger().info('gripper %s -> %.1f servo deg over %d ms'
                                   % ('opening' if opening else 'closing',
                                      self.target[_GRIP], ms))
            self.arm.Arm_serial_servo_write(_GRIP_ID, self.target[_GRIP], ms)
            self.grip_until = now + ms * 1000000

        if any(moved[:_GRIP]):
            if now < self.grip_until:
                # AN ARM MOVE WITH THE JAWS STILL CLOSING, and write6 would
                # ruin it: it addresses all six servos, so it re-commands the
                # gripper -- same position, track_time to get there -- and the
                # servo abandons the slow move for the fast one. THAT is what
                # made grip_time_ms look like it did nothing while the isolated
                # action goal obeyed it perfectly. Five packets instead of six,
                # and servo 6 is left alone to finish.
                for sid in range(1, _GRIP_ID):
                    self.arm.Arm_serial_servo_write(sid, self.target[sid - 1],
                                                    self.track_time)
            else:
                self.arm.Arm_serial_servo_write6(*self.target,
                                                 time=self.track_time)

        self.last_written = self.target

    def _set_encoder_rate(self, hz):
        """Start, restart or stop the encoder timer. Safe to call at any time."""
        if self.encoder_timer is not None:
            self.destroy_timer(self.encoder_timer)
            self.encoder_timer = None
        self.encoder_rate = hz

        if hz <= 0.0:
            self.get_logger().info(
                'Encoder read-back OFF; /servo_states is advertised but '
                'silent. Turn it on with:  ros2 param set /moveit_bridge '
                'encoder_rate 10.0')
            return

        self.encoder_timer = self.create_timer(1.0 / hz, self.read_encoders)
        self.get_logger().info(
            'Encoder read-back ON: /servo_states at %.1f Hz, sharing the bus '
            'with the %.1f Hz write tick.' % (hz, self.write_rate))

        # A sweep is ~12ms and blocks the write tick. Past roughly a third of
        # the cycle, reads delay the writes they are being compared against.
        budget = 0.33 / self.write_rate
        if 1.0 / hz < budget:
            self.get_logger().warn(
                '%.1f Hz leaves under a third of the %.1f Hz write cycle for '
                'writing. Reads will delay the writes they are being compared '
                'against; %.1f Hz or less is safer.'
                % (hz, self.write_rate, 1.0 / budget))

    def _on_set_params(self, params):
        """Apply encoder_rate the moment it is set, not at the next start."""
        for p in params:
            if p.name != 'encoder_rate':
                continue
            hz = float(p.value)
            if hz < 0.0:
                return SetParametersResult(
                    successful=False, reason='encoder_rate cannot be negative')
            self._set_encoder_rate(hz)
        return SetParametersResult(successful=True)

    def read_encoders(self):
        """Sweep the encoders and publish them on /servo_states.

        Floats, not the whole-degree Arm_serial_servo_read the mirror uses: one
        degree is twice the 0.5-servo-degree step this node uses to decide a
        joint moved, so int readings would be mostly quantisation.

        A silent servo publishes nothing for that joint rather than a held or
        zeroed value -- this topic exists to be differenced against
        /joint_states, and a substituted value is a fabricated agreement.
        """
        servo_deg = self.arm.Arm_serial_servo_read6()
        urdf = servo_to_urdf(servo_deg)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name, angle in zip(JOINT_NAMES, urdf):
            if angle is not None:
                msg.name.append(name)
                msg.position.append(angle)
        if not msg.name:
            self.get_logger().warn('no servo answered; nothing to publish',
                                   throttle_duration_sec=5.0)
            return
        self.encoder_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MoveItBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
