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

- Slow first move (sync). When this node starts, /joint_states reflects MoveIt's
  model pose, which may be far from where the real arm physically is. The first
  write eases the arm there over `sync_time_ms` so it doesn't hard-snap. After
  that, tracking writes use the short `track_time_ms`.

- Skip-if-unchanged. While the pose is constant (idle, or during planning before
  execution) the target doesn't change, so we don't write. The arm only moves
  during actual execution.

- Torque on at startup, so the servos can hold the commanded positions (a prior
  mirror/calibrate session may have left torque off).

Owns /dev/ttyTHS1. Stop the mirror, gui_teleop, and calibrate_zero first --
nothing else may hold the port.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from Arm_Lib import Arm_Device

from dofbot_ctrl.joint_map import JOINT_NAMES, urdf_to_servo

# A target within this many servo degrees of the last write counts as unchanged.
_STEP_DEG = 0.5


class MoveItBridge(Node):

    def __init__(self):
        super().__init__('moveit_bridge')
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('rate', 10.0)           # servo write rate (Hz)
        self.declare_parameter('track_time_ms', 200)   # per-step servo move time
        self.declare_parameter('sync_time_ms', 2000)   # first (sync) move time

        port = self.get_parameter('port').value
        rate = self.get_parameter('rate').value
        self.track_time = int(self.get_parameter('track_time_ms').value)
        self.sync_time = int(self.get_parameter('sync_time_ms').value)

        self.arm = Arm_Device(com=port)
        self.arm.Arm_serial_set_torque(1)  # ensure the arm can hold positions

        self.target = None        # latest servo command (deg) from /joint_states
        self.last_written = None
        self.synced = False

        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            'Following /joint_states -> servos at %.1f Hz. The first move is a '
            'slow sync to the model pose -- support the arm.' % rate)

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

        if self.last_written is not None and all(
                abs(a - b) <= _STEP_DEG
                for a, b in zip(self.target, self.last_written)):
            return  # unchanged -> don't hammer the bus

        self.arm.Arm_serial_servo_write6(*self.target, time=self.track_time)
        self.last_written = self.target


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
