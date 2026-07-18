#!/usr/bin/env python3
# coding: utf-8
"""
Drive the real arm from the RViz joint_state_publisher GUI sliders, starting
from the arm's current pose (no jump on startup).

Flow:
  1. Read the six servos once. Republish that pose on ~/seed_joint_states (which
     jsp-gui has in its source_list) until jsp-gui echoes it back on
     /joint_states -- i.e. until its sliders actually match. Republishing is
     necessary because a single latched seed can arrive before jsp-gui has the
     robot_description, in which case jsp-gui drops it and centers to defaults.
  2. Once the sliders match the seed, stop seeding, enable writes, and from then
     on write each commanded /joint_states pose to the servos via the calibrated
     urdf_to_servo mapping.

The match check doubles as a safety gate: jsp-gui publishes its default (zero)
pose before it applies the seed, and writing that would slam the arm to zero.
Writes stay disabled until the sliders provably hold the hardware pose.

This node owns /dev/ttyTHS1 (read for the seed, then writes). Nothing else may
hold the port while it runs -- not the mirror, not arm-service.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import JointState

from Arm_Lib import Arm_Device

from dofbot_ctrl.joint_map import JOINT_NAMES, servo_to_urdf, urdf_to_servo

# Arming tolerance (radians): how close jsp-gui's echo must be to the seed to
# count as "sliders seeded". Generous -- the distinguishing joint (gripper) is
# ~1.57 rad away when un-seeded, so this only needs to clear jsp-gui's
# limit-clamping and float noise.
_MATCH_RAD = 0.035  # ~2 deg
# A command is "unchanged" (skip the write) within this many servo degrees.
_STEP_DEG = 1.0
_CENTER = 90.0  # fallback for a servo that never replied during seeding


class GuiTeleop(Node):

    def __init__(self):
        super().__init__('gui_teleop')
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('move_time_ms', 500)
        self.declare_parameter('seed_topic', 'seed_joint_states')

        port = self.get_parameter('port').value
        self.move_time = int(self.get_parameter('move_time_ms').value)
        self.arm = Arm_Device(com=port)

        self.seed_servo = self._read_seed()
        self.seed_urdf = servo_to_urdf(self.seed_servo)  # compare in this space
        self.armed = False
        self.last_written = None

        latched = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.seed_pub = self.create_publisher(
            JointState, self.get_parameter('seed_topic').value, latched)
        # Republish until armed, to survive jsp-gui coming up after us.
        self.seed_timer = self.create_timer(0.5, self._publish_seed)
        self._publish_seed()

        self.create_subscription(JointState, '/joint_states', self.on_cmd, 10)
        self.get_logger().info(
            'Seeding jsp-gui to the hardware pose; writes stay disabled until '
            'the sliders match. Then move a slider to drive the arm.')

    def _read_seed(self):
        for _ in range(5):
            servo = [self.arm.Arm_serial_servo_read(sid) for sid in range(1, 7)]
            if all(v is not None for v in servo):
                self.get_logger().info('Seed pose (servo deg): %s' % servo)
                return [float(v) for v in servo]
        missing = [i + 1 for i, v in enumerate(servo) if v is None]
        self.get_logger().warn(
            'servo(s) %s did not reply; seeding those to %d deg'
            % (missing, int(_CENTER)))
        return [float(v) if v is not None else _CENTER for v in servo]

    def _publish_seed(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = self.seed_urdf
        self.seed_pub.publish(msg)

    def on_cmd(self, msg):
        by_name = dict(zip(msg.name, msg.position))
        if not all(n in by_name for n in JOINT_NAMES):
            return
        pose = [by_name[n] for n in JOINT_NAMES]

        if not self.armed:
            # Arm only once jsp-gui's sliders provably hold the seed.
            if all(abs(p - s) <= _MATCH_RAD for p, s in zip(pose, self.seed_urdf)):
                self.armed = True
                self.seed_timer.cancel()
                self.last_written = urdf_to_servo(pose)
                self.get_logger().info('Sliders seeded; writes enabled.')
            return

        servo = urdf_to_servo(pose)
        if all(abs(a - b) <= _STEP_DEG
               for a, b in zip(servo, self.last_written)):
            return
        self.last_written = servo
        self.arm.Arm_serial_servo_write6(*servo, time=self.move_time)


def main(args=None):
    rclpy.init(args=args)
    node = GuiTeleop()
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
