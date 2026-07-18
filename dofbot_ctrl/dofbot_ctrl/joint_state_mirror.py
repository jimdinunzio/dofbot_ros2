#!/usr/bin/env python3
# coding: utf-8
"""
Mirror the physical arm into /joint_states. Read-only: this node never
commands motion.

Owns the serial port via Arm_Lib. Nothing else may hold /dev/ttyTHS1 while this
runs -- pyserial takes no exclusive lock, so a second owner (e.g. arm-service)
interleaves bytes on the bus and corrupts reads silently rather than failing.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from Arm_Lib import Arm_Device

from dofbot_ctrl.joint_map import JOINT_NAMES, servo_to_urdf


class JointStateMirror(Node):

    def __init__(self):
        super().__init__('joint_state_mirror')
        self.declare_parameter('port', '/dev/ttyTHS1')
        # Each servo read blocks for the driver's serial timeout, so a full
        # six-servo sweep costs ~120ms; rates much above ~8Hz cannot keep up.
        self.declare_parameter('rate', 5.0)
        self.declare_parameter('release_torque', False)

        port = self.get_parameter('port').value
        rate = self.get_parameter('rate').value
        self.arm = Arm_Device(com=port)
        self.last_good = [None] * 6
        self.torque_released = False

        if self.get_parameter('release_torque').value:
            self.get_logger().warn(
                'Releasing torque: the arm goes limp and will fall under its '
                'own weight. Support it before moving it by hand.')
            self.arm.Arm_serial_set_torque(0)
            self.torque_released = True

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            'Mirroring %s -> /joint_states at %.1f Hz' % (port, rate))

    def tick(self):
        servo_deg = [self.arm.Arm_serial_servo_read(sid)
                     for sid in range(1, 7)]

        # A servo that drops a reply reads None. Hold its last good value
        # rather than publishing a spurious jump to zero.
        for i, angle in enumerate(servo_deg):
            if angle is None:
                servo_deg[i] = self.last_good[i]
            else:
                self.last_good[i] = angle

        if any(angle is None for angle in servo_deg):
            self.get_logger().warn(
                'no reading yet from servo(s) %s; not publishing'
                % [i + 1 for i, a in enumerate(servo_deg) if a is None],
                throttle_duration_sec=5.0)
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = servo_to_urdf(servo_deg)
        self.pub.publish(msg)

    def restore_torque(self):
        if self.torque_released:
            try:
                self.arm.Arm_serial_set_torque(1)
                self.get_logger().info('Torque restored.')
            except Exception as exc:
                self.get_logger().error('could not restore torque: %s' % exc)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMirror()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_torque()
        node.destroy_node()
        # On Ctrl-C rclpy's signal handler may already have shut the context
        # down; calling shutdown again raises. Guard it.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
