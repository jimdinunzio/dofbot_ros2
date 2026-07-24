#!/usr/bin/env python3
"""Publish the robot chassis and the ground plane as static collision objects.

Path A collision avoidance: the DOFBOT arm is mounted on a plate in FRONT of a
cylindrical mobile-robot body. The URDF meshes only cover the arm + its mounting
plate, so MoveIt has no idea the body exists and will happily plan the arm
straight back (-x) into it. MoveIt likewise has no ground plane of its own, so
without a floor object it happily plans the gripper through the table.

This node injects a cylinder (the chassis) and a box (the floor) into the MoveIt
planning scene (frame: base_link, +x forward, -x toward the chassis). Every
dimension is a ROS parameter so you can tune them live against the real arm,
then bake the proven numbers into the URDF/SRDF for production (Path B).

Frame convention (base_link):
  +x = forward (pick zone)          -x = backward (chassis)
  +z = up                            origin = arm mounting-hole center

Arm reach, from the URDF: the shoulder (arm2) sits at z=0.1255 and the segments
beyond it total 0.0829+0.0829+0.0782+0.0681 = 0.312 m, with arm1 sweeping +-90
deg. So the reachable ground area is a half-disc of radius ~0.31 m in front of
the base axis; the floor defaults cover that with margin.

Params (all metres):
  frame_id     TF frame the objects are fixed to        (default base_link)
  radius       chassis cylinder radius                  (default 0.055)
  height       chassis cylinder height                  (default 0.15)
  back_offset  distance in -x from origin to cyl. axis  (default 0.06)
  z_bottom     bottom of the cylinder, z of base_link   (default 0.0)
  margin       safety inflation added to radius         (default 0.01)
  object_id    collision-object name                    (default robot_chassis)
  period_s     re-assert interval (move_group may (re)start) (default 1.0)

  floor_enable      publish the ground plane box        (default True)
  floor_z_top       top surface of the floor            (default -0.020)
  floor_thickness   box thickness (down from z_top)     (default 0.02)
  floor_x_front     forward (+x) edge of the floor      (default 0.35)
  floor_x_back      rear (-x) edge; default is the      (default -0.105)
                    chassis front face, so the floor
                    spans only in front of the chassis
  floor_half_width  half-extent in +-y                  (default 0.35)
"""

import rclpy
from rclpy.node import Node

from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class ChassisCollision(Node):
    def __init__(self):
        super().__init__('chassis_collision')

        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('radius', 0.055)
        self.declare_parameter('height', 0.15)
        self.declare_parameter('back_offset', 0.06)
        self.declare_parameter('z_bottom', 0.0)
        self.declare_parameter('margin', 0.00)
        self.declare_parameter('object_id', 'robot_chassis')
        self.declare_parameter('period_s', 1.0)

        # Ground plane. Its top sits 20 mm below base_link's plate (which starts
        # at z=0), so it never touches base_link -- no adjacency to whitelist.
        self.declare_parameter('floor_enable', True)
        self.declare_parameter('floor_z_top', -0.020)
        self.declare_parameter('floor_thickness', 0.005)
        self.declare_parameter('floor_x_front', 0.35)
        self.declare_parameter('floor_x_back', -0.105)
        self.declare_parameter('floor_half_width', 0.35)

        # /planning_scene diffs are what move_group merges. Volatile QoS to match
        # move_group's subscriber; we re-publish on a timer so a late/restarted
        # move_group still receives the object.
        self.pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        period = self.get_parameter('period_s').value
        self.timer = self.create_timer(period, self.publish_scene)

        p = self.get_parameter
        self.get_logger().info(
            f"chassis: r={p('radius').value} h={p('height').value} "
            f"back_offset={p('back_offset').value} z_bottom={p('z_bottom').value} "
            f"margin={p('margin').value} frame={p('frame_id').value}")
        if p('floor_enable').value:
            self.get_logger().info(
                f"floor: z_top={p('floor_z_top').value} "
                f"x=[{p('floor_x_back').value}, {p('floor_x_front').value}] "
                f"y=+-{p('floor_half_width').value} "
                f"thickness={p('floor_thickness').value}")
        else:
            self.get_logger().warn('floor_enable=False: no ground plane, so poses '
                                   'below the table will be reported valid')

    def _cylinder(self, radius, z_bottom, z_top, x_center):
        """A z-axis cylinder spanning [z_bottom, z_top], centred at x_center."""
        cyl = SolidPrimitive()
        cyl.type = SolidPrimitive.CYLINDER
        cyl.dimensions = [z_top - z_bottom, radius]  # [height, radius]
        pose = Pose()
        pose.position.x = x_center       # cylinder primitive is centred on its pose
        pose.position.y = 0.0
        pose.position.z = (z_bottom + z_top) / 2.0
        pose.orientation.w = 1.0
        return cyl, pose

    def _box(self, x_min, x_max, y_half, z_min, z_max):
        """An axis-aligned box spanning the given extents."""
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [x_max - x_min, 2.0 * y_half, z_max - z_min]
        pose = Pose()
        pose.position.x = (x_min + x_max) / 2.0   # box is centred on its pose
        pose.position.y = 0.0
        pose.position.z = (z_min + z_max) / 2.0
        pose.orientation.w = 1.0
        return box, pose

    def publish_scene(self):
        g = lambda n: self.get_parameter(n).value

        margin = g('margin')
        z_bottom = g('z_bottom')
        # back_offset is a distance behind the origin; accept either sign
        x_center = -abs(g('back_offset'))

        prims, poses = [], []

        # main body cylinder. The prims/poses list is kept multi-primitive on
        # purpose: the finer chassis features (wider bottom plate, side wheels,
        # front bumpers) would be added here later as extra primitives -- two
        # cylinders for the wheels, small boxes for the bumpers, a flat box for
        # the plate -- if the arm ever proves to reach into those low/side zones.
        cyl, pose = self._cylinder(
            g('radius') + margin, z_bottom, z_bottom + g('height'), x_center)
        prims.append(cyl)
        poses.append(pose)

        # ground plane: MoveIt has no floor of its own, so without this a pose
        # below the table collides with nothing and is reported valid.
        if g('floor_enable'):
            z_top = g('floor_z_top')
            box, pose = self._box(
                g('floor_x_back'), g('floor_x_front'), g('floor_half_width'),
                z_top - g('floor_thickness'), z_top)
            prims.append(box)
            poses.append(pose)

        obj = CollisionObject()
        obj.header.frame_id = g('frame_id')
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = g('object_id')
        obj.primitives = prims
        obj.primitive_poses = poses
        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        self.pub.publish(scene)


def main():
    rclpy.init()
    node = ChassisCollision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
