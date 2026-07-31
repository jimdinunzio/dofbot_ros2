#!/usr/bin/env python3
# coding: utf-8
"""
Turn a GraspableObject into planning-scene geometry, and work out where it sits
once the gripper is holding it.

Everything here reads its dimensions from the catalogue entry, so the collision
object, the grasp point and the attached-object pose cannot drift apart.

POSITION CONVENTION
-------------------
An object's position is the centre of its bounding volume, in base_link. That is
the seam perception will meet: a depth camera returns a point on the near
SURFACE, and the caller is responsible for stepping along the view ray by the
object radius to reach the centre before calling pick(). Doing it there, once
and explicitly, is better than the vendor's blind +0.02/+0.01/+0.01 constants,
which partly encode this correction and partly encode calibration error.
"""

from math import cos, sin, sqrt

from dofbot_ctrl import dofbot_kinematics as kin
from dofbot_ctrl.moveit_client import WRIST_LINK

# arm5_Link -> Gripping_point_Link, in the arm5 frame. Read out of the FK rather
# than retyped from the URDF: with every joint at zero all frames are aligned
# with base_link, so the difference between the two tips IS that offset vector.
TOOL_OFFSET = tuple(
    a - b for a, b in zip(kin.fk([0.0] * 5, 'tcp')[:3],
                          kin.fk([0.0] * 5, 'arm5')[:3]))


def add(client, obj, x, y, z, object_id=None):
    """Put `obj` into the planning scene with its centre at (x, y, z)."""
    object_id = object_id or obj.name
    if obj.shape == 'cylinder':
        return client.add_cylinder(object_id, x, y, z, obj.height, obj.radius)
    return client.add_box(object_id, x, y, z, obj.box_size)


def grasp_point(obj, x, y, z):
    """Where on the object the jaws should close, for a centre at (x, y, z).

    This is pure object geometry: the jaws close at `grasp_height` above the
    object's base, which for a tall object is well below its centre. Yahboom's
    demos never need this because every demo object is the same 3 cm block, so
    grasp height and half-height coincide -- which is precisely why a 122 mm can
    breaks their pipeline.

    Note this is the CONTACT point, not the TCP target. The two differ by the
    tip offset (see back_off and gripper.tip_offset_for): the fingers reach past
    Gripping_point_Link, so the TCP has to sit back from the contact point.
    """
    return (x, y, z - obj.centre_offset())


def back_off(point, phi, distance):
    """Move `distance` back along the tool axis from `point`, toward the wrist.

    The tool axis is tilted phi from vertical and lies in the theta1 half-plane,
    whose bearing the target itself sets. Used twice: for the pre-grasp standoff,
    and for the hover that accounts for the fingers reaching past the TCP frame.
    """
    px, py, pz = point[:3]
    rho = sqrt(px * px + py * py)
    ux, uy = (1.0, 0.0) if rho < 1e-9 else (px / rho, py / rho)
    return (px - distance * sin(phi) * ux,
            py - distance * sin(phi) * uy,
            pz - distance * cos(phi))


def standoff_point(obj, x, y, z, phi, distance):
    """A pre-grasp TCP pose `distance` back along the tool axis from the grasp.

    Backing off along the approach direction (rather than straight up) is what
    makes the final move a pure translation the jaws can make without sweeping
    sideways into the object.
    """
    return back_off(grasp_point(obj, x, y, z), phi, distance)


def _shortest_arc(target):
    """Quaternion (x, y, z, w) rotating +z onto the unit vector `target`."""
    tx, ty, tz = target
    dot = tz                      # (0, 0, 1) . target
    if dot > 1.0 - 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    if dot < -1.0 + 1e-12:
        return (1.0, 0.0, 0.0, 0.0)          # 180 deg about x
    # cross((0,0,1), target) = (-ty, tx, 0)
    qx, qy, qz, qw = -ty, tx, 0.0, 1.0 + dot
    n = sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return (qx / n, qy / n, qz / n, qw / n)


def held_pose(obj, phi, roll=0.0, tip_offset=0.0):
    """Where `obj` sits in the arm5_Link frame once the jaws have closed on it.

    Returns (x, y, z, qx, qy, qz, qw), ready for DofbotMoveIt.attach(pose=...).

    `tip_offset` is how far past Gripping_point_Link the jaws actually grip,
    along the tool axis. The object is held at the CONTACT point, not at the TCP
    frame, and the two are not the same place.

    Derivation. The object was standing upright when it was grasped, so its own
    axis is world-vertical, and the gripper has since carried it rigidly. World
    +z, expressed in arm5_Link, is

        up = (-sin(phi)*cos(roll), sin(phi)*sin(roll), cos(phi))

    since arm4 is base rotated by Ry(phi) about the pitch axis and arm5 adds
    Rz(roll). The jaws hold the object at TOOL_OFFSET plus `tip_offset` along
    the tool axis, and its centre is centre_offset() further along `up`.

    This generalises the recipe in Yahboom's set_scene.cpp rather than copying
    it. Theirs puts a 0.03-tall cylinder at y = 0.075 in Arm5_Link so its far
    face lands on their TCP at y = 0.09 -- correct only for their frame (+y is
    the tool axis on the Pro; ours is +z), their TCP distance, and an object
    gripped at its very top with its axis along the tool. Put grasp_height =
    height and phi = 0 into the formula below and you get exactly their
    z = TOOL_OFFSET_z - height/2. Every other case needs the general form.

    Limitation, deliberate: for a box the rotation about `up` is left at
    whatever the shortest-arc quaternion produces rather than being aligned to
    the approach azimuth. For a symmetric cylinder that is exact; for a small
    cube the residual yaw is immaterial to collision checking. Fix it here if a
    long or flat object ever needs picking.
    """
    up = (-sin(phi) * cos(roll), sin(phi) * sin(roll), cos(phi))
    contact = (TOOL_OFFSET[0], TOOL_OFFSET[1], TOOL_OFFSET[2] + tip_offset)
    d = obj.centre_offset()
    return (contact[0] + d * up[0],
            contact[1] + d * up[1],
            contact[2] + d * up[2]) + _shortest_arc(up)


def attach(client, obj, phi, roll=0.0, tip_offset=0.0, object_id=None):
    """Attach `obj` to the gripper, placed by held_pose in the arm5_Link frame."""
    return client.attach(object_id or obj.name,
                         pose=held_pose(obj, phi, roll, tip_offset),
                         frame_id=WRIST_LINK)
