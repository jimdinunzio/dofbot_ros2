#!/usr/bin/env python3
# coding: utf-8
"""
Draw a catalogue object in RViz as its real textured mesh.

This is VISUAL ONLY. The planning scene still gets the primitive from
scene_objects.add() -- a cylinder for the can -- and that is deliberate, not a
shortcut. FCL builds a BVH over every triangle of a collision mesh and
/check_state_validity is called once per swept pitch candidate and once per
Cartesian waypoint, so a mesh in the planning scene buys a prettier collision
model at a cost measured in seconds per pick. A soda can IS a cylinder to
within a millimetre; there is nothing to buy.

What a mesh does buy is the ability to see, at a glance, whether the jaws are
where you think they are, which a featureless grey cylinder does not tell you.
So the mesh is a Marker on its own topic, drawn at exactly the pose the
collision primitive occupies.

ONE SOURCE OF TRUTH FOR SIZE
----------------------------
The marker is NOT drawn at whatever size the mesh was modelled at. Its scale is
computed from the catalogue entry's width and height divided by the mesh's own
bounding box, so the visual can never disagree with the collision cylinder or
the grasp calculation. Change GraspableObject.width and the drawing follows.
That is the same rule graspable.py already applies to everything else, and it
is why the fitting is done here at runtime rather than baked into the mesh file
by an export step nobody re-runs.

WHATEVER FRAME THE MESH WAS MODELLED IN
---------------------------------------
The mesh must be Z-UP and CENTRED ON ITS OWN ORIGIN. Both are checked (see
check_frame) and neither is corrected, which is a deliberate reversal of how
this module started out.

It began by absorbing both, because the asset arrived Y-up and sitting a metre
off its origin and that seemed cheaper than a round trip to the modeller. It is
not. Compensation is permanent complexity bought to avoid a one-minute fix, and
it hides the mistake rather than reporting it: an absorbed rotation draws a
plausible can that is quietly wrong, and there is nothing on screen to say
which factor flipped. Both faults were fixed in Blender in about a minute, and
deleting the code that had been carrying them removed a quaternion product, a
vector rotation and a scaled offset subtraction -- every remaining line of
which could only ever have been wrong.

So the fit now carries exactly one thing:

    scale       catalogue size / mesh bounding box

and pose_for is a straight copy of the pose scene_objects already computed.

The rule this leaves behind: when an asset is wrong, fix the asset. Check it
here and fail loudly, naming the export step that was missed.

FOLLOWING THE GRIPPER
---------------------
Once the object is attached, the marker is republished in arm5_Link at
scene_objects.held_pose() -- the same pose the attached collision object is
given -- with frame_locked set, so RViz re-transforms it from live tf every
frame and the mesh tracks the arm without this module publishing again.

frame_locked is load-bearing. RViz transforms an ordinary marker once, on
arrival, and caches the result; the can would therefore re-anchor to arm5_Link
at the moment of the grasp, when the gripper is down at the can, and then stay
on the ground while the arm carried the collision cylinder off without it.

LATE SUBSCRIBERS
----------------
The publisher is TRANSIENT_LOCAL, so an RViz started after the pick began still
gets the current markers. The matching display in dofbot_moveit/config/
moveit.rviz sets the same durability; a display left at the default VOLATILE is
compatible but will show nothing until the next state change.
"""

import os

from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker

from dofbot_ctrl import scene_objects
from dofbot_ctrl.moveit_client import BASE_FRAME, WRIST_LINK

MARKER_TOPIC = '/graspable_objects'
NAMESPACE = 'graspable'

# How far the mesh's proportions may differ from the catalogue's before the
# export is called wrong rather than merely imprecise. A Y-up export of an
# upright object is off by the square of its aspect ratio -- 3.4x for the can --
# so this catches the real mistake with room to spare for modelling slop, which
# on cokecan.obj is 3%.
ASPECT_TOLERANCE = 0.25

# How far the mesh's bounding-box centre may sit from its own origin, as a
# fraction of the object's largest dimension. Generous: the failure this guards
# against is an object left where it happened to sit in the modelling scene,
# which is metres, while a correctly centred export lands within microns.
# cokecan.obj's worst axis is 44 um against the 12 mm allowed here.
CENTRE_TOLERANCE = 0.10

# Parsing a mesh costs a file read, and the same object is shown and re-shown
# through a pick. Keyed by resolved path.
_BBOX_CACHE = {}


class MeshFrameError(ValueError):
    """The mesh is not in the frame this module requires."""


def resolve(uri):
    """package://pkg/rest -> an absolute path under the package's share dir."""
    if not uri.startswith('package://'):
        return uri
    package, _, rest = uri[len('package://'):].partition('/')
    return os.path.join(get_package_share_directory(package), rest)


def bounding_box(uri):
    """((min_x, min_y, min_z), (max_x, max_y, max_z)) of an OBJ, in mesh units.

    Reads the `v` lines and nothing else, which is all the fit needs. Doing this
    at startup rather than hardcoding the numbers is what lets a re-exported
    mesh -- rescaled, recentred, re-axised -- drop in without a code change.
    """
    path = resolve(uri)
    if path not in _BBOX_CACHE:
        lo = [float('inf')] * 3
        hi = [float('-inf')] * 3
        with open(path) as mesh:
            for line in mesh:
                if line[:2] == 'v ':
                    for i, value in enumerate(line.split()[1:4]):
                        value = float(value)
                        lo[i] = min(lo[i], value)
                        hi[i] = max(hi[i], value)
        if lo[0] > hi[0]:
            raise ValueError('%s has no vertices' % path)
        _BBOX_CACHE[path] = (tuple(lo), tuple(hi))
    return _BBOX_CACHE[path]


def check_frame(obj, lo, hi):
    """Raise unless the mesh is Z-up and centred on its own origin.

    Both halves are checks rather than corrections, and that is the whole
    design. Compensating for a mis-exported mesh is permanent complexity paying
    for a one-minute fix in the modeller, and it hides the mistake: a rotation
    absorbed here draws a plausible-looking can that is subtly wrong, where a
    refusal names the problem. Fixing the export costs one checkbox.

    ORIENTATION is tested by comparing ASPECT RATIOS rather than asserting "z is
    the longest axis", which is what makes it work for any object: a tall can
    and a flat coaster both have a mesh aspect that should match their catalogue
    aspect, and an axis swap inverts it either way. For a cube-proportioned
    object the test degenerates to always passing, which is correct -- there is
    nothing to detect.

    ORIGIN is tested against the object's own size, not an absolute distance, so
    the tolerance means the same thing for a can and for a pallet.
    """
    extent = [h - l for h, l in zip(hi, lo)]
    span = max(extent[0], extent[1])
    if span <= 0.0 or extent[2] <= 0.0:
        raise MeshFrameError('%s: degenerate mesh bounding box %r'
                             % (obj.name, extent))

    mesh_aspect = extent[2] / span
    want_aspect = obj.height / obj.width
    if abs(mesh_aspect - want_aspect) > ASPECT_TOLERANCE * want_aspect:
        raise MeshFrameError(
            '%s: mesh %s is %.1f x %.1f x %.1f mm, aspect %.2f, but the '
            'catalogue says %.2f (%.0f x %.0f mm). Almost always this means '
            'the mesh was exported Y-up: re-export it Z-up. If the model is '
            'simply the wrong shape, fix the model -- this module rescales a '
            'mesh but will not restretch one.'
            % (obj.name, os.path.basename(obj.mesh),
               extent[0] * 1e3, extent[1] * 1e3, extent[2] * 1e3,
               mesh_aspect, want_aspect, obj.width * 1e3, obj.height * 1e3))

    centre = [(h + l) / 2.0 for h, l in zip(hi, lo)]
    allowed = CENTRE_TOLERANCE * max(obj.width, obj.height)
    if max(abs(c) for c in centre) > allowed:
        raise MeshFrameError(
            '%s: mesh %s has its bounding-box centre at (%.1f, %.1f, %.1f) mm '
            'rather than on its own origin, which is more than the %.1f mm '
            'allowed. In Blender: Object > Set Origin > Origin to Geometry '
            '(Center: Bounds Center), then Object > Clear > Location AND '
            'Object > Clear > Delta Location -- clearing Location alone leaves '
            'a delta transform behind, which does not show in the N panel.'
            % (obj.name, os.path.basename(obj.mesh),
               centre[0] * 1e3, centre[1] * 1e3, centre[2] * 1e3,
               allowed * 1e3))


def fit(obj):
    """Per-axis multiplier drawing `obj`'s mesh at CATALOGUE size, for
    Marker.scale.

    x and y are scaled by the same factor -- the larger of the two is taken as
    the diameter -- so a can stays round. Only z is scaled independently,
    because a catalogue entry gives width and height separately and a model's
    proportions need not match them exactly. cokecan.obj is 64.3 x 122.7 mm
    against a catalogue 66 x 122, so it is stretched 2.8% across and squashed
    0.6% tall.

    That the catalogue wins is the point, not a compromise. Everything that
    MOVES THE ARM -- the collision cylinder, jaw_angle_for, tip_offset_for,
    grasp_point, held_pose -- reads obj.width. Draw the mesh at its own size
    and RViz would show the fingertips a millimetre clear of the can at the
    exact moment the planner believed they were touching it, which is a wrong
    answer to the only question the picture exists to answer. If the model is
    the better measurement, change the catalogue entry: that number also sets
    the angle the gripper servo is commanded to, so it has to move too.
    """
    lo, hi = bounding_box(obj.mesh)
    check_frame(obj, lo, hi)
    extent = [h - l for h, l in zip(hi, lo)]
    across = obj.width / max(extent[0], extent[1])
    return (across, across, obj.height / extent[2])


def pose_for(obj, centre_pose):
    """Marker (position, orientation, scale) drawing `obj` at a centre pose.

    `centre_pose` is (x, y, z) or (x, y, z, qx, qy, qz, qw) for the object's
    CENTRE -- the same convention scene_objects uses throughout, so this takes
    the output of held_pose() unchanged.

    It is a straight copy because check_frame has already insisted the mesh is
    Z-up and centred on its own origin, which makes the object's centre pose and
    the marker pose the same thing. Everything this function used to do -- a
    fix-up quaternion, a rotated bounding-box-centre subtraction -- was
    compensating for an export that has since been corrected.
    """
    scale = fit(obj)
    position = tuple(centre_pose[:3])
    orientation = (tuple(centre_pose[3:7]) if len(centre_pose) >= 7
                   else (0.0, 0.0, 0.0, 1.0))
    return position, orientation, scale


class MeshMarkers:
    """Publishes one mesh marker per catalogue object shown.

    Objects without a `mesh` are silently skipped, so a caller can call these
    unconditionally: test_block has no model and simply does not draw.
    """

    def __init__(self, node, topic=MARKER_TOPIC):
        self.node = node
        # Transient local so a late RViz still sees the can. Depth has to cover
        # every object that might be on screen at once, not just one.
        self.pub = node.create_publisher(
            Marker, topic,
            QoSProfile(depth=10,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._ids = {}

    def _id(self, name):
        return self._ids.setdefault(name, len(self._ids))

    def _marker(self, obj, action=Marker.ADD):
        marker = Marker()
        marker.ns = NAMESPACE
        marker.id = self._id(obj.name)
        marker.type = Marker.MESH_RESOURCE
        marker.action = action
        # frame_locked is what makes an attached object FOLLOW THE GRIPPER, and
        # it is not optional. Without it RViz transforms a marker into the fixed
        # frame ONCE, when the message arrives, and caches the result. The can
        # would re-anchor to arm5_Link at the instant of the grasp -- which is
        # the instant the gripper is down at the can -- and then sit there on
        # the ground while the arm lifted the collision cylinder away without
        # it. Set, RViz re-transforms every frame from live tf.
        #
        # header.stamp is left at its default zero alongside it, so the one
        # transform a non-frame-locked marker would get uses the latest tf
        # rather than tf at publish time. That is necessary but, as above, not
        # sufficient.
        marker.frame_locked = True
        marker.lifetime = Duration().to_msg()       # zero: until deleted
        return marker

    def _publish(self, obj, frame_id, centre_pose):
        if not obj.mesh:
            return None
        position, orientation, scale = pose_for(obj, centre_pose)

        marker = self._marker(obj)
        marker.header.frame_id = frame_id
        marker.pose.position.x, marker.pose.position.y, \
            marker.pose.position.z = position
        (marker.pose.orientation.x, marker.pose.orientation.y,
         marker.pose.orientation.z, marker.pose.orientation.w) = orientation
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.mesh_resource = obj.mesh
        marker.mesh_use_embedded_materials = True
        # A white tint leaves an embedded texture exactly as it is, and leaves a
        # mesh whose texture failed to load visible rather than black.
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        self.pub.publish(marker)
        return marker

    def show(self, obj, x, y, z, frame_id=BASE_FRAME):
        """Draw `obj` standing in the world with its centre at (x, y, z)."""
        return self._publish(obj, frame_id, (x, y, z))

    def hold(self, obj, phi, roll=0.0, tip_offset=0.0):
        """Re-anchor `obj` to the gripper, where scene_objects.attach put it."""
        return self._publish(obj, WRIST_LINK,
                             scene_objects.held_pose(obj, phi, roll,
                                                     tip_offset))

    def hide(self, obj):
        """Remove `obj`'s marker."""
        if not obj.mesh:
            return None
        marker = self._marker(obj, Marker.DELETE)
        marker.header.frame_id = BASE_FRAME
        self.pub.publish(marker)
        return marker
