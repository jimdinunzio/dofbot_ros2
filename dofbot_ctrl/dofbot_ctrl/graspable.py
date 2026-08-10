#!/usr/bin/env python3
# coding: utf-8
"""
Catalogue of objects the arm knows how to pick.

The can is the first target, not the only one, so object properties live in a
catalogue rather than inline in the pick sequence. The point of the catalogue is
that ONE entry feeds both the collision geometry (scene_objects.py) and the
grasp calculation (pick_place.py). Yahboom's demos hardcode "3 cm block" twice --
once as a grasp height, once as a stacking increment -- which is exactly why
their pipeline cannot be retargeted to a different object without editing code
in two unrelated files.

Bounding dimensions and grasp dimensions are separate fields on purpose. For
many objects the place you grip is not the widest point, and the collision
object still needs the true extents. A soda can is the obvious case: 66 mm
across the body, ~53 mm at the neck.

Known object geometry is legitimately model knowledge, not sensor knowledge. A
355 ml can is 66 x 122 mm, and no camera needs to measure that. When perception
arrives it supplies the POSITION; this file supplies the size, the grasp width
and the grip height.
"""

from dataclasses import dataclass, field

from dofbot_ctrl import gripper

SHAPES = ('cylinder', 'box')


class ObjectError(ValueError):
    """The object is not something this arm can be asked to pick."""


@dataclass(frozen=True)
class GraspableObject:
    """One pickable object.

    name        identifier, also used as the collision-object id in the scene
    shape       'cylinder' (upright, `width` is the diameter) or 'box'
    width       overall bounding width, metres
    height      overall height, metres
    depth       second horizontal extent; boxes only, defaults to `width`
    grasp_width the dimension actually between the jaws, defaults to `width`
    grasp_height height ABOVE THE OBJECT'S BASE at which to close the jaws,
                defaults to mid-height
    symmetric   True if wrist roll does not matter (an upright cylinder), so
                theta5 is free and the approach azimuth alone fixes the grasp
    squeeze     how much narrower than grasp_width to COMMAND, metres, so the
                jaws load against the object instead of resting at contact.
                None (the default) means gripper.DEFAULT_SQUEEZE, resolved at
                grasp time so it follows a profile swap; a number here is a
                grip that has been tuned on THIS object on hardware
    mesh        optional package:// URI of a Z-UP mesh to DRAW the object as.
                Purely cosmetic -- collision still uses `shape` -- and it is
                scaled to the fields above rather than trusted at its modelled
                size, so a mesh cannot silently disagree with the geometry that
                is planned against. Its origin may be anywhere. See
                scene_markers, which rejects a mesh whose proportions say it
                was exported up the wrong axis.
    """

    name: str
    shape: str
    width: float
    height: float
    depth: float = None
    grasp_width: float = None
    grasp_height: float = None
    symmetric: bool = False
    squeeze: float = None
    mesh: str = field(default=None, compare=False)
    note: str = field(default='', compare=False)

    def __post_init__(self):
        if self.shape not in SHAPES:
            raise ObjectError('%s: shape must be one of %r, got %r'
                              % (self.name, SHAPES, self.shape))
        # frozen dataclass: fill the derived defaults through object.__setattr__
        if self.depth is None:
            object.__setattr__(self, 'depth', self.width)
        if self.grasp_width is None:
            object.__setattr__(self, 'grasp_width', self.width)
        if self.grasp_height is None:
            object.__setattr__(self, 'grasp_height', self.height / 2.0)
        if self.shape == 'cylinder' and self.depth != self.width:
            raise ObjectError('%s: a cylinder cannot have depth != width'
                              % self.name)
        if not 0.0 < self.grasp_height < self.height:
            raise ObjectError('%s: grasp_height %.3f is not inside the object'
                              % (self.name, self.grasp_height))
        # squeeze is deliberately NOT defaulted here the way the fields above
        # are. Filling it in at construction would freeze whichever gripper was
        # fitted when this module was imported into an object that outlives the
        # swap, and the whole point of the catalogue is that fits_gripper() and
        # friends stay live questions. None stays None; grip_angle() resolves it.
        if self.squeeze is not None and self.squeeze < 0.0:
            raise ObjectError('%s: squeeze %.4f is negative, which would '
                              'command the jaws WIDER than the object and grip '
                              'nothing' % (self.name, self.squeeze))

    # --------------------------------------------------------------- geometry

    @property
    def radius(self):
        if self.shape != 'cylinder':
            raise ObjectError('%s is a box, not a cylinder' % self.name)
        return self.width / 2.0

    @property
    def box_size(self):
        if self.shape != 'box':
            raise ObjectError('%s is a cylinder, not a box' % self.name)
        return (self.depth, self.width, self.height)

    def centre_offset(self):
        """Vertical distance from the grasp point up to the object's centre.

        Positive means the centre is above the jaws. scene_objects uses this to
        place the object correctly once it is attached to the gripper.
        """
        return self.height / 2.0 - self.grasp_height

    # ------------------------------------------------------------- validation

    def fits_gripper(self):
        return gripper.fits(self.grasp_width)

    def grip_angle(self):
        """The angle to COMMAND to hold this object, radians.

        Here rather than at the call site so a per-object squeeze cannot be
        silently dropped by a caller that forgot to pass it -- the catalogue
        entry is the source of truth for how hard this object is held, the same
        way it already is for how wide it is.

        Do not collision-check the result; see gripper.grip_angle_for.
        """
        return gripper.grip_angle_for(self.grasp_width, self.squeeze)

    def check(self):
        """Raise ObjectError if this gripper cannot take the object.

        Called before any motion is planned, so an impossible grasp is a clear
        message rather than a collision or a stall halfway through a sequence.
        """
        try:
            gripper.jaw_angle_for(self.grasp_width)
        except gripper.GripperError as exc:
            raise ObjectError('%s: %s%s' % (self.name, exc,
                                            '\n  ' + self.note if self.note
                                            else ''))
        return self


# ---------------------------------------------------------------- catalogue

CATALOGUE = {}


def register(obj):
    CATALOGUE[obj.name] = obj
    return obj


# The goal object. NEEDS THE EXTENDED FINGERS, and has been gripped with them on
# hardware. The stock jaws cannot take it at all: the body is wider than they
# open, so the tips strike the wall on approach and knock the can over rather
# than closing around it. There is no squeeze grip to fall back on, and the only
# stock-jaw option is the neck, a narrow band with a few mm of tolerance, which
# is not a reliability budget worth building on.
SODA_CAN = register(GraspableObject(
    name='soda_can',
    shape='cylinder',
    width=0.066,
    height=0.122,
    # THIS IS WHAT WAS ACTUALLY WRONG when the can would not stay in the jaws,
    # and it masked everything else while it stood. At 45 mm the grip was low on
    # the body; at 80 mm -- ABOVE the 61 mm centre of mass, so the can hangs
    # from the jaws rather than balancing on them -- the pick works on hardware.
    #
    # Worth remembering before reaching for the gripper model next time: a grasp
    # that fails is not evidence about jaw geometry until the grip HEIGHT has
    # been ruled out. Both the squeeze below and gripper.BACK_STOP_CLEARANCE
    # were tuned against this fault before it was found, which is why neither
    # of them reads like the fix it was thought to be.
    grasp_height=0.080,
    symmetric=True,
    # Tuned on hardware for THIS object: hold it at Rlink1_Joint = 1.381 rad.
    # That is a lighter squeeze than gripper.DEFAULT_SQUEEZE gives (1.412), so
    # it overrides rather than replaces the default -- nothing else is asked to
    # take the can's number.
    #
    # WHAT IS STORED IS A WIDTH, NOT THE ANGLE, because that is the coordinate
    # the rest of the model works in: grip_angle_for interpolates
    # grasp_width - squeeze through the fitted profile's table, so the number
    # here is 66.0 - 63.0 mm. The consequence is that it is only worth 1.381
    # against the CURRENT _EXTENDED rows -- re-key that table and this has to
    # be re-derived, exactly like DEFAULT_SQUEEZE. test_grasp_model.py pins the
    # resulting angle so a re-key cannot move it quietly.
    squeeze=0.003,

    # Drawn from cokecan.obj, which is modelled 64.3 x 122.7 mm -- close to the
    # 66 x 122 above but not equal to it, which is exactly why scene_markers
    # rescales rather than trusting the file. The 1.8 mm is on the DIAMETER,
    # the dimension the jaws close on, so letting the drawing win would put the
    # picture and the grasp 0.9 mm apart.
    mesh='package://dofbot_description/meshes/cokecan.obj'))

# NEEDS THE STOCK JAWS: it passes straight between the extended fingers, which
# stop too far apart to touch it. That trade runs the other way from the can, so
# between the two profiles the arm handles both -- just never at the same time.
# This is the develop-and-prove object and the one Yahboom's own demos use, so
# any borrowed constant was tuned for it, which is a reason to keep it rather
# than replace it with something wider.
TEST_BLOCK = register(GraspableObject(
    name='test_block',
    shape='box',
    width=0.030,
    height=0.030,
    symmetric=False))


def get(name):
    if name not in CATALOGUE:
        raise ObjectError('unknown object %r; catalogue has %s'
                          % (name, sorted(CATALOGUE)))
    return CATALOGUE[name]
