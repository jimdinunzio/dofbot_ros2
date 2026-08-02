#!/usr/bin/env python3
# coding: utf-8
"""
Generate low-poly collision meshes for the DOFBOT arm links.

    python3 scripts/make_collision_meshes.py            # report only
    python3 scripts/make_collision_meshes.py --write    # write meshes/collision/

WHY
---
The shipped meshes are raw CAD tessellations and the URDF uses them for
<collision> as well as <visual>. That is 1,049,211 triangles across the arm,
with arm4_Link alone at 246,098 against MoveIt's 10,000-VERTEX warning
threshold. The cost is real and was measured, not assumed: move_group takes
~45-60 s to start while FCL builds structures over a million triangles, and
/check_state_validity is slow enough that a 10 s client timeout can fire.

Convex hulls of arm1..arm4 cut ~772k triangles to under 5k.

WHY HULLS RATHER THAN DECIMATION
--------------------------------
Several of these meshes are NOT watertight -- base_link has 1565 non-manifold
edges, arm4 has 118, arm2/arm3 have 39 each. Decimation that preserves topology
fights that; qhull does not care, because it only reads the point cloud. So a
hull is the robust choice here, not merely the cheap one. It also needs nothing
beyond scipy, which is already installed.

A hull is always CONSERVATIVE: it contains the original, so it can never miss a
real collision (verified: max original vertex outside its own hull is ~1e-17 m).
The risk runs the other way -- an over-inflated hull rejects valid poses. That is
why base_link and arm5_Link are excluded, and why the remaining hulls were
measured: mean standoff 4-5 mm, worst case 11-18 mm, all of it bridging servo
cutouts.

WHY NOT REGENERATE FROM THE IGES CAD
------------------------------------
The URDF's link origins were authored against these exact STL files, and
dofbot_kinematics is verified to machine precision against /compute_fk on that
basis. A hull of the existing mesh keeps the link frame trivially -- same
points, same coordinates. Re-tessellating from separately-sourced CAD would
introduce a per-part alignment problem and silently invalidate that. Keep the
IGES for designing parts that bolt on, not for collision geometry.

WHAT IS EXCLUDED, AND WHY
-------------------------
base_link is mostly air inside, so its hull fills 62% of the bounding box. It is
replaced by two primitives inlined in dofbot.urdf, sized from Jim's measurements
of the physical part: a 0.1450 x 0.1200 x 0.0030 mounting-plate box offset
-0.0135 in x, and the r 0.0400 arm base cylinder spanning z 0.0030 .. 0.0828.

The cylinder runs to 0.0828 rather than stopping at the housing shoulder at
0.0800, because the four servo mounting screw heads stand 2.8 mm proud of it
(5 mm across, on a +-0.0249 square). They are well inside r 0.0400, so covering
them costs nothing in radius, and they are at exactly the height arm1_Link
sweeps past.

base_link.STL has since been edited in Fusion -- 18 mm of standoff removed to
match the physical arm, and the face count cut to 31,052 -- so the triangle
figures above describe the STOCK mesh, not what is in the tree now.

arm5_Link was hulled first, then reverted to its full mesh once the result was
inspected assembled in RViz. See the note on LINKS below.

The gripper links keep their full meshes for the same reason.
"""

import argparse
import os
import struct
import sys

import numpy as np
from scipy.spatial import ConvexHull

HERE = os.path.dirname(os.path.abspath(__file__))
MESHES = os.path.join(HERE, '..', 'meshes')
OUT = os.path.join(MESHES, 'collision')

# base_link is deliberately absent -- see the module docstring.
#
# arm5_Link is absent too, and was reverted to its full mesh after Jim inspected
# the hulls assembled in RViz. It is the GRIPPER MOUNT: a U-shaped yoke whose
# concavity is exactly where the fingers sit, so a hull fills the opening. It is
# also the link with the least margin for error, being the one that goes near
# the object. Same reason the gripper links themselves are absent -- hulling a
# finger fills the gap the object is supposed to sit in, so the jaws read as
# permanently closed. arm5 and the fingers want decimation (Fusion's Reduce) or
# hand-placed primitives, and they are being redesigned together anyway.
#
# The hulls that remain are the four links where the shape is convex enough for
# it: worst-case standoff 11-18 mm, all of it bridging servo cutouts, and every
# arm-to-arm pair is already disable_collisions in the SRDF so that inflation
# cannot cause a false self-collision.
LINKS = ('arm1_Link', 'arm2_Link', 'arm3_Link', 'arm4_Link')


def load_stl(path):
    """Binary STL -> (n, 3, 3) float64 array of triangle vertices."""
    with open(path, 'rb') as f:
        header = f.read(84)
        count = struct.unpack('<I', header[80:84])[0]
        raw = np.frombuffer(f.read(50 * count), dtype=np.uint8)
    if raw.size != 50 * count:
        raise ValueError('%s: truncated, expected %d triangles' % (path, count))
    tris = raw.reshape(count, 50)[:, 12:48].copy()
    return tris.view('<f4').reshape(count, 3, 3).astype(np.float64)


def write_stl(path, tris, name=b'dofbot collision hull'):
    """Write a binary STL with outward normals computed per facet."""
    count = len(tris)
    with open(path, 'wb') as f:
        f.write(name.ljust(80, b'\0')[:80])
        f.write(struct.pack('<I', count))
        for tri in tris:
            n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
            norm = np.linalg.norm(n)
            n = n / norm if norm > 0 else np.zeros(3)
            f.write(struct.pack('<3f', *n))
            for v in tri:
                f.write(struct.pack('<3f', *v))
            f.write(b'\0\0')


def hull_of(tris):
    """Convex hull as outward-wound triangles.

    scipy does not guarantee simplex winding, so each facet is flipped to face
    away from the hull centroid. A collision mesh with inconsistent normals is
    accepted by FCL but confuses anything that tries to compute a volume.
    """
    points = tris.reshape(-1, 3)
    hull = ConvexHull(points)
    centre = points[hull.vertices].mean(axis=0)
    faces = []
    for simplex in hull.simplices:
        tri = points[simplex]
        n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        if np.dot(n, tri[0] - centre) < 0:
            tri = tri[[0, 2, 1]]
        faces.append(tri)
    return np.array(faces), hull


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--write', action='store_true',
                    help='write meshes/collision/*.STL (default: report only)')
    args = ap.parse_args()

    if args.write:
        os.makedirs(OUT, exist_ok=True)

    print('%-14s %10s %8s %8s   %s'
          % ('link', 'triangles', 'hull', 'saved', 'hull fills bbox'))
    before = after = 0
    for link in LINKS:
        src = os.path.join(MESHES, link + '.STL')
        if not os.path.exists(src):
            print('%-14s MISSING' % link)
            continue
        tris = load_stl(src)
        faces, hull = hull_of(tris)
        before += len(tris)
        after += len(faces)

        pts = tris.reshape(-1, 3)
        bbox = np.prod(pts.max(axis=0) - pts.min(axis=0))
        print('%-14s %10d %8d %7.1f%%   %5.1f%%'
              % (link, len(tris), len(faces),
                 100.0 * (1 - len(faces) / len(tris)), 100.0 * hull.volume / bbox))

        if args.write:
            write_stl(os.path.join(OUT, link + '.STL'), faces)

    print('%-14s %10d %8d %7.1f%%'
          % ('TOTAL', before, after, 100.0 * (1 - after / before)))
    if args.write:
        print('\nwrote %d meshes to %s' % (len(LINKS), os.path.normpath(OUT)))
    else:
        print('\n(report only -- pass --write to generate)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
