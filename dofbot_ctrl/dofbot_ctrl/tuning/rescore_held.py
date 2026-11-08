#!/usr/bin/env python3
# coding: utf-8
"""
Score the held check against saved, labelled wrist-camera frames.

    ros2 run dofbot_ctrl rescore_held                 # every ~/held_frames/stow_*.png
    ros2 run dofbot_ctrl rescore_held a.png b.png     # just these
    # try another crop (fractions x0 y0 x1 y1) without touching vision.py
    ros2 run dofbot_ctrl rescore_held --crop 0.10 0.5 0.90 1.0

THE TRUTH IS IN THE FILE NAME: `held` in it means the can was in the jaws,
`empty` means it was not. Name new frames stow_held_<what>.png or
stow_empty_<what>.png and they are scored on the next run. A name with neither
is shown but not scored.

Each frame goes through vision.look('held', ...), the same call pick_place
makes at stow, so a pass here is a pass there. Frames must be the raw camera
view -- calibrate_view --shot without --detect -- and taken FROM STOW: the
check's crop only means something from there.

Exit status 0 when every labelled frame is answered correctly, 1 otherwise
(UNKNOWN counts as not correct), 2 when there is nothing to score or nanoOWL
cannot be reached.
"""

import argparse
import glob
import os
import sys

from dofbot_ctrl import vision

DEFAULT_DIR = os.path.expanduser('~/held_frames')
LABEL = 'a soda can'
EXPECT = {True: vision.PRESENT, False: vision.ABSENT}


def truth(path):
    """True held, False empty, None unlabelled -- from the file name."""
    name = os.path.basename(path).lower()
    held, empty = 'held' in name, 'empty' in name
    if held == empty:
        return None
    return held


def main(argv=None):
    import cv2

    parser = argparse.ArgumentParser(
        prog='rescore_held', description=__doc__.split('\n\n')[0])
    parser.add_argument('frames', nargs='*',
                        help='PNG files (default %s/stow_*.png)' % DEFAULT_DIR)
    parser.add_argument('--crop', nargs=4, type=float, default=None,
                        metavar=('X0', 'Y0', 'X1', 'Y1'),
                        help='override vision.HELD_CROP, as fractions of the '
                             'frame (now %s)' % (vision.HELD_CROP,))
    parser.add_argument('--url', default=vision.OWL_URL)
    cli = parser.parse_args(argv)

    paths = cli.frames or sorted(glob.glob(os.path.join(DEFAULT_DIR,
                                                        'stow_*.png')))
    if not paths:
        print('no frames: give paths, or save stow_held_*.png / '
              'stow_empty_*.png into %s' % DEFAULT_DIR, file=sys.stderr)
        return 2
    if cli.crop:
        vision.HELD_CROP = tuple(cli.crop)

    # Loading CLIP on the first classification can outlast a check's own
    # timeout, which would score as UNKNOWN for reasons nothing to do with the
    # frame.
    ok, _seconds, why = vision.warm_up(LABEL, url=cli.url)
    if not ok:
        print('nanoOWL: %s' % why, file=sys.stderr)
        return 2

    print('crop %s, alternatives %s, floor %.2f\n'
          % (vision.HELD_CROP, vision.HELD_ALTERNATIVES,
             vision.CLASSIFY_MIN_SCORE))
    right = wrong = unscored = 0
    margins = {True: [], False: []}
    for path in paths:
        frame = cv2.imread(path)
        name = os.path.basename(path)
        if frame is None:
            print('  ??    %-40s could not read' % name)
            unscored += 1
            continue
        sighting = vision.look('held', LABEL, frame, cli.url)
        expected = truth(path)
        if expected is None:
            mark = '  -- '
            unscored += 1
        elif sighting.verdict == EXPECT[expected]:
            mark = '  ok '
            right += 1
            margins[expected].append(sighting.score)
        else:
            mark = 'WRONG'
            wrong += 1
        print('%s %-6s %-40s %-8s %s'
              % (mark, {True: 'held', False: 'empty', None: '?'}[expected],
                 name[:40], sighting.verdict.upper(), sighting.reason))

    print('\n%d right, %d wrong of %d labelled%s'
          % (right, wrong, right + wrong,
             ', %d unscored' % unscored if unscored else ''))
    for expected, scores in margins.items():
        if scores:
            print('  %-5s answered at %.2f..%.2f (floor %.2f)'
                  % ({True: 'held', False: 'empty'}[expected], min(scores),
                     max(scores), vision.CLASSIFY_MIN_SCORE))
    if right + wrong == 0:
        return 2
    return 0 if wrong == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
