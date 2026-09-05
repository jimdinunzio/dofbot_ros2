#!/usr/bin/env python3
# coding: utf-8
"""
Look through the arm camera and see what the detector makes of it.

    # what does the camera see right now, and what would each check say?
    ros2 run dofbot_ctrl calibrate_view -- --detect --shot /tmp/view.png

    # is a miss the wording or the view? One frame, many prompts. If NOTHING
    # fires on something plainly visible in the saved frame, it is the view
    ros2 run dofbot_ctrl calibrate_view -- --shot /tmp/view.png --labels \
        'a soda can' 'a coke can' 'a drink can' 'a red object'

IF YOU CAN SEE FLOOR IN THE SHOT, THE HELD VERDICT IN IT IS MEANINGLESS. That
check is a whole-frame classification and cannot tell a can in the jaws from
one lying behind it -- measured, one in view reads as held at 1.000. 'carry'
points the tool well up and shows no floor, which is what makes the question
answerable there; this tool is usually run from wherever the arm happens to be,
which is usually not that. Read the held line as a comment on the frame in
front of you, not as a verdict about the gripper.

APPROACH_MAX_OFFSET, the one remaining gate, is how far off the image centre
the can being approached may sit. It is unset, and wants a real standoff frame
first: nobody has established where the lens points, and a gate set wrong
aborts picks that would have worked. Every detection here is printed with its
`offset` and drawn on the saved frame, with the one each rule would choose
marked, so setting it is reading a number off a picture.

The camera is opened for a fraction of a second per shot and released. Nothing
else on this machine holds it: nanoOWL runs in network frame-source mode and
opens no camera of its own.
"""

import argparse
import sys

from dofbot_ctrl import graspable, vision

# BGR. The chosen detection is bright; the rest are dim, so which one the rule
# picked is obvious at a glance rather than something to work out from labels.
CHOSEN_COLOUR = (0, 255, 0)
OTHER_COLOUR = (140, 140, 140)
CENTRE_COLOUR = (0, 200, 255)


def draw_centre(image):
    """A cross at the image centre -- what `offset` is measured from."""
    import cv2

    height, width = image.shape[:2]
    cv2.drawMarker(image, (width // 2, height // 2), CENTRE_COLOUR,
                   cv2.MARKER_CROSS, 24, 1)
    return image


def draw_candidate(image, cand, tag, chosen):
    import cv2

    colour = CHOSEN_COLOUR if chosen else OTHER_COLOUR
    x1, y1, x2, y2 = (int(round(v)) for v in cand.box)
    cv2.rectangle(image, (x1, y1), (x2, y2), colour, 2 if chosen else 1)
    cx, cy = vision.centre(cand.box)
    cv2.drawMarker(image, (int(round(cx)), int(round(cy))), colour,
                   cv2.MARKER_CROSS, 16, 2 if chosen else 1)
    cv2.putText(image, '%s w%.2f o%.2f s%.2f'
                % (tag, cand.width, cand.offset, cand.score),
                (x1 + 4, max(12, y1 - 6)), cv2.FONT_HERSHEY_PLAIN, 0.9,
                colour, 1)
    return image


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog='calibrate_view', description=__doc__.split('\n\n')[0])
    parser.add_argument('--shot', default='/tmp/arm_view.png',
                        help='where to write the annotated frame '
                             '(default %(default)s)')
    parser.add_argument('--detect', action='store_true',
                        help='ask nanoOWL what it sees. Without this the tool '
                             'only grabs a frame and reports its brightness')
    parser.add_argument('--object', default='soda_can',
                        help='catalogue entry to look for (default %(default)s)')
    parser.add_argument('--label', default='',
                        help='prompt the detector with this instead of the '
                             'label derived from the catalogue name')
    parser.add_argument('--approach-max-offset', type=float, default=None,
                        metavar='FRACTION',
                        help='try this value for vision.APPROACH_MAX_OFFSET')
    parser.add_argument('--labels', nargs='+', metavar='LABEL',
                        help='try several prompt wordings on the ONE frame and '
                             'print what each returns, then exit. What settles '
                             'whether a miss is the wording or the view: if '
                             'nothing at all fires on an object you can see in '
                             'the saved frame, the camera is the problem, not '
                             'the prompt')
    parser.add_argument('--url', default=vision.OWL_URL,
                        help='nanoOWL XML-RPC server (default %(default)s)')
    cli = parser.parse_args(sys.argv[1:] if argv is None else argv)

    import cv2

    try:
        label = cli.label or vision.label_for(graspable.get(cli.object))
    except graspable.ObjectError as exc:
        print(exc, file=sys.stderr)
        return 2

    # Applied to this process only, so a value can be tried on a real frame
    # before it is written into vision.py -- and judge() below then answers
    # with it, rather than with a number this tool has printed and nothing
    # actually used.
    if cli.approach_max_offset is not None:
        vision.APPROACH_MAX_OFFSET = cli.approach_max_offset

    frame = vision.grab()
    if frame is None:
        print('the arm camera at index %d would not open or would not read'
              % vision.CAMERA_INDEX, file=sys.stderr)
        return 1

    width, height = vision.frame_size(frame)
    print('frame %dx%d, mean pixel %.1f%s'
          % (width, height, frame.mean(),
             '  -- DARK: no useful light reached the sensor, so a live check '
             'would answer UNKNOWN here' if vision.is_dark(frame) else ''))
    print('APPROACH_MAX_OFFSET %s' % vision.APPROACH_MAX_OFFSET)
    for check, usable in sorted(vision.CHECK_USABLE.items()):
        if not usable:
            print('note: the %s check is switched off -- the camera cannot see '
                  'what it asks about. This tool still reports on it, which is '
                  'how you tell when that has been fixed.' % check)

    if cli.labels:
        # One frame, many wordings. Reusing the same frame is the point: a
        # sweep over live frames confounds the wording with whatever moved.
        for label in cli.labels:
            detections, seconds, reason = vision.detect(frame, label,
                                                        url=cli.url)
            if detections is None:
                print('  %-28s could not ask: %s' % (label, reason))
                continue
            cands = vision.candidates(detections, label, (width, height),
                                      floor=0.0)
            print('  %-28s %.2fs  %s'
                  % (label, seconds,
                     '; '.join('%.3f @ %.0f,%.0f %.0f,%.0f'
                               % (c.score, c.box[0], c.box[1],
                                  c.box[2], c.box[3])
                               for c in sorted(cands, key=lambda c: -c.score))
                     or '(nothing)'))
        if not cv2.imwrite(cli.shot, frame):
            print('could not write %s' % cli.shot, file=sys.stderr)
            return 1
        print('wrote %s' % cli.shot)
        return 0

    if cli.detect:
        # The held check first, because it is the one whose answer depends on
        # what is BEHIND the gripper -- and a picture is the only way to see
        # that. It is a classification, so it shares nothing with the detection
        # path below and needs its own inference.
        alternatives = vision.held_alternatives(label)
        index, score, secs, why = vision.classify(frame, alternatives,
                                                  url=cli.url)
        if index is None:
            print('\nheld     could not ask: %s' % why)
        else:
            winner = alternatives[index]
            verdict = ('PRESENT' if index == len(alternatives) - 1
                       else 'ABSENT')
            if score < vision.CLASSIFY_MIN_SCORE:
                verdict = 'UNKNOWN (under the %.2f floor)' % \
                    vision.CLASSIFY_MIN_SCORE
            print('\nheld     %s -- %r at %.3f  (%.2fs)'
                  % (verdict, winner, score, secs))
            print('         reads the WHOLE frame: only meaningful from a '
                  'pose showing no floor, i.e. carry. If this shot has floor '
                  'in it, a can lying there reads as held.')

        # Then the detection path, which is what the approach check uses.
        detections, seconds, reason = vision.detect(frame, label, url=cli.url)
        if detections is None:
            print('could not ask: %s' % reason)
        else:
            cands = vision.candidates(detections, label,
                                      (width, height))
            print('%s in %.2fs, %d detection(s), %d of them %s above score '
                  '%.2f' % (vision._prompt(label), seconds, len(detections),
                            len(cands), label, vision.MIN_SCORE))

            # Detection-based checks only: 'held' is answered above and
            # chooses nothing from this list.
            picks = {'approach': vision.choose('approach', cands)}
            for i, cand in enumerate(sorted(cands, key=lambda c: -c.area)):
                chose = [c for c in picks if picks[c] is cand]
                tag = '/'.join(chose) if chose else '-'
                print('  #%d  width %.3f  offset %.3f  score %.3f  '
                      'box %.0f,%.0f %.0f,%.0f  %s'
                      % (i, cand.width, cand.offset, cand.score,
                         cand.box[0], cand.box[1], cand.box[2], cand.box[3],
                         ('chosen by: ' + tag) if chose else ''))
                draw_candidate(frame, cand, tag, bool(chose))

            print()
            cand = picks['approach']
            if cand is None:
                print('  approach ABSENT -- nothing of that label in frame')
            else:
                present, why = vision.judge('approach', cand)
                print('  approach %s -- %s'
                      % ('PRESENT' if present else 'ABSENT', why))

            if not cands:
                print('  (nothing -- either it is not in view, or %.2f is too '
                      'high a score floor for this scene)' % vision.MIN_SCORE)

    draw_centre(frame)
    if not cv2.imwrite(cli.shot, frame):
        print('could not write %s' % cli.shot, file=sys.stderr)
        return 1
    print('wrote %s' % cli.shot)
    return 0


if __name__ == '__main__':
    sys.exit(main())
