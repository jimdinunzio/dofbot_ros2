#!/usr/bin/env python3
# coding: utf-8
"""
What the arm camera sees, asked of nanoOWL and answered in three values.

The pick sequence is otherwise blind: it is told where the object is, plans a
grasp to that coordinate and closes the jaws whether or not anything is there.
This module is the eye that confirms it -- once at the pre-grasp standoff
("is the object actually where I was told?") and once at carry ("did it come
with me?").

THREE VALUES, NOT TWO
---------------------
Every check answers PRESENT, ABSENT or UNKNOWN, and the third one is the point.

    PRESENT   the object was seen where it should be
    ABSENT    the view was good and the object was NOT there
    UNKNOWN   the question could not be asked

UNKNOWN covers nanoOWL being down and the camera returning a black frame. It is
deliberately not folded into ABSENT: an unanswerable question is not a negative
answer, and treating it as one would abort every pick on a machine where the
GPU service happens to be running something else. Callers act on ABSENT and log
UNKNOWN. Vision is a confirmation added to the pick, never a new dependency
of it.

WHICH DETECTION IS THE ONE THAT MATTERS
---------------------------------------
The detector is open-vocabulary and returns every can it can see, so each check
first has to say which of them it is talking about. Neither rule needs the
camera to be calibrated, and that is why neither is a pixel rectangle:

    held      THE LARGEST BOX. The can in the jaws is nearer the lens than
              anything else in the room, and apparent size goes as 1/distance,
              so it subtends the biggest angle. Nothing else can outgrow it
              without being closer than the fingertips.
    approach  THE BOX NEAREST THE IMAGE CENTRE. At the standoff the arm is
              pointed at the target, so the can it is about to grasp is the one
              the camera is looking at; other cans on the floor sit off to the
              sides.

SELECTING IS NOT THE SAME AS PRESENCE, which is the one place these rules need
help. Both always pick something whenever anything is detected, so on their own
they answer "yes" to a can lying on the floor a metre away. The default
presence test is therefore the weakest one that cannot be wrong: IS THERE A CAN
IN VIEW AT ALL? That already catches what the checks are mostly for -- an
approach to coordinates with nothing at them, and a can that has gone by the
time the arm reaches carry -- and it needs no measurement, so both checks work
as shipped.

The two gates below sharpen that, and each is ONE NUMBER off ONE FRAME rather
than a calibration. Both default to off, and both fail toward PRESENT, because
a missed drop costs a wasted place and a false abort costs a pick that would
have worked.

WHAT THE CAMERA CAN ACTUALLY SEE -- AND THE JAWS ARE NOT IN IT
--------------------------------------------------------------
THE WRIST CAMERA CANNOT SEE ITS OWN GRIP POINT. Measured on 2026-09-05 with a
can in the jaws: only the lid and the top centimetre or two of the can reach
the bottom edge of the frame, no part of the gripper is visible at all, and
nanoOWL detects nothing for any wording of "can" -- 'a soda can', 'a coke can',
'a drink can', 'a beverage can', 'the top of a soda can' all return nothing.
Only 'a red object' fires, at 0.10, on the sliver that is visible.

The URDF says why, and the arithmetic is in the joint origins. In arm4_Link's
frame, z is the tool axis:

    Camera_Joint      arm4 -> Camera_Link       xyz -0.0481   0  0.0707
    arm5_Joint        arm4 -> arm5_Link         xyz -0.00215  0  0.078149
    Gripping_Joint    arm5 -> Gripping_point    z   0.068091

so from the camera the grip point lies 43.3 mm SIDEWAYS and 75.5 mm along the
tool axis -- 29.8 degrees off the camera's own mounting axis. The camera's
horizontal field is about 63 degrees, estimated from the 66 mm can filling 66%
of the frame width at that range, which puts the vertical half-angle near 25.
THE GRIP POINT SITS ABOUT FIVE DEGREES OUTSIDE THE FRAME. The camera is
mounted parallel to the tool axis and offset to one side of it, and it never
toes back in.

NO ARM POSE FIXES THIS. arm5_Joint is a revolute about z -- the tool axis
itself -- and the grip point is 2.65 mm off that axis, so rolling the wrist
spins the jaws in place and moves nothing in the image. The gripper projects to
the same region of the frame in every posture the arm can strike. That
invariance is what would have made a fixed pixel region a sound rule; the
trouble is that the region is off the bottom of the sensor.

The approach check is a different matter, because ANGLE FALLS WITH RANGE. The
same 43.3 mm sideways offset subtends less the further down the tool axis the
target sits:

    standoff   range    off-axis
       0 mm    75.5     29.8 deg     the grasp itself -- outside the frame
      20 mm    95.5     24.4 deg     at the edge
      40 mm   115.5     20.5 deg     inside
      80 mm   155.5     15.6 deg     comfortably inside

So `approach` can see what it is asked about and `held` cannot, and CHECK_USABLE
below says so. The fix is physical: aim the camera down and inward by ten or
fifteen degrees, so the tool axis is nearer the middle of the frame instead of
past its edge. Nothing in this file needs to change when that happens -- flip
`held` to True in CHECK_USABLE.

CLASSIFICATION: THE SIZE AXIS IS THE ONE THAT WORKS
---------------------------------------------------
nanoOWL's tree syntax has two forms: [square brackets] DETECT and return boxes,
(parentheses) CLASSIFY the whole frame and pick a winner among the
alternatives. Classification needs no detectable object, which matters here
because the held can is truncated at the frame edge and detection returns
nothing for it at any wording.

Measured 2026-09-05 against three frames from ONE arm pose -- can HELD, NO CAN
at all, can dropped ON the FLOOR still in view:

    (no can, a small can, a huge can)        huge .66   none .80   small .75
    (a small can, a huge can)                huge .67   small .79  small .76
    (a can far from the camera,              close .98  far .95    CLOSE 1.00
     a can touching the camera)                                    <-- wrong
    (an empty gripper,                       held .99   empty .93  HELD 1.00
     a gripper holding a soda can)                                 <-- wrong
    (no soda can, soda can)                  can .86    none .53   can .95
    (a whole can, a cut off can)             whole .99  cut .66    whole .99
                                             <-- inverted: HELD is the cut one
    (a can on the floor, <anything>)         floor      FLOOR .99  floor
                                             <-- constant, all six tried

THE FIRST LINE IS THE RESULT. A three-way on APPARENT SIZE separates all three
states correctly, and it is the physically right question: the held can is a
hand's breadth from the lens and huge, a dropped one is further off and small.
It asks in pictures what a box width would have measured, and works where no
box can be produced at all.

TREAT IT AS PROMISING, NOT ESTABLISHED. One frame per state, and the margins
are 0.66 and 0.75 against a 0.33 chance line -- decisive, but nothing like the
0.99 the presence axis returns. It wants repeating across lighting, can
placements and standoffs before anything gates a pick on it.

THREE RULES FOR WORDING A PROMPT, each bought by a wrong answer above:

  1. THE ALTERNATIVES MUST COVER THE WHOLE SPACE, ABSENCE INCLUDED. A
     classification always returns a winner: '(a small can, a huge can)' calls
     an empty frame "a small can" at 0.79. Adding 'no can' fixed it.
  2. THEY MUST DIFFER ONLY IN THE THING BEING ASKED ABOUT. Any alternative that
     also describes the SCENE gets won by the scene: every pairing containing
     "a can on the floor" was a constant, because all three frames are of a
     floor, and it took the empty one at 0.994.
  3. A PROMPT THAT IS RIGHT ON THE FRAME YOU TRIED IT ON IS NOT YET EVIDENCE.
     Half the wordings above are constants or inversions that read perfectly
     sensibly. Cross-check every candidate against every frame.

Negation is weak wording besides: '(no soda can, soda can)' gets the negative
at 0.53, a coin flip in a two-way softmax, because CLIP embeds a negated noun
close to the noun. 'no can' as one option among three fares better than 'no X'
as one of a pair.

THE POSE SETTLES IT, AND MORE CHEAPLY THAN THE PROMPT DOES. Every wrong answer
above is the same failure: a can in the BACKGROUND, on the floor, read as the
can in the jaws. From a pose tilted up far enough that no floor is in frame,
that scene cannot occur -- the only can that can appear is the one being held,
and plain presence is then the whole answer.

'carry' IS SUCH A POSE. It points the tool well up, so no floor reaches the
frame; the ambiguity is designed out rather than argued with. The geometry
costs nothing either, because RAISING THE ARM DOES NOT MOVE THE HELD CAN IN THE
IMAGE: camera and jaws are rigidly linked (arm5_Joint is a roll about the tool
axis), so the held can sits in the same truncated sliver at the bottom edge
whatever the arm is doing, and only the background moves.

WHICH IS WHY THIS ANSWER MEANS NOTHING FROM ANOTHER POSE. It is not that some
poses are less accurate; it is that a pose which can see the floor can see a
can on the floor, and this check has no way to tell that can from a held one.
The pick sequence asks at carry. is_holding() can be called at any moment from
anywhere, and asked at 'ready' over a littered floor it will say held, at 1.000,
with nothing in the jaws.

The measured margins -- 0.99 holding, 0.93 empty -- were taken at a pose with
floor in view, not at carry. Carry's background is plainer, which should only
help, but the pair is worth re-shooting there before the numbers are quoted as
carry's.

The same dropped can, fully in frame, DETECTS at 0.797, so aiming the camera
down would fix both checks outright -- but it is a hardware change, and the
pose above is not.

WHERE THE INTELLIGENCE LIVES
----------------------------
nanoOWL (nano-owl-service, XML-RPC on port 8000) already runs on this machine
as an open-vocabulary detector, so nothing here loads a model or touches the
GPU. It is started in "network" frame source mode, meaning it does NOT open a
camera of its own -- frames are pushed to it. That is what makes this possible
at all: /dev/video0, the camera on the wrist, stays ours to open.

SHARING ONE DETECTOR
--------------------
The server holds ONE prompt and ONE latest-detection slot, globally, for every
client. The robot's brain uses the same server to look for cans through its own
camera. Two things keep the two uses from reading each other's answers:

  1. Every pushed frame carries a sequence number, and get_detections() reports
     the seq of the frame it ran on. We poll until OUR seq comes back and
     ignore anything else. The seq is seeded from the clock rather than from
     zero so two independent clients do not both start at 1.
  2. The previous prompt is read before ours is set and put back afterwards, so
     a search in progress elsewhere is interrupted rather than silently
     retargeted.

Neither is free of races -- the server has no notion of a session -- and they
are not meant to make concurrent use safe. They make a mid-pick check survive
being interleaved with one. Concurrency is not expected: the brain looks for a
can and the arm then picks it, in that order.
"""

import http.client
import os
import time
import xmlrpc.client
from collections import namedtuple

PRESENT = 'present'
ABSENT = 'absent'
UNKNOWN = 'unknown'

# The nanoOWL XML-RPC server. Loopback by default because it runs on this
# machine -- the same service the robot's brain reaches at 192.168.55.1:8000.
OWL_URL = os.environ.get('DOFBOT_OWL_URL', 'http://127.0.0.1:8000/')

# The wrist camera. One USB camera is attached (a Microdia UVC device); it
# enumerates as /dev/video0 for capture and /dev/video1 for metadata, so the
# capture index is 0. The chassis Oak-D belongs to the robot's brain and is not
# on this machine at all.
CAMERA_INDEX = int(os.environ.get('DOFBOT_ARM_CAMERA', '0'))

# The device's largest mode. Requested, not assumed: every measure below is a
# FRACTION of the frame actually returned, so a device that hands back
# something else stays correct.
FRAME_SIZE = (640, 480)

# UVC auto-exposure needs frames to settle; the first one off a freshly opened
# device is routinely dark or half-written. Read and discard this many before
# keeping one. Cheap at 30 fps -- about a third of a second.
WARMUP_FRAMES = 10

# Below this mean pixel value the frame carries no information and the answer
# is UNKNOWN, not ABSENT. A lens-capped/unlit frame off this camera measures a
# mean near 1.4 of 255 with a maximum of 31, which is what this threshold is
# set against: it separates "no light reached the sensor" from "a dim room".
# A dark scene must never be reported as an object that is not there.
MIN_FRAME_MEAN = 8.0

# Detection score floor. UNTUNED -- carried over from the score_threshold the
# nano-owl client uses for its own filtering, and worth revisiting once
# calibrate_view has produced real frames with and without the can in them.
# Too low invents objects; too high aborts good picks.
MIN_SCORE = 0.1

# --------------------------------------------------- the held classification
#
# The held check is a CLASSIFICATION, not a detection. The held can reaches the
# frame only as a truncated sliver at the bottom edge -- the grip point sits
# about five degrees outside the field of view -- and detection returns nothing
# for it at any wording. Classification does not need a detectable object.
#
# IT REQUIRES A CHECK POSE WITH NO FLOOR IN FRAME, and 'carry' is one: it
# points the tool well up, so the only can that can appear is the one being
# held. Classification answers about the whole picture and cannot tell a can in
# the jaws from one lying in the background -- measured, one in view reads as
# held at 1.000 -- so the pose is what makes the question answerable, and
# asking it from anywhere that sees floor gets a confident wrong answer.
#
# The alternatives differ ONLY in the can, which is what makes the comparison
# about the can and not about the room; see the docstring's three wording
# rules. Measured 2026-09-05: 0.992 holding, 0.927 empty.
def held_alternatives(label):
    """The two worlds the held check chooses between, for this object."""
    return ('an empty gripper', 'a gripper holding ' + label)


# Below this the winner is too close to a coin flip to act on, and the answer
# is UNKNOWN rather than a verdict. A two-way classification's chance line is
# 0.5 and it ALWAYS returns a winner, so without a floor a 0.51 guess would
# abort a pick. Both measured readings clear this comfortably; the wording that
# did not -- '(no soda can, soda can)', which got its negative at 0.527 -- is
# exactly what it is here to reject.
CLASSIFY_MIN_SCORE = 0.70

# ------------------------------------------------------------------ the gate
#
# How far the target's box centre may sit from the CENTRE OF THE IMAGE, as a
# fraction of the frame width, before the nearest-to-centre box stops counting
# as the thing the arm is pointed at.
#
# OFF BY DEFAULT FOR A REASON, and it is not the same reason as the gate above.
# Camera_Link's URDF pose says where the camera body is bolted and nothing
# about which way the lens points, so nobody has established that the target
# lands near the middle of the frame at a standoff. If it does not, a gate here
# aborts picks that would have worked. Confirm the aim from one standoff frame
# before switching this on.
APPROACH_MAX_OFFSET = None

# How long to wait for the detector to answer the frame we pushed, and how
# often to ask. Inference is a couple of hundred milliseconds; the budget is
# generous because the arm is standing still at a standoff while it waits, and
# spending a second there is cheaper than closing the jaws on nothing.
OWL_TIMEOUT = 3.0
OWL_POLL = 0.05

# Connecting to a server that is not listening should fail immediately, not
# hang the pick. This bounds the XML-RPC socket, not the detection.
OWL_CONNECT_TIMEOUT = 2.0

# JPEG quality for the pushed frame. The detector runs on a 640x480 image;
# there is no reason to spend bandwidth beyond what the encoder recovers.
JPEG_QUALITY = 85

# The two questions. The name runs through Sighting, the logs and
# vision_check's CLI, and it selects both the rule that picks the relevant
# detection and the gate applied to it.
CHECKS = ('approach', 'held')

# Whether each check can be answered by this hardware as it is aimed. Not a
# policy knob and not the abort policy -- the same kind of statement as
# gripper.CALIBRATED. Both are answerable now: 'approach' by detection, since
# the target is 15-25 degrees off axis at a standoff and comfortably in frame,
# and 'held' by classification, which needs no detectable object.
CHECK_USABLE = {'approach': True, 'held': True}


class VisionError(RuntimeError):
    """The camera saw that the object is not where the pick assumed it was.

    Raised for ABSENT only. UNKNOWN never raises: an unanswerable question is
    not a negative answer.
    """


Sighting = namedtuple(
    'Sighting',
    'verdict check label score box width offset seen reason inference_time')
Sighting.__doc__ = """One answer from the camera.

verdict   PRESENT, ABSENT or UNKNOWN
check     which question was asked, one of CHECKS
label     the prompt label that was looked for
score     the chosen detection's score, 0.0 if there was none
box       its (x1, y1, x2, y2) in frame pixels, None if there was none
width     its width as a fraction of the frame width -- how near the lens it
          is. 0.0 for the held check, which returns no box
offset    how far its centre sits from the centre of the image, as a fraction
          of the frame width, and what APPROACH_MAX_OFFSET gates on
seen      how many detections of `label` cleared the score floor
reason    plain text: why UNKNOWN, or what was or was not seen
inference_time  seconds the detector spent, 0.0 if it never ran
"""


def _unknown(check, reason, label=''):
    return Sighting(UNKNOWN, check, label, 0.0, None, 0.0, 0.0, 0, reason, 0.0)


# --------------------------------------------------------------------- prompt


def label_for(obj):
    """The nanoOWL prompt label for a catalogue object.

    Derived from the catalogue name -- 'soda_can' asks about 'a soda can' --
    because OWL is open-vocabulary and the catalogue name is already an English
    description of the thing. An entry may override it with a `vision_label`
    when the name is not what the detector should be shown.
    """
    override = getattr(obj, 'vision_label', None)
    if override:
        return override
    return 'a ' + obj.name.replace('_', ' ')


def _prompt(label):
    """One label as a nanoOWL tree prompt.

    Flat, single-label, no nesting. The tree syntax can express "a can inside a
    gripper", but that asks the detector to do the geometry this module does
    itself from the boxes -- and a box can be checked against a saved frame,
    while a nested prompt's opinion cannot.
    """
    return '[%s]' % label


# --------------------------------------------------------------------- camera


def grab(index=CAMERA_INDEX, size=FRAME_SIZE, warmup=WARMUP_FRAMES):
    """One settled BGR frame from the arm camera, or None.

    Opens and releases the device around a single grab, deliberately. Holding
    it open would make this the process that owns /dev/video0, and the pick,
    `is_holding()` and `calibrate_view` are three different processes that each
    need it for a fraction of a second.

    cv2 is imported here rather than at module scope so that the rest of this
    file -- the verdict logic and its tests -- loads on a machine without it.
    """
    import cv2

    camera = cv2.VideoCapture(index)
    try:
        if not camera.isOpened():
            return None
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        frame = None
        for _ in range(max(1, warmup)):
            ok, image = camera.read()
            if ok:
                frame = image
        return frame
    finally:
        camera.release()


def encode(frame, quality=JPEG_QUALITY):
    """A BGR frame as JPEG bytes, or None if the encoder refuses it."""
    import cv2

    ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else None


def is_dark(frame, floor=MIN_FRAME_MEAN):
    """True if the frame carries no usable light. See MIN_FRAME_MEAN."""
    return float(frame.mean()) < floor


def frame_size(frame):
    """(width, height) of a BGR frame, read from the frame, never assumed."""
    return frame.shape[1], frame.shape[0]


# ------------------------------------------------------------------- detector


def _seq():
    """A frame sequence number unlikely to collide with another client's.

    The server correlates a detection to a frame by this number and nothing
    else, and it is the client that assigns it. Seeded from the monotonic clock
    in milliseconds, masked into a positive 32-bit int because XML-RPC's
    integer type is signed 32-bit.
    """
    return int(time.monotonic() * 1000.0) & 0x7FFFFFFF


class _TimeoutTransport(xmlrpc.client.Transport):
    """xmlrpc.client has no timeout knob; make_connection is the way in.

    Without it a server that accepts the connection and then stops answering
    blocks the pick forever, with the arm parked at a standoff. The same shape
    as arm_client's, and for the same reason.
    """

    def __init__(self, timeout):
        super().__init__()
        self._timeout = timeout

    def make_connection(self, host):
        if self._connection and host == self._connection[0]:
            return self._connection[1]
        chost, self._extra_headers, _x509 = self.get_host_info(host)
        self._connection = (host, http.client.HTTPConnection(
            chost, timeout=self._timeout))
        return self._connection[1]


def connect(url=OWL_URL, timeout=OWL_CONNECT_TIMEOUT):
    """A live proxy to the nanoOWL server, or None if it is not answering.

    Not an error: the GPU service is switched between nanoOWL and the VLM by
    the supervisor, so "not running" is an ordinary state of the world.
    """
    try:
        proxy = xmlrpc.client.ServerProxy(
            url, allow_none=True, transport=_TimeoutTransport(timeout))
        proxy.ping()
        return proxy
    except Exception:
        return None


def detect(frame, label, url=OWL_URL, timeout=OWL_TIMEOUT):
    """Run one DETECTION of `label` on one frame, returning boxes.

    Returns (detections, inference_time, reason). `detections` is None when the
    question could not be asked, and `reason` then says why.
    """
    return ask(frame, _prompt(label), url=url, timeout=timeout)


def ask(frame, prompt, url=OWL_URL, timeout=OWL_TIMEOUT):
    """Put one already-formed tree prompt to nanoOWL, on one frame.

    TAKES A PROMPT, NOT A LABEL, and that is the whole reason it is separate
    from detect(): '[a soda can]' detects and '(an empty gripper, ...)'
    classifies, and a function that wraps its argument in brackets cannot send
    the second. Wrapping a classification prompt a second time asks the
    detector to find an object literally named '(an empty gripper, ...)', which
    it answers -- with nothing, every time, on every frame.

    The previous prompt is restored on the way out. See the module docstring on
    sharing one detector: this is a courtesy to whoever set it, not a lock.
    """
    proxy = connect(url)
    if proxy is None:
        return None, 0.0, 'nanoOWL is not answering at %s' % url

    jpeg = encode(frame)
    if jpeg is None:
        return None, 0.0, 'the frame would not JPEG-encode'

    try:
        previous = proxy.get_prompt() or ''
        if not proxy.set_prompt(prompt):
            return None, 0.0, ('nanoOWL rejected the prompt %r -- if it is a '
                               'classification, the first one loads CLIP and '
                               'can run the box out of memory; the reason is '
                               'in the server log' % prompt)
        try:
            seq = _seq()
            if not proxy.push_frame(xmlrpc.client.Binary(jpeg), seq):
                return None, 0.0, ('nanoOWL refused the frame; it is probably '
                                   'running in camera mode, not network mode')
            deadline = time.time() + timeout
            while time.time() < deadline:
                result = proxy.get_detections()
                # BOTH have to match, and the prompt is not redundant. The
                # server reads its prompt when it starts a prediction, so a
                # frame pushed while it was already mid-loop can come back with
                # OUR seq and the PREVIOUS prompt's labels. Seen: a label
                # sweep, where each iteration restored the prompt the one
                # before it had set, and detections that were really there
                # arrived labelled as the previous query and got filtered out
                # as "nothing". Whoever else is using the server can move the
                # prompt under us the same way.
                if (result and result.get('frame_seq') == seq
                        and result.get('prompt') == prompt):
                    return (list(result.get('detections') or []),
                            float(result.get('inference_time') or 0.0), '')
                time.sleep(OWL_POLL)
            return None, 0.0, ('nanoOWL did not answer frame %d within %.1fs'
                               % (seq, timeout))
        finally:
            # Only put back a prompt there was one; set_prompt('') does not
            # parse as a tree and would log an error on the server for nothing.
            #
            # Swallowed on purpose: by here we may already be returning a good
            # answer, and failing to restore somebody else's prompt must not
            # turn that into "could not ask". The next client to look sets its
            # own prompt anyway.
            if previous and previous != prompt:
                try:
                    proxy.set_prompt(previous)
                except Exception:
                    pass
    except Exception as exc:
        return None, 0.0, 'nanoOWL call failed: %s' % exc


def classify(frame, alternatives, url=OWL_URL, timeout=OWL_TIMEOUT):
    """Ask which of `alternatives` the whole frame is, as nanoOWL sees it.

    Returns (index, score, inference_time, reason). `index` is None when the
    question could not be asked, and `reason` then says why.

    The wire format is the reason this is not just detect() with parentheses.
    A classification comes back as ONE detection -- the root 'image' node,
    boxed to the whole frame -- whose `labels` are indices into the tree's own
    label list and whose `scores` are the softmax over them. Index 0 is the
    root itself and always scores 1.0; the rest are the alternatives, in the
    order they were written. candidates() drops the root node by design, so the
    detection path cannot read this at all.
    """
    prompt = '(%s)' % ', '.join(alternatives)
    detections, seconds, reason = ask(frame, prompt, url=url, timeout=timeout)
    if detections is None:
        return None, 0.0, 0.0, reason
    for det in detections:
        for index, score in zip(det.get('labels') or [],
                                det.get('scores') or []):
            # Index 0 is the root image node scoring 1.0, not an answer.
            if int(index) >= 1:
                return int(index) - 1, float(score), seconds, ''
    return None, 0.0, seconds, ('nanoOWL returned no classification for %r'
                                % prompt)


# --------------------------------------------------------------- choosing one

Candidate = namedtuple('Candidate', 'box score area width offset')


def centre(box):
    """The centre (x, y) of an (x1, y1, x2, y2) box."""
    return ((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0)


def candidates(detections, label, size, floor=MIN_SCORE):
    """Every detection of `label` worth considering, measured.

    `size` is the frame's (width, height). Each candidate carries the two
    figures the rules and the gates are expressed in -- `width` and `offset`,
    both fractions of the frame width -- so nothing downstream re-derives them
    from pixels and no rule has to know the resolution.

    The tree predictor returns the root 'image' node alongside real detections;
    it is not an object and is dropped. Matching is exact and case-insensitive
    against the label that was asked for, so a prompt of one label cannot be
    answered by another.
    """
    frame_w, frame_h = size
    wanted = label.strip().lower()
    found = []
    for det in detections or []:
        name = str(det.get('label') or '').strip().lower()
        if not name or name == 'image' or name != wanted:
            continue
        scores = det.get('scores') or []
        score = max(scores) if scores else 0.0
        if score < floor:
            continue
        box = [float(v) for v in det.get('box') or []]
        if len(box) != 4:
            # No box is nothing to measure, so there is no way to rank it
            # against the others or to gate it. Not a candidate.
            continue
        box_w = max(0.0, box[2] - box[0])
        box_h = max(0.0, box[3] - box[1])
        cx, cy = centre(box)
        # BOTH axes are scaled by the frame WIDTH. Dividing y by the height
        # instead would stretch the vertical, and "how far off centre" would
        # then mean something different up the frame than across it.
        offset = ((((cx - frame_w / 2.0) ** 2 + (cy - frame_h / 2.0) ** 2)
                   ** 0.5) / frame_w) if frame_w else 0.0
        found.append(Candidate(box, score, box_w * box_h,
                               (box_w / frame_w) if frame_w else 0.0, offset))
    return found


def largest(cands):
    """The nearest thing to the lens: the biggest box. See CHECKS, 'held'."""
    return max(cands, key=lambda c: c.area) if cands else None


def nearest_centre(cands):
    """What the arm is pointed at. See CHECKS, 'approach'."""
    return min(cands, key=lambda c: c.offset) if cands else None


def choose(check, cands):
    """The one detection `check` is talking about, or None if there are none."""
    return largest(cands) if check == 'held' else nearest_centre(cands)


# ---------------------------------------------------------------------- check


def judge(check, chosen):
    """Does the chosen detection pass this check's gate?

    Approach only -- held is a classification and never reaches here. Returns
    (present, why). A gate that is None is off, and the answer is then the
    presence test that needs no measurement: something of the right label is in
    view. See the gate above for why it defaults off.
    """
    if APPROACH_MAX_OFFSET is None:
        return True, ('in view and %.0f%% of the frame width off centre; no '
                      'limit is set, so any can in view counts'
                      % (100.0 * chosen.offset))
    if chosen.offset <= APPROACH_MAX_OFFSET:
        return True, ('%.0f%% of the frame width off centre, inside the %.0f%% '
                      'the arm is pointed at'
                      % (100.0 * chosen.offset, 100.0 * APPROACH_MAX_OFFSET))
    return False, ('%.0f%% of the frame width off centre, outside the %.0f%% '
                   'the arm is pointed at -- not the can being approached'
                   % (100.0 * chosen.offset, 100.0 * APPROACH_MAX_OFFSET))


def look(check, label, frame=None, url=OWL_URL):
    """Ask the camera one question, and answer it in three values.

    check   one of CHECKS. It selects both the rule that picks the relevant
            detection and the gate applied to it
    label   what to look for, e.g. 'a soda can'; see label_for()
    frame   a BGR frame to use instead of grabbing one. For calibrate_view and
            for tests; a live check grabs its own.

    Never raises for a world it cannot see. Every failure to ask -- no camera,
    no light, no detector -- comes back UNKNOWN with a reason.
    """
    if check not in CHECKS:
        raise ValueError('no such check %r; have %s' % (check, list(CHECKS)))

    if not CHECK_USABLE[check]:
        return _unknown(check, 'the camera cannot see what the %s check is '
                               'about: the grip point sits just outside its '
                               'field of view, so nothing is ever detected '
                               'there. See CHECK_USABLE' % check, label)

    if frame is None:
        frame = grab()
    if frame is None:
        return _unknown(check, 'the arm camera at index %d would not open or '
                               'would not read' % CAMERA_INDEX, label)
    if is_dark(frame):
        return _unknown(check, 'the arm camera returned a frame with mean '
                               '%.1f, below the %.1f that says any light '
                               'reached the sensor -- check the lighting and '
                               'the lens' % (frame.mean(), MIN_FRAME_MEAN),
                        label)

    if check == 'held':
        return _look_held(label, frame, url)

    detections, seconds, reason = detect(frame, label, url=url)
    if detections is None:
        return _unknown(check, reason, label)

    cands = candidates(detections, label, frame_size(frame))
    if not cands:
        return Sighting(ABSENT, check, label, 0.0, None, 0.0, 0.0, 0,
                        'no %s scoring %.2f or better anywhere in the frame'
                        % (label, MIN_SCORE), seconds)

    chosen = choose(check, cands)
    present, why = judge(check, chosen)
    # Say how many were in frame whenever the rule had to choose. Otherwise a
    # picture with three cans in it and a wrong answer reads exactly like a
    # picture with one.
    among = (' (%d in frame; took the %s)'
             % (len(cands),
                'largest' if check == 'held' else 'one nearest the centre')
             if len(cands) > 1 else '')
    return Sighting(PRESENT if present else ABSENT, check, label, chosen.score,
                    list(chosen.box), chosen.width, chosen.offset, len(cands),
                    '%s at score %.2f, %s%s' % (label, chosen.score, why, among),
                    seconds)


def _look_held(label, frame, url):
    """The held check: a classification, not a detection. See CHECKS.

    ONLY MEANINGFUL FROM A POSE WITH NO FLOOR IN FRAME, which is 'carry' and is
    not most other poses. Classification answers about the whole picture, so a
    can in the background is a can as far as it is concerned -- measured, one
    lying in view reads as held at 1.000. Carry points the tool well up and no
    floor reaches the frame, so the only can that can appear there is the one
    being held. That is the workaround, and it beats any wording: it removes
    the scene rather than arguing with it.

    Asked from somewhere that CAN see the floor, this will confidently report a
    can on the floor as a can in the jaws. See CHECKS.
    """
    alternatives = held_alternatives(label)
    index, score, seconds, reason = classify(frame, alternatives, url=url)
    if index is None:
        return _unknown('held', reason, label)
    if not 0 <= index < len(alternatives):
        return _unknown('held', 'nanoOWL answered with alternative %d, which '
                                'was not asked' % index, label)

    winner = alternatives[index]
    if score < CLASSIFY_MIN_SCORE:
        return _unknown('held', '%r at %.2f, under the %.2f that separates an '
                                'answer from a coin flip -- a two-way choice '
                                'always returns a winner'
                                % (winner, score, CLASSIFY_MIN_SCORE), label)

    # The last alternative is the holding one; see held_alternatives.
    holding = index == len(alternatives) - 1
    return Sighting(PRESENT if holding else ABSENT, 'held', label, score, None,
                    0.0, 0.0, 1 if holding else 0,
                    '%r at %.2f' % (winner, score), seconds)


def approach(obj, frame=None, url=OWL_URL):
    """Is `obj` where the pick was told it is? Asked from the standoff."""
    return look('approach', label_for(obj), frame, url)


def held(obj, frame=None, url=OWL_URL):
    """Is `obj` still in the jaws? Asked at carry."""
    return look('held', label_for(obj), frame, url)


def as_dict(sighting):
    """A Sighting as plain JSON-able types, for crossing a process boundary."""
    return {
        'verdict': sighting.verdict,
        'check': sighting.check,
        'label': sighting.label,
        'score': round(sighting.score, 4),
        'box': [round(v, 1) for v in sighting.box] if sighting.box else None,
        'width': round(sighting.width, 4),
        'offset': round(sighting.offset, 4),
        'seen': int(sighting.seen),
        'reason': sighting.reason,
        'inference_time': round(sighting.inference_time, 4),
    }


def describe(sighting):
    """One line for a log."""
    return '%s check: %s -- %s' % (sighting.check, sighting.verdict.upper(),
                                   sighting.reason)
