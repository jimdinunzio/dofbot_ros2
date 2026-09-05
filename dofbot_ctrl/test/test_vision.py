#!/usr/bin/env python3
# coding: utf-8
"""
The camera check: which detection each question is about, and what it earns.

TWO THINGS ARE BEING TESTED and they are not the same. First, SELECTION -- of
the cans in frame, which one is the check talking about. The held can is the
largest box because it is nearest the lens; the can being approached is the one
nearest the image centre because that is where the arm is pointed. Second,
PRESENCE -- whether that one counts, which is where the two optional gates come
in. Selection always picks something whenever anything is detected, so on its
own it would answer yes about a can lying on the floor.

AND THE BOUNDARY BETWEEN ABSENT AND UNKNOWN. ABSENT aborts a pick; UNKNOWN lets
it run. Every way the question can fail to be asked -- no detector, no camera,
no light -- has to land on UNKNOWN, because a stack that cannot see is the
normal state of this machine whenever the GPU is running something else, and it
picked cans perfectly well before it could see at all.

No camera and no detector are touched. The frames are arrays built here and the
detector is a stub, so these run anywhere.

Values are RELATIONS against the module's own constants -- scores against
MIN_SCORE, brightness against MIN_FRAME_MEAN, sizes against whichever gate is
set -- so retuning any of them moves the expectation with it rather than
breaking a test that remembered a number.
"""

import importlib.util
import json
import os

import pytest

from dofbot_ctrl import graspable, vision

np = pytest.importorskip('numpy', reason='frames are numpy arrays')

WIDTH, HEIGHT = vision.FRAME_SIZE
SIZE = (WIDTH, HEIGHT)

CAN = graspable.get('soda_can')
LABEL = vision.label_for(CAN)


def frame(value):
    """A flat BGR frame of the given brightness."""
    return np.full((HEIGHT, WIDTH, 3), value, dtype=np.uint8)


def lit():
    """A frame bright enough that is_dark() passes it."""
    return frame(int(vision.MIN_FRAME_MEAN) + 20)


def detection(box, score=None, label=LABEL):
    if score is None:
        score = vision.MIN_SCORE * 2.0
    return {'label': label, 'label_id': 1, 'parent_label': 'image',
            'parent_id': 0, 'box': list(box), 'scores': [score],
            'labels': [1]}


def box_at(x, y, width, height=None):
    """A box of the given size centred on (x, y)."""
    height = width if height is None else height
    return (x - width / 2.0, y - height / 2.0,
            x + width / 2.0, y + height / 2.0)


def near_can(x=None, y=None):
    """A can filling much of the frame: what being in the jaws looks like."""
    return box_at(WIDTH / 2.0 if x is None else x,
                  HEIGHT / 2.0 if y is None else y, WIDTH * 0.4, HEIGHT * 0.6)


def far_can(x=None, y=None):
    """A can several times further off: one lying in view, not held."""
    return box_at(WIDTH / 2.0 if x is None else x,
                  HEIGHT / 2.0 if y is None else y, WIDTH * 0.1, HEIGHT * 0.15)


def stub_detector(monkeypatch, detections, seconds=0.1, reason=''):
    monkeypatch.setattr(vision, 'detect',
                        lambda *a, **k: (detections, seconds, reason))


def gates(monkeypatch, offset=None):
    """Set the approach gate for one test. Module-level policy."""
    monkeypatch.setattr(vision, 'APPROACH_MAX_OFFSET', offset)


@pytest.fixture(autouse=True)
def _both_checks_usable(monkeypatch):
    """Test the rules as if the camera could see both subjects.

    It currently cannot -- CHECK_USABLE['held'] is False because the grip point
    sits outside the frame -- and that short-circuits look() before any rule
    runs. These tests are about the rules, which have to keep working for the
    day the camera is aimed properly. What CHECK_USABLE itself does is tested
    separately, below.
    """
    monkeypatch.setattr(vision, 'CHECK_USABLE',
                        {check: True for check in vision.CHECKS})


# ------------------------------------------------------------------- prompts


def test_label_comes_from_the_catalogue_name():
    assert vision.label_for(CAN) == 'a ' + CAN.name.replace('_', ' ')


def test_an_entry_can_override_its_own_label():
    named = graspable.GraspableObject(
        name='soda_can_2', shape='cylinder', width=CAN.width,
        height=CAN.height, vision_label='a silver drinks can')
    assert vision.label_for(named) == 'a silver drinks can'


def test_the_prompt_is_one_flat_tree_node():
    prompt = vision._prompt(LABEL)
    assert prompt.startswith('[') and prompt.endswith(']')
    # One label, no nesting: the geometry is done here, not by the detector.
    assert '[' not in prompt[1:] and ',' not in prompt


# --------------------------------------------------------------- measurement


def test_width_is_a_fraction_of_the_frame_width():
    cand, = vision.candidates([detection(box_at(WIDTH / 2.0, HEIGHT / 2.0,
                                                WIDTH / 4.0))], LABEL, SIZE)
    assert cand.width == pytest.approx(0.25)


def test_a_box_on_the_image_centre_has_no_offset():
    cand, = vision.candidates(
        [detection(near_can())], LABEL, SIZE)
    assert cand.offset == pytest.approx(0.0)


def test_offset_uses_the_frame_width_in_both_axes():
    """Otherwise 'how far off centre' means two things in one number."""
    across, = vision.candidates(
        [detection(far_can(x=WIDTH / 2.0 + WIDTH / 10.0))], LABEL, SIZE)
    down, = vision.candidates(
        [detection(far_can(y=HEIGHT / 2.0 + WIDTH / 10.0))], LABEL, SIZE)
    assert across.offset == pytest.approx(down.offset)
    assert across.offset == pytest.approx(0.1)


# ----------------------------------------------------------------- selection


def test_approach_takes_the_box_nearest_the_image_centre():
    """The arm is pointed at the target; other cans sit off to the sides."""
    middle = detection(far_can())
    aside = detection(near_can(x=WIDTH * 0.9))
    cands = vision.candidates([middle, aside], LABEL, SIZE)
    assert vision.choose('approach', cands).box == list(middle['box'])


def test_a_higher_score_does_not_win_a_geometric_rule():
    """Score decides what counts as a detection, never which one is chosen."""
    ahead = detection(far_can(), score=vision.MIN_SCORE * 1.5)
    aside = detection(near_can(x=WIDTH * 0.9), score=1.0)
    cands = vision.candidates([ahead, aside], LABEL, SIZE)
    assert vision.choose('approach', cands).box == list(ahead['box'])


# --------------------------------------------- held is a classification


def stub_classifier(monkeypatch, index, score, seconds=0.05, reason=''):
    monkeypatch.setattr(vision, 'classify',
                        lambda *a, **k: (index, score, seconds, reason))


def test_the_holding_alternative_is_the_last_one():
    """_look_held reads the winner by position, so the order is load-bearing."""
    alts = vision.held_alternatives(LABEL)
    assert len(alts) == 2
    assert LABEL in alts[-1]


def test_the_holding_alternative_means_held(monkeypatch):
    alts = vision.held_alternatives(LABEL)
    stub_classifier(monkeypatch, len(alts) - 1, 0.99)
    sighting = vision.look('held', LABEL, frame=lit())
    assert sighting.verdict == vision.PRESENT


def test_the_empty_alternative_means_not_held(monkeypatch):
    stub_classifier(monkeypatch, 0, 0.93)
    sighting = vision.look('held', LABEL, frame=lit())
    assert sighting.verdict == vision.ABSENT


def test_a_near_coin_flip_is_unknown_not_a_verdict(monkeypatch):
    """A two-way classification ALWAYS returns a winner, so a bare winner is
    not an answer. The measured wording this rejects got its negative at
    0.527, against a 0.5 chance line."""
    alts = vision.held_alternatives(LABEL)
    stub_classifier(monkeypatch, len(alts) - 1,
                    vision.CLASSIFY_MIN_SCORE - 0.01)
    assert vision.look('held', LABEL, frame=lit()).verdict == vision.UNKNOWN
    stub_classifier(monkeypatch, len(alts) - 1, vision.CLASSIFY_MIN_SCORE)
    assert vision.look('held', LABEL, frame=lit()).verdict == vision.PRESENT


def test_the_floor_is_above_a_two_way_coin_flip():
    """Below 0.5 the 'winner' of a pair is not even the more likely one."""
    assert vision.CLASSIFY_MIN_SCORE > 0.5


def test_a_classification_that_could_not_be_asked_is_unknown(monkeypatch):
    stub_classifier(monkeypatch, None, 0.0, reason='nanoOWL is not answering')
    assert vision.look('held', LABEL, frame=lit()).verdict == vision.UNKNOWN


def test_an_alternative_nobody_asked_for_is_unknown(monkeypatch):
    """Guards the index arithmetic between our prompt and the tree's labels."""
    stub_classifier(monkeypatch, 99, 0.99)
    assert vision.look('held', LABEL, frame=lit()).verdict == vision.UNKNOWN


def test_a_prompt_is_never_wrapped_twice(monkeypatch):
    """detect() brackets its argument; classify() must not go through it.

    It did, once, and the bug was silent and total: the detector was asked to
    find an object literally named "(an empty gripper, ...)" and answered with
    nothing, on every frame, so every held check came back UNKNOWN.
    """
    sent = []
    monkeypatch.setattr(vision, 'ask',
                        lambda frame, prompt, **k: (sent.append(prompt),
                                                    ([], 0.05, ''))[1])
    vision.classify(lit(), ('an empty gripper', 'a gripper holding a can'))
    vision.detect(lit(), LABEL)
    classification, detection_prompt = sent
    assert classification.startswith('(') and classification.endswith(')')
    assert detection_prompt.startswith('[') and detection_prompt.endswith(']')
    for prompt in sent:
        assert '[(' not in prompt and '([' not in prompt


def test_classify_reads_the_root_node_the_detection_path_drops(monkeypatch):
    """The answer lives in the 'image' node, which candidates() discards.

    Index 0 is the root itself scoring 1.0; the alternatives follow, in the
    order they were written into the prompt.
    """
    root = {'label': 'image', 'label_id': 0, 'parent_label': '',
            'parent_id': -1, 'box': [0, 0, WIDTH, HEIGHT],
            'labels': [0, 2], 'scores': [1.0, 0.856]}
    monkeypatch.setattr(vision, 'ask', lambda *a, **k: ([root], 0.05, ''))
    index, score, _secs, why = vision.classify(lit(), ('a', 'b'))
    assert (index, why) == (1, '')
    assert score == pytest.approx(0.856)
    # And the detection path really does throw it away.
    assert vision.candidates([root], 'image', SIZE) == []


# ------------------------------------------------------- what counts at all


def test_the_root_image_node_is_not_an_object():
    root = detection((0, 0, WIDTH, HEIGHT), 1.0, label='image')
    assert vision.candidates([root], LABEL, SIZE) == []


def test_another_label_cannot_answer_our_question():
    other = detection(near_can(), label='a dog')
    assert vision.candidates([other], LABEL, SIZE) == []


def test_scores_under_the_floor_are_not_detections():
    weak = detection(near_can(), score=vision.MIN_SCORE / 2.0)
    assert vision.candidates([weak], LABEL, SIZE) == []
    strong = detection(near_can(), score=vision.MIN_SCORE)
    assert len(vision.candidates([strong], LABEL, SIZE)) == 1


def test_a_detection_with_no_box_cannot_be_ranked():
    """No box is nothing to measure, so it can neither win nor be gated."""
    assert vision.candidates([dict(detection(near_can()), box=[])],
                             LABEL, SIZE) == []


# ------------------------------------------------- talking to the detector


class FakeOwl:
    """A stand-in for the nanoOWL server, with its one global prompt."""

    def __init__(self, answers):
        # answers: list of (frame_seq, prompt, detections) to hand out in turn
        self.answers = list(answers)
        self.prompt = ''
        self.prompts_set = []
        self.pushed = []

    def ping(self):
        return 'pong'

    def get_prompt(self):
        return self.prompt

    def set_prompt(self, prompt):
        self.prompt = prompt
        self.prompts_set.append(prompt)
        return True

    def push_frame(self, data, seq):
        self.pushed.append(seq)
        return True

    def get_detections(self):
        if not self.answers:
            return None
        seq, prompt, dets = self.answers.pop(0)
        return {'prompt': prompt, 'detections': dets, 'inference_time': 0.05,
                'frame_seq': self.pushed[-1] if seq == 'ours' else seq}


def test_an_answer_for_our_frame_under_another_prompt_is_not_ours(monkeypatch):
    """The server reads its prompt when it STARTS a prediction.

    A frame pushed while it was already mid-loop comes back carrying our seq
    and the previous prompt's labels. Matching on seq alone accepted that, and
    the real detections in it were then filtered out as the wrong label --
    reported as "nothing seen" for an object plainly in the frame.
    """
    ours = vision._prompt(LABEL)
    owl = FakeOwl([('ours', '[something else]', [detection(near_can())]),
                   ('ours', ours, [detection(near_can())])])
    monkeypatch.setattr(vision, 'connect', lambda *a, **k: owl)
    monkeypatch.setattr(vision, 'encode', lambda *a, **k: b'jpeg')

    dets, seconds, why = vision.detect(lit(), LABEL)
    assert dets is not None and why == ''
    assert seconds == pytest.approx(0.05)


def test_the_previous_prompt_is_put_back(monkeypatch):
    owl = FakeOwl([('ours', vision._prompt(LABEL), [])])
    owl.prompt = '[a bin]'
    monkeypatch.setattr(vision, 'connect', lambda *a, **k: owl)
    monkeypatch.setattr(vision, 'encode', lambda *a, **k: b'jpeg')

    vision.detect(lit(), LABEL)
    assert owl.prompt == '[a bin]'


def test_a_stale_answer_from_another_client_is_ignored(monkeypatch):
    """Somebody else's frame, right prompt, wrong seq."""
    ours = vision._prompt(LABEL)
    owl = FakeOwl([(12345, ours, [detection(near_can())])])
    monkeypatch.setattr(vision, 'connect', lambda *a, **k: owl)
    monkeypatch.setattr(vision, 'encode', lambda *a, **k: b'jpeg')
    monkeypatch.setattr(vision, 'OWL_TIMEOUT', 0.15)

    dets, _seconds, why = vision.detect(lit(), LABEL, timeout=0.15)
    assert dets is None and 'did not answer' in why


# ------------------------------------------------------- what earns UNKNOWN


def test_a_dark_frame_is_unknown_not_absent(monkeypatch):
    """The room being unlit is not evidence that the can is gone."""
    stub_detector(monkeypatch, [])
    dark = frame(max(0, int(vision.MIN_FRAME_MEAN) - 1))
    assert vision.is_dark(dark)
    assert vision.look('held', LABEL, frame=dark).verdict == vision.UNKNOWN


def test_no_camera_is_unknown_not_absent(monkeypatch):
    monkeypatch.setattr(vision, 'grab', lambda *a, **k: None)
    assert vision.look('held', LABEL).verdict == vision.UNKNOWN


def test_a_detector_that_is_not_running_is_unknown_not_absent(monkeypatch):
    """The whole reason UNKNOWN exists: the GPU service is often elsewhere."""
    monkeypatch.setattr(vision, 'connect', lambda *a, **k: None)
    sighting = vision.look('held', LABEL, frame=lit())
    assert sighting.verdict == vision.UNKNOWN
    assert 'nanoOWL' in sighting.reason


def test_nothing_seen_at_all_is_absent(monkeypatch):
    """This one IS a negative answer: the view was good and there was no can."""
    stub_detector(monkeypatch, [])
    sighting = vision.look('approach', LABEL, frame=lit())
    assert sighting.verdict == vision.ABSENT
    assert sighting.box is None and sighting.seen == 0


def test_a_check_the_camera_cannot_see_is_unknown_not_absent(monkeypatch):
    """The one that is live today, and the reason it has to be UNKNOWN.

    'held' asks about the grip point, which sits just outside the frame, so it
    never detects anything there. ABSENT aborts a pick -- so treating a blind
    check as a negative answer would abort every pick at carry.
    """
    monkeypatch.setattr(vision, 'CHECK_USABLE', dict(vision.CHECK_USABLE,
                                                     held=False))
    monkeypatch.setattr(vision, 'grab',
                        lambda *a, **k: pytest.fail('must not reach the camera'))
    sighting = vision.look('held', LABEL)
    assert sighting.verdict == vision.UNKNOWN
    assert 'CHECK_USABLE' in sighting.reason


def test_every_check_has_a_usability_verdict():
    """A new check with no entry would raise KeyError inside a pick."""
    assert set(vision.CHECK_USABLE) == set(vision.CHECKS)


def test_an_unknown_check_name_is_a_programming_error():
    with pytest.raises(ValueError):
        vision.look('sideways', LABEL, frame=lit())


# ----------------------------------------------------------------- reporting


def test_the_sighting_carries_what_the_gate_reads(monkeypatch):
    stub_detector(monkeypatch, [detection(near_can())])
    sighting = vision.look('approach', LABEL, frame=lit())
    expected = vision.candidates([detection(near_can())], LABEL, SIZE)[0]
    assert sighting.width == pytest.approx(expected.width)
    assert sighting.offset == pytest.approx(expected.offset)


def test_how_many_were_in_frame_is_reported(monkeypatch):
    stub_detector(monkeypatch, [detection(near_can(x=WIDTH * 0.2)),
                                detection(far_can(x=WIDTH * 0.8))])
    assert vision.look('approach', LABEL, frame=lit()).seen == 2


def test_the_check_name_is_carried_through(monkeypatch):
    stub_detector(monkeypatch, [])
    assert vision.look('approach', LABEL, frame=lit()).check == 'approach'


def test_a_sighting_survives_json(monkeypatch):
    stub_detector(monkeypatch, [detection(near_can())])
    payload = vision.as_dict(vision.look('approach', LABEL, frame=lit()))
    assert json.loads(json.dumps(payload)) == payload


# ------------------------------------------------------ crossing a process


def _arm_server():
    """arm_server.py, imported from beside dofbot_ctrl. Stdlib only, no side
    effects at import -- main() is guarded."""
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.abspath(
        os.path.join(here, '..', '..', 'arm_service', 'arm_server.py'))
    if not os.path.exists(path):
        pytest.skip('arm_service is not beside dofbot_ctrl in this tree')
    spec = importlib.util.spec_from_file_location('_arm_server', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_vision_check_and_arm_server_agree_on_the_verdict_line():
    """The one line arm_server parses. Two files, one literal.

    They are versioned together and nothing enforces the match at runtime: a
    prefix changed in one and not the other makes is_holding() answer 'unknown'
    forever, and say nothing about why.
    """
    from dofbot_ctrl import vision_check

    assert _arm_server().VERDICT_PREFIX == vision_check.VERDICT_PREFIX


def test_arm_server_reads_the_three_answers_as_three():
    """held is True/False/None, and None is never False."""
    module = _arm_server()

    def line(verdict):
        return module.VERDICT_PREFIX + json.dumps({'verdict': verdict,
                                                   'reason': verdict})

    assert module._verdict(line(vision.PRESENT))['held'] is True
    assert module._verdict(line(vision.ABSENT))['held'] is False
    assert module._verdict(line(vision.UNKNOWN))['held'] is None
    # Nothing to parse is the unknown answer, not a negative one.
    assert module._verdict('move_group said something else')['held'] is None
    assert module._verdict(module.VERDICT_PREFIX + 'not json')['held'] is None


def test_the_last_verdict_line_wins():
    module = _arm_server()
    output = '\n'.join([
        module.VERDICT_PREFIX + json.dumps({'verdict': vision.UNKNOWN}),
        module.VERDICT_PREFIX + json.dumps({'verdict': vision.PRESENT}),
    ])
    assert module._verdict(output)['held'] is True


# --------------------------------------------------------- the shipped state


def test_the_approach_gate_is_either_off_or_a_sane_fraction():
    """Survives being set: it checks the shape of whatever is there."""
    gate = vision.APPROACH_MAX_OFFSET
    assert gate is None or 0.0 < gate < 1.0
