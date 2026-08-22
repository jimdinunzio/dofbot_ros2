"""
Offline tests for the servo-bus read path: no ROS, no hardware.

    pytest src/dofbot_ros2/dofbot_ctrl/test/test_serial_reads.py

Arm_Driver runs against a fake serial port, so framing, the early-exit read and
the stale-reply flush are all testable without an arm. The fake counts read()
calls and byte requests, which is how "a successful query does not wait out the
timeout" gets asserted rather than asserted-in-a-comment.

Expected values are computed from the driver's own constants: a test that
hard-codes 3100 passes just as happily after someone re-ranges the servos.
"""

import time

import pytest

from Arm_Lib import arm_driver
from Arm_Lib.arm_driver import Arm_Driver


def _reply_frame(sid, pos):
    """A well-formed servo reply, built the way _parse_reply expects to read it."""
    val_h, val_l = (pos >> 8) & 0xFF, pos & 0xFF
    body = [sid, 0x04, 0x02, val_h, val_l]
    return bytes([0xFF, arm_driver._REPLY_HEADER] + body
                 + [(~sum(body)) & 0xFF])


class FakeSerial:
    """A serial port that hands back a scripted stream, one read() at a time.

    read(n) returns at most n bytes and NEVER blocks past what is queued: an
    empty return stands for "the timeout expired with nothing further". That is
    the behaviour the real port has once the reply has landed, and it is what
    makes a wait-for-32-bytes read expensive and an early-exit read cheap.
    """

    def __init__(self, stream=b'', timeout=0.02):
        self.stream = stream
        self.timeout = timeout
        self.written = b''
        self.reads = []          # sizes requested, in order
        self.flushes = 0

    @property
    def in_waiting(self):
        return len(self.stream)

    def write(self, data):
        self.written += data
        return len(data)

    def read(self, size=1):
        self.reads.append(size)
        if not self.stream:
            # Nothing queued: the real port blocks for its whole timeout and
            # returns empty. Sleep so a test can tell a timeout from a hit.
            time.sleep(self.timeout)
            return b''
        out, self.stream = self.stream[:size], self.stream[size:]
        return out

    def reset_input_buffer(self):
        self.flushes += 1
        self.stream = b''


def _driver(stream=b'', timeout=0.02):
    """An Arm_Driver wired to a FakeSerial, bypassing __init__'s real port."""
    d = Arm_Driver.__new__(Arm_Driver)
    d.port = 'fake'
    d.reply_timeout = timeout
    d.ser = FakeSerial(stream, timeout)
    import threading
    d.lock = threading.Lock()
    d._warned = set()
    return d


def _queue(d, data):
    """Put bytes on the wire AFTER the flush that _query does before writing."""
    original = d.ser.reset_input_buffer

    def flush_then_arm():
        original()
        d.ser.stream = data
        d.ser.reset_input_buffer = original
    d.ser.reset_input_buffer = flush_then_arm


# --------------------------------------------------------------- the fix

def test_successful_query_does_not_wait_out_the_timeout():
    """A reply that has landed must return immediately, not wait out the
    timeout. Asking for more bytes than a frame holds is what breaks this."""
    pos = (arm_driver.POS_MIN + arm_driver.POS_MAX) // 2
    d = _driver(timeout=0.05)
    _queue(d, _reply_frame(1, pos))

    t0 = time.monotonic()
    got = d._query(1, arm_driver._CMD_READ, arm_driver._ADDR_READ_POS, [0x02],
                   expect_id=1)
    elapsed = time.monotonic() - t0

    assert got == pos
    assert elapsed < d.reply_timeout, (
        'a query whose reply was already waiting took %.1f ms of a %.1f ms '
        'timeout' % (elapsed * 1000, d.reply_timeout * 1000))


def test_never_asks_for_more_than_a_frame_at_a_time():
    """No read() may request more bytes than a frame holds -- that is what
    makes read() unable to return early."""
    d = _driver()
    _queue(d, _reply_frame(3, arm_driver.POS_MIN))
    d._query(3, arm_driver._CMD_READ, arm_driver._ADDR_READ_POS, [0x02],
             expect_id=3)
    assert max(d.ser.reads) <= arm_driver._REPLY_LEN


def test_silent_servo_costs_the_timeout_and_returns_none():
    """The timeout bounds silence, and only silence."""
    d = _driver(timeout=0.02)
    t0 = time.monotonic()
    got = d._query(2, arm_driver._CMD_READ, arm_driver._ADDR_READ_POS, [0x02],
                   expect_id=2)
    elapsed = time.monotonic() - t0
    assert got is None
    assert elapsed >= d.reply_timeout


def test_stale_reply_is_flushed_before_the_next_query():
    """A reply that arrived after its query gave up must not answer the retry.

    The id check cannot catch this: a retry re-queries the same servo, so the
    stale frame matches and the caller silently gets a position from one query
    ago. The flush is the only thing standing between that and a feedback loop
    reading history.
    """
    stale, fresh = arm_driver.POS_MIN, arm_driver.POS_MAX
    d = _driver()
    d.ser.stream = _reply_frame(4, stale)     # left over, not yet consumed
    _queue(d, _reply_frame(4, fresh))

    got = d._query(4, arm_driver._CMD_READ, arm_driver._ADDR_READ_POS, [0x02],
                   expect_id=4)
    assert d.ser.flushes >= 1
    assert got == fresh


def test_leading_echo_is_skipped():
    """A half-duplex bus echoes the command back; the frame after it is the
    reply."""
    pos = arm_driver.POS_MIN + 100
    d = _driver()
    echo = d._build_frame(5, arm_driver._CMD_READ, arm_driver._ADDR_READ_POS,
                          [0x02])
    _queue(d, echo + _reply_frame(5, pos))
    assert d._query(5, arm_driver._CMD_READ, arm_driver._ADDR_READ_POS, [0x02],
                    expect_id=5) == pos


# ------------------------------------------------------- float conversion

@pytest.mark.parametrize('sid', range(1, 7))
def test_float_and_int_conversions_agree_at_the_range_ends(sid):
    """_pos_to_angle_f must be the same map, only without the truncation, so
    the endpoints -- where there is no fraction to lose -- have to match."""
    lo = arm_driver.POS5_MIN if sid == 5 else arm_driver.POS_MIN
    hi = arm_driver.POS5_MAX if sid == 5 else arm_driver.POS_MAX
    for pos in (lo, hi):
        assert (Arm_Driver._pos_to_angle_f(sid, pos)
                == pytest.approx(Arm_Driver._pos_to_angle(sid, pos), abs=1.0))


@pytest.mark.parametrize('sid', range(1, 7))
def test_float_conversion_round_trips_the_write_mapping(sid):
    """Angle -> raw -> angle must come back to where it started, within the
    one count that _angle_to_pos's own int() throws away."""
    span = ((arm_driver.ANGLE5_MAX, arm_driver.ANGLE5_MIN) if sid == 5
            else (arm_driver.ANGLE_MAX, arm_driver.ANGLE_MIN))
    counts_per_deg = ((arm_driver.POS5_MAX - arm_driver.POS5_MIN) if sid == 5
                      else (arm_driver.POS_MAX - arm_driver.POS_MIN)
                      ) / (span[0] - span[1])
    for frac in (0.1, 0.37, 0.5, 0.83):
        angle = span[1] + frac * (span[0] - span[1])
        back = Arm_Driver._pos_to_angle_f(sid, Arm_Driver._angle_to_pos(sid, angle))
        assert back == pytest.approx(angle, abs=1.0 / counts_per_deg)


@pytest.mark.parametrize('sid', arm_driver._INVERTED_IDS)
def test_the_two_conversions_genuinely_differ_on_inverted_servos(sid):
    """The float version is not a refactor of the int one -- they disagree.

    _pos_to_angle inverts an already-truncated angle, so it is offset from the
    float map rather than merely coarser. joint_map's zero offsets were
    measured against its output, so it must not be quietly re-rounded.
    """
    mid = (arm_driver.POS_MIN + arm_driver.POS_MAX) // 2
    disagreements = sum(
        1 for pos in range(mid, mid + 200)
        if Arm_Driver._pos_to_angle(sid, pos)
        != round(Arm_Driver._pos_to_angle_f(sid, pos)))
    assert disagreements > 0, (
        'servo %d: the int and float conversions round identically, so the '
        'reason for keeping both no longer holds' % sid)


@pytest.mark.parametrize('sid', [s for s in range(1, 7)
                                 if s not in arm_driver._INVERTED_IDS])
def test_float_conversion_never_strays_past_a_count_on_direct_servos(sid):
    """Where there is no flip, the two must agree to within the truncation."""
    lo = arm_driver.POS5_MIN if sid == 5 else arm_driver.POS_MIN
    for pos in range(lo, lo + 200):
        assert (Arm_Driver._pos_to_angle(sid, pos)
                <= Arm_Driver._pos_to_angle_f(sid, pos) + 1e-9)
        assert (Arm_Driver._pos_to_angle_f(sid, pos)
                - Arm_Driver._pos_to_angle(sid, pos)) < 1.0


def test_resolution_is_far_finer_than_a_whole_degree():
    """One count must be a small fraction of a degree, or float reads buy
    nothing over the int ones they replace."""
    counts_per_deg = ((arm_driver.POS_MAX - arm_driver.POS_MIN)
                      / (arm_driver.ANGLE_MAX - arm_driver.ANGLE_MIN))
    assert counts_per_deg > 1.0
    assert 1.0 / counts_per_deg < 0.5


# ------------------------------------------------------------ batch reads

def test_read6_reports_a_silent_servo_as_none_not_as_zero():
    """A dropped reply must stay visible. /servo_states exists to be
    differenced against /joint_states, and a substituted value is a fabricated
    agreement -- zero degrees is also a perfectly plausible joint angle."""
    d = _driver(timeout=0.001)
    assert d.Arm_serial_servo_read6() == [None] * 6


def test_read6_asks_each_servo_once_by_default():
    """Bounded cost per sample. The 5 retries in Arm_serial_servo_read are
    right for calibration, where a lost reading costs a re-measure; a feedback
    loop would rather see the drop than pay five timeouts to hide it."""
    d = _driver(timeout=0.001)
    d.Arm_serial_servo_read6()
    assert d.ser.written.count(bytes([0xFF, 0xFF])) == 6
