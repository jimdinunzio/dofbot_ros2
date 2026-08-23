"""
Measure what the servo bus can do. READ-ONLY: never commands a position, never
touches torque. Safe to run on a powered arm at rest.

    ros2 run dofbot_ctrl measure_bus
    ros2 run dofbot_ctrl measure_bus -- --gap-scan --no-flush
    ros2 run dofbot_ctrl measure_bus -- --only sweep

What it reports:

  latency     per-servo round trip and drop rate. Querying ONE servo back to
              back is not how anything reads the arm, so treat this as a
              diagnostic and quote the sweep instead.
  sweep       sustained all-six sweeps -- the honest feedback rate.
  gap-scan    drop rate against the gap left between two queries to the same
              servo. A servo re-queried too soon does not answer, which sets a
              per-JOINT poll ceiling no bus speed can lift.
  no-flush    whether the pre-write input flush is causing dropped replies.
  resolution  raw-count spread of a STATIONARY arm: the noise floor a
              following-error threshold has to clear.

Measured 2026-08-22: 82 Hz sustained sweep, no drops; per-joint recovery gap
8-10 ms; noise floor 1 count (0.082 deg).
"""

import argparse
import statistics
import sys
import time

from Arm_Lib import Arm_Device
from Arm_Lib.arm_driver import _ADDR_READ_POS, _CMD_READ, REPLY_TIMEOUT

from dofbot_ctrl.serial_port import rival_warning

SERVOS = tuple(range(1, 7))

# Raw counts per degree, from the driver's own calibration constants, so this
# tracks a re-range instead of restating one.
from Arm_Lib.arm_driver import (ANGLE_MAX, ANGLE_MIN, ANGLE5_MAX, ANGLE5_MIN,
                                POS_MAX, POS_MIN, POS5_MAX, POS5_MIN)

COUNTS_PER_DEG = {sid: (POS_MAX - POS_MIN) / (ANGLE_MAX - ANGLE_MIN)
                  for sid in SERVOS}
COUNTS_PER_DEG[5] = (POS5_MAX - POS5_MIN) / (ANGLE5_MAX - ANGLE5_MIN)


def _stats(samples):
    """min / median / p95 / max of a sample list, in the list's own units."""
    if not samples:
        return None
    ordered = sorted(samples)
    p95 = ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))]
    return (ordered[0], statistics.median(ordered), p95, ordered[-1])


def _fmt_ms(stats):
    if stats is None:
        return '      --        --        --        --'
    return ''.join('%9.2f' % (s * 1000.0) for s in stats)


def _legacy_query(arm, sid):
    """The read path exactly as it was: write, then ask for 32 bytes.

    Reaches into the driver on purpose. The point is to time the OLD code
    against the same arm on the same bus in the same run, which a description
    of it cannot do.
    """
    frame = arm._build_frame(sid, _CMD_READ, _ADDR_READ_POS, [0x02])
    with arm.lock:
        arm.ser.write(frame)
        buf = arm.ser.read(32)
    return arm._parse_reply(buf, sid)


def _probe(arm, sid, legacy=False, flush=True):
    """One query, timed. Returns (seconds, answered)."""
    t0 = time.monotonic()
    if legacy:
        pos = _legacy_query(arm, sid)
    elif flush:
        pos = arm._query(sid, _CMD_READ, _ADDR_READ_POS, [0x02], expect_id=sid)
    else:
        pos = _unflushed_query(arm, sid)
    return time.monotonic() - t0, bool(pos)


def _unflushed_query(arm, sid):
    """_query without the pre-write reset_input_buffer, to test whether the
    flush is what drops replies. Not a mode anything should run in."""
    frame = arm._build_frame(sid, _CMD_READ, _ADDR_READ_POS, [0x02])
    deadline = time.monotonic() + arm.reply_timeout
    with arm.lock:
        arm.ser.write(frame)
        buf = b''
        while True:
            chunk = arm.ser.read(max(1, arm.ser.in_waiting))
            if chunk:
                buf += chunk
                val = arm._parse_reply(buf, sid)
                if val is not None:
                    return val
            if time.monotonic() >= deadline:
                return None


def measure_latency(arm, samples, legacy=False, gap=0.0, flush=True):
    """Time single queries per servo. Returns {sid: (stats, drops, n)}.

    `gap` is slept AFTER each query, so it sets how long the servo has been
    left alone before the next one reaches it. That turns out to matter a great
    deal -- see report_gap_scan.
    """
    out = {}
    for sid in SERVOS:
        times, drops = [], 0
        for _ in range(samples):
            dt, ok = _probe(arm, sid, legacy, flush)
            times.append(dt)
            if not ok:
                drops += 1
            if gap:
                time.sleep(gap)
        out[sid] = (_stats(times), drops, samples)
    return out


def report_latency(arm, samples, legacy):
    print('\n== query latency, %d samples/servo ==' % samples)
    print('%-8s %8s %9s %9s %9s %9s  %s'
          % ('servo', 'min ms', 'median', 'p95', 'max', 'drops', ''))

    def show(label, results):
        for sid in SERVOS:
            stats, drops, n = results[sid]
            print('%-8s %s %6d/%-3d  %s'
                  % ('%s %d' % (label, sid), _fmt_ms(stats), drops, n,
                     'NO REPLIES' if drops == n else ''))

    new = measure_latency(arm, samples, legacy=False)
    show('new', new)
    answered = [new[sid][0][0] for sid in SERVOS if new[sid][0]]
    if not answered:
        print('\n  Nothing answered. Arm powered off, or another process has '
              'the port.')
        return None
    print('\n  fastest answered round trip %.2f ms' % (min(answered) * 1000))

    dropping = [sid for sid in SERVOS if new[sid][1]]
    if dropping:
        print('\n  These drops are this test\'s own doing: it queries ONE servo '
              'back to back,\n  which nothing real does, so the median blends '
              'hits with timeouts and is not\n  a round-trip time. Quote the '
              'sweep below instead. Servo(s) %s need a\n  gap -- see '
              '--gap-scan.' % ', '.join(str(s) for s in dropping))

    if legacy:
        old = measure_latency(arm, samples, legacy=True)
        show('old', old)
        old_medians = [old[sid][0][1] for sid in SERVOS if old[sid][0]]
        if old_medians:
            old_sweep = sum(old_medians)
            print('\n  Fixed-32-byte read: %.1f ms per sweep -> %.1f Hz. That '
                  'is ~reply_timeout x 6\n  by construction -- a read that '
                  'cannot return early measures the timeout,\n  not the bus.'
                  % (old_sweep * 1000, 1.0 / old_sweep))
    return None


# Inter-query gaps scanned, in seconds. Dense between 8 and 12 ms because the
# threshold measured 2026-08-22 sits there, and it decides an update_rate.
_GAPS = (0.0, 0.002, 0.005, 0.008, 0.009, 0.010, 0.011, 0.012, 0.020)


def report_gap_scan(arm, samples):
    """Drop rate against the gap left between two queries to the SAME servo.

    A servo re-queried too soon after its last reply does not answer, and the
    knee is the minimum poll interval per joint -- the real ceiling for any
    feedback loop. Round-robin hides it, because five other servos are queried
    in between.
    """
    print('\n== drop rate vs inter-query gap, %d samples/servo ==' % samples)
    print('   (same servo queried repeatedly; the gap is the wait between)')
    print('\n%-10s %s' % ('gap ms', ''.join('%9s' % ('servo %d' % s)
                                             for s in SERVOS)))
    knees = {}
    for gap in _GAPS:
        results = measure_latency(arm, samples, gap=gap)
        cells = []
        for sid in SERVOS:
            _, drops, n = results[sid]
            cells.append('%8.0f%%' % (100.0 * drops / n))
            if drops == 0 and sid not in knees:
                knees[sid] = gap
        print('%-10.0f %s' % (gap * 1000, ''.join(cells)))

    missing = [s for s in SERVOS if s not in knees]
    if missing:
        print('\n  Servo(s) %s dropped replies at EVERY gap tested. That is not '
              'a recovery\n  gap -- try --no-flush, and check for a rival on the '
              'port.'
              % ', '.join(str(s) for s in missing))
        return

    worst = max(knees.values())
    if worst == 0.0:
        print('\n  No servo needs a gap; all six answer back to back.')
        return

    # The gap is measured from reply-received to next-query-sent, because
    # _probe returns as soon as the frame parses and the sleep follows it. So
    # the threshold is bracketed by the last gap that failed and the first that
    # passed -- quoting the passing one alone overstates it.
    below = max([g for g in _GAPS if g < worst], default=0.0)
    print('\n  Every servo answers once left alone for %.0f ms; %.0f ms still '
          'drops.' % (worst * 1000, below * 1000))
    if below:
        print('  So the recovery interval is in (%.0f, %.0f] ms, and the '
              'per-JOINT poll\n  ceiling is %.0f-%.0f Hz.'
              % (below * 1000, worst * 1000, 1.0 / worst, 1.0 / below))
    else:
        print('  So the recovery interval is at most %.0f ms, a per-JOINT poll '
              'ceiling of\n  %.0f Hz or better.' % (worst * 1000, 1.0 / worst))
    print('''
  This is per SERVO, not per bus, so no amount of bus speed lifts it. A sweep
  faster than this interval starts dropping again -- read the sustained figure
  as a rate sitting above a cliff, not as headroom.''')


def report_no_flush(arm, samples):
    """Gap-0 drop rate with and without the pre-write flush, side by side."""
    print('\n== does the input flush cause the drops? %d samples/servo =='
          % samples)
    with_flush = measure_latency(arm, samples, gap=0.0, flush=True)
    without = measure_latency(arm, samples, gap=0.0, flush=False)
    print('\n%-10s %9s %9s' % ('servo', 'flushed', 'unflushed'))
    for sid in SERVOS:
        print('%-10d %8.0f%% %8.0f%%'
              % (sid, 100.0 * with_flush[sid][1] / samples,
                 100.0 * without[sid][1] / samples))
    print('\n  Two similar columns mean the flush is innocent and the servo '
          'needs a gap.\n  A big drop in the second column means the flush is '
          'eating replies, and the\n  pre-write reset_input_buffer needs '
          'rethinking -- do not just remove it, it is\n  what stops a late '
          'reply answering the next query.')


def report_sweep(arm, seconds):
    """Sustained six-servo sweeps -- the honest state-feedback rate."""
    print('\n== sustained sweep, %.0f s ==' % seconds)
    durations = []
    drops = {sid: 0 for sid in SERVOS}
    n = 0
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        t0 = time.monotonic()
        vals = arm.Arm_serial_servo_read6()
        durations.append(time.monotonic() - t0)
        n += 1
        for sid, v in zip(SERVOS, vals):
            if v is None:
                drops[sid] += 1

    stats = _stats(durations)
    if stats is None:
        print('  no sweeps completed')
        return None
    lo, med, p95, hi = stats
    print('  sweeps      %d' % n)
    print('  per sweep   min %.1f ms, median %.1f ms, p95 %.1f ms, max %.1f ms'
          % (lo * 1000, med * 1000, p95 * 1000, hi * 1000))
    print('  rate        %.1f Hz median, %.1f Hz at p95' % (1.0 / med, 1.0 / p95))
    bad = {sid: c for sid, c in drops.items() if c}
    print('  drops       %s'
          % (', '.join('servo %d: %d/%d' % (s, c, n)
                       for s, c in sorted(bad.items())) if bad else 'none'))
    print('\n  Pace a feedback loop off the p95, not the median: a read that '
          'sometimes\n  overruns its cycle is a controller that sometimes '
          'misses one.')
    return med


def report_resolution(arm, samples):
    """Raw-count spread with the arm held still -- the noise floor."""
    print('\n== resolution and noise at rest, %d samples ==' % samples)
    print('  DO NOT TOUCH THE ARM while this runs; movement reads as noise.')
    series = {sid: [] for sid in SERVOS}
    for _ in range(samples):
        for sid, pos in zip(SERVOS, arm.Arm_serial_servo_read6_raw()):
            if pos is not None:
                series[sid].append(pos)

    print('\n%-8s %8s %8s %10s %12s' % ('servo', 'n', 'spread', 'spread', 'stdev'))
    print('%-8s %8s %8s %10s %12s' % ('', '', 'counts', 'deg', 'deg'))
    for sid in SERVOS:
        vals = series[sid]
        if not vals:
            print('%-8d %8s %8s %10s %12s' % (sid, '0', '--', '--', '--'))
            continue
        spread = max(vals) - min(vals)
        cpd = COUNTS_PER_DEG[sid]
        sd = statistics.pstdev(vals) if len(vals) > 1 else 0.0
        print('%-8d %8d %8d %10.3f %12.3f'
              % (sid, len(vals), spread, spread / cpd, sd / cpd))
    print('\n  One count is %.3f deg on servos 1-4/6 and %.3f deg on servo 5.'
          % (1.0 / COUNTS_PER_DEG[1], 1.0 / COUNTS_PER_DEG[5]))
    print('  A following-error threshold has to clear the spread column, or it '
          'fires\n  on a stationary arm.')


def main(args=None):
    ap = argparse.ArgumentParser(
        description='Measure servo-bus read performance. Read-only.')
    ap.add_argument('--port', default='/dev/ttyTHS1')
    ap.add_argument('--samples', type=int, default=100,
                    help='samples per servo for latency and resolution')
    ap.add_argument('--seconds', type=float, default=5.0,
                    help='duration of the sustained sweep test')
    ap.add_argument('--legacy', action='store_true',
                    help='also time the old fixed-32-byte read, for comparison')
    ap.add_argument('--only',
                    choices=['latency', 'sweep', 'resolution', 'gap-scan',
                             'no-flush'],
                    action='append',
                    help='run just these (repeatable); default is the first '
                         'three. gap-scan and no-flush diagnose dropped '
                         'replies and are not run unless asked for.')
    ap.add_argument('--gap-scan', action='store_true',
                    help='shorthand for --only gap-scan')
    ap.add_argument('--no-flush', action='store_true',
                    help='shorthand for --only no-flush')
    opts = ap.parse_args(args if args is not None else sys.argv[1:])

    warning = rival_warning(opts.port)
    if warning:
        print('REFUSING TO MEASURE.%s' % warning)
        print('\nEvery number this tool produces would be a measurement of the '
              'contention,\nnot of the bus.')
        return 1

    arm = Arm_Device(com=opts.port)
    print('Reading %s. Nothing is commanded and torque is not touched.'
          % opts.port)

    if not any(arm.Arm_serial_servo_read(sid) is not None for sid in SERVOS):
        print('\nNO REPLY FROM ANY SERVO. The arm is almost certainly POWERED '
              'OFF -- check\nthe switch and the battery.')
        return 1

    which = set(opts.only or [])
    if opts.gap_scan:
        which.add('gap-scan')
    if opts.no_flush:
        which.add('no-flush')
    if not which:
        which = {'latency', 'sweep', 'resolution'}

    if 'latency' in which:
        report_latency(arm, opts.samples, opts.legacy)
    if 'sweep' in which:
        report_sweep(arm, opts.seconds)
    if 'gap-scan' in which:
        report_gap_scan(arm, opts.samples)
    if 'no-flush' in which:
        report_no_flush(arm, opts.samples)
    if 'resolution' in which:
        report_resolution(arm, opts.samples)
    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
