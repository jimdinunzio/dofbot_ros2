"""
How far behind the plan the real arm runs. READ-ONLY: subscribes and prints.

    ros2 param set /moveit_bridge encoder_rate 10.0      # or encoder_rate:=10.0
    ros2 run dofbot_ctrl measure_tracking                # Ctrl-C for the report

Compares /joint_states (what MoveIt commanded) against /servo_states (what the
encoders read), and splits the disagreement three ways:

  moving      raw error mid-slew. What a JointTrajectoryController path
              tolerance would be checked against if state interfaces were real.
  lag         a single fitted delay per joint. An arm tracing the right path
              late is a slow bus; error that survives the fit is the arm not
              doing what it was told.
  settled     error where the command has been parked. THIS is the grasp
              number -- a pick grips after motion stops, so the mid-slew peak
              says nothing about whether the jaws land on the object.

Only samples taken while the arm is moving are scored for the first two; a pick
sits still most of its wall clock, and idle samples agree perfectly.

Measured 2026-08-22: ~225 ms lag, residual 5.5-15.3 mrad against a 1.4 mrad
noise floor -- the arm tracks well, just late.
"""

import argparse
import math
import statistics
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from dofbot_ctrl import dofbot_kinematics as kin
from dofbot_ctrl.joint_map import ARM_JOINT_NAMES

# A joint is "moving" if the command changes by more than this per second.
# Derived, not picked: below _STEP_DEG per second the bridge would not even
# send a packet.
from dofbot_ctrl.moveit_bridge import _STEP_DEG

MOVING_RAD_PER_S = math.radians(_STEP_DEG)

# Lags searched when fitting the delay, in seconds.
_LAG_MAX = 1.0
_LAG_STEP = 0.005

# How long the command must have been parked for a sample to count as settled.
# Wider than the ~225 ms lag measured on this arm, so the tail of an arrival
# cannot leak in and flatter the figures.
SETTLE_QUIET = 0.4


class Tracking(Node):

    def __init__(self):
        super().__init__('measure_tracking')
        self.commanded = []   # (t, {joint: rad}) from /joint_states
        self.measured = []    # (t, {joint: rad}) from /servo_states
        self.create_subscription(JointState, '/joint_states',
                                 lambda m: self._store(m, self.commanded), 50)
        self.create_subscription(JointState, '/servo_states',
                                 lambda m: self._store(m, self.measured), 50)
        self.get_logger().info(
            'Recording /joint_states vs /servo_states. Run a pick, then Ctrl-C.')

    @staticmethod
    def _store(msg, into):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        into.append((t, dict(zip(msg.name, msg.position))))


def _interp(series, joint, t):
    """The commanded angle of `joint` at time `t`, linearly interpolated.

    None outside the series' span -- extrapolating a commanded trajectory
    invents motion that was never asked for, and those samples belong at the
    edges of a recording where the two topics do not overlap.
    """
    lo, hi = 0, len(series) - 1
    if not series or t < series[0][0] or t > series[hi][0]:
        return None
    while lo < hi - 1:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    (t0, a), (t1, b) = series[lo], series[hi]
    if joint not in a or joint not in b:
        return None
    if t1 <= t0:
        return a[joint]
    f = (t - t0) / (t1 - t0)
    return a[joint] + f * (b[joint] - a[joint])


def _moving_at(commanded, joint, t):
    """Is the commanded trajectory changing at time `t`?"""
    ahead = _interp(commanded, joint, t + 0.05)
    behind = _interp(commanded, joint, t - 0.05)
    return (ahead is not None and behind is not None
            and abs(ahead - behind) / 0.1 > MOVING_RAD_PER_S)


def _pairs(commanded, measured, joint, lag=0.0):
    """[(t, commanded, measured, moving)] for one joint, encoder shifted by `lag`.

    `lag` is how far the arm is assumed to trail: a measurement stamped t is
    compared against what was commanded at t - lag.
    """
    out = []
    for t, vals in measured:
        if joint not in vals:
            continue
        want = _interp(commanded, joint, t - lag)
        if want is None:
            continue
        out.append((t, want, vals[joint], _moving_at(commanded, joint, t - lag)))
    return out


def _settled(commanded, measured, joint, quiet=SETTLE_QUIET):
    """[(commanded, measured)] at samples where the command has been PARKED.

    A sample counts only if the trajectory was still `quiet` seconds earlier
    too, so the tail of an arrival is excluded and what is left is the arm
    standing where it was told to stand.
    """
    out = []
    for t, want, got, moving in _pairs(commanded, measured, joint):
        if moving or _moving_at(commanded, joint, t - quiet):
            continue
        out.append((want, got))
    return out


def _rms(errors):
    return math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else 0.0


def _fit_lag(commanded, measured, joint):
    """(best lag, residual RMS at it) by direct search over the lag range."""
    best = (0.0, None)
    lag = 0.0
    while lag <= _LAG_MAX:
        errs = [m - c
                for _, c, m, moving in _pairs(commanded, measured, joint, lag)
                if moving]
        if errs:
            r = _rms(errs)
            if best[1] is None or r < best[1]:
                best = (lag, r)
        lag += _LAG_STEP
    return best


def diagnose(node):
    """Why /servo_states is silent. Three causes, three fixes, and they look
    identical from here -- so establish which it is rather than guess."""
    if not rclpy.ok():
        return ('The ROS context was already shut down, so nothing could be '
                'checked. Most\n  likely: ros2 param set /moveit_bridge '
                'encoder_rate 10.0')
    lines = []
    publishers = node.count_publishers('/servo_states')
    bridge_up = any(name == 'moveit_bridge'
                    for name, _ in node.get_node_names_and_namespaces())

    if not bridge_up:
        lines.append(
            'moveit_bridge IS NOT RUNNING. Nothing can read the encoders '
            'without it,\n  because it owns the serial port. In simulation '
            '(bridge:=false) there are no\n  encoders to read and this tool '
            'has nothing to measure.')
    elif publishers == 0:
        lines.append(
            'moveit_bridge is running but nothing publishes /servo_states. '
            'That means an\n  OLD bridge, from before the topic existed -- '
            'rebuild and relaunch.')
    else:
        rate = _bridge_encoder_rate(node)
        if rate == 0.0:
            lines.append(
                'moveit_bridge is running with encoder_rate = 0, so the topic '
                'is advertised\n  but silent. Turn it on -- it applies '
                'immediately:\n'
                '      ros2 param set /moveit_bridge encoder_rate 10.0')
        elif rate is None:
            lines.append(
                'moveit_bridge is running and advertises /servo_states, but '
                'would not report\n  its encoder_rate. Check its log.')
        else:
            lines.append(
                'moveit_bridge says encoder_rate = %.1f Hz and the topic is '
                'advertised, yet\n  nothing arrived. Check its log for read '
                'failures, and that the arm is\n  powered on.' % rate)
    return '\n\n'.join(lines)


def _bridge_encoder_rate(node):
    """The bridge's encoder_rate, or None if it will not say."""
    from rcl_interfaces.srv import GetParameters
    client = node.create_client(GetParameters, '/moveit_bridge/get_parameters')
    if not client.wait_for_service(timeout_sec=2.0):
        return None
    future = client.call_async(GetParameters.Request(names=['encoder_rate']))
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    result = future.result()
    if not result or not result.values:
        return None
    return result.values[0].double_value


def report(commanded, measured, node=None):
    if not measured:
        print('\nNO /servo_states RECEIVED -- nothing to compare.\n')
        print('  %s' % (diagnose(node) if node is not None else
                        'Turn the read-back on: ros2 param set '
                        '/moveit_bridge encoder_rate 10.0'))
        return
    if not commanded:
        print('\nNo /joint_states received -- nothing to compare against.')
        return

    span = measured[-1][0] - measured[0][0]
    print('\n== tracking error ==')
    print('%d encoder samples over %.1f s (%.1f Hz), %d commanded samples'
          % (len(measured), span, len(measured) / span if span else 0.0,
             len(commanded)))

    print('\n%-12s %10s %10s %10s %10s %10s'
          % ('joint', 'peak', 'rms', 'best lag', 'residual', 'samples'))
    print('%-12s %10s %10s %10s %10s %10s'
          % ('', 'mrad', 'mrad', 'ms', 'rms mrad', 'moving'))

    any_moving = False
    for joint in ARM_JOINT_NAMES:
        pairs = _pairs(commanded, measured, joint)
        moving = [(c, m) for _, c, m, mv in pairs if mv]
        if not moving:
            print('%-12s %10s %10s %10s %10s %10d'
                  % (joint, '--', '--', '--', '--', 0))
            continue
        any_moving = True
        errs = [m - c for c, m in moving]
        lag, residual = _fit_lag(commanded, measured, joint)
        print('%-12s %10.1f %10.1f %10.0f %10.1f %10d'
              % (joint, max(abs(e) for e in errs) * 1000, _rms(errs) * 1000,
                 lag * 1000, (residual or 0.0) * 1000, len(moving)))

    if not any_moving:
        print('\nThe arm never moved during the recording, so there is no '
              'tracking to measure.\nStart this before the pick, not after.')
        return

    _report_settled(commanded, measured)
    _report_tcp(commanded, measured)

    print('\nPeak/rms are what a JointTrajectoryController path tolerance would '
          'be checked\nagainst if the state interfaces were real. Residual is '
          'what is left once a\nconstant delay per joint is taken out -- error '
          'that is lag, not deviation.')


def _report_settled(commanded, measured):
    """Error where the arm is PARKED. This is the grasp-relevant figure.

    Split into bias and spread, because they have opposite fixes and the same
    rms. A one-signed offset is the encoder and the command disagreeing about
    where zero is -- joint_map's offsets were measured through the whole-degree
    read, so they carry up to +-0.5 deg (8.7 mrad) of quantisation, which is the
    same size as the errors here. Spread around that is the arm genuinely not
    parking twice in the same place: deadband, backlash, or sag under load.

    Only the first is worth recalibrating, and NOTHING should be trimmed
    against a biased measurement.
    """
    print('\n%-12s %9s %9s %9s %9s' % ('joint', 'bias', 'spread', 'peak',
                                        'samples'))
    print('%-12s %9s %9s %9s %9s' % ('(settled)', 'mrad', 'mrad', 'mrad',
                                     'parked'))
    biases, spreads = [], []
    for joint in ARM_JOINT_NAMES:
        pairs = _settled(commanded, measured, joint)
        if not pairs:
            print('%-12s %9s %9s %9s %9d' % (joint, '--', '--', '--', 0))
            continue
        errs = [m - c for c, m in pairs]
        bias = statistics.fmean(errs)
        spread = statistics.pstdev(errs) if len(errs) > 1 else 0.0
        biases.append(abs(bias))
        spreads.append(spread)
        print('%-12s %9.1f %9.1f %9.1f %9d'
              % (joint, bias * 1000, spread * 1000,
                 max(abs(e) for e in errs) * 1000, len(pairs)))

    if not biases:
        return
    print('\nBias is a fixed offset; spread is failure to park twice the same '
          'way.')
    if max(biases) > 2.0 * max(spreads):
        print('BIAS-DOMINATED -- this is a calibration disagreement, not the '
              'arm missing.\nRe-run calibrate_zero: its offsets came from the '
              'whole-degree read and are\nquantised to +-8.7 mrad, the same '
              'size as these numbers.')
    elif max(spreads) > 2.0 * max(biases):
        print('SPREAD-DOMINATED -- the arm does not park twice in the same '
              'place, so this is\ndeadband, backlash or sag. Recalibrating '
              'zero will not touch it.')
    else:
        print('Mixed: some of each. Recalibrating zero addresses the bias '
              'column only.')


def _tcp_errors(commanded, measured, settled_only=False):
    """Tool-frame distance between commanded and measured pose, per sample."""
    errs = []
    for t, vals in measured:
        if not all(j in vals for j in ARM_JOINT_NAMES):
            continue
        want = [_interp(commanded, j, t) for j in ARM_JOINT_NAMES]
        if any(q is None for q in want):
            continue
        if settled_only and any(
                _moving_at(commanded, j, t)
                or _moving_at(commanded, j, t - SETTLE_QUIET)
                for j in ARM_JOINT_NAMES):
            continue
        cx, cy, cz = kin.fk(want)[:3]
        mx, my, mz = kin.fk([vals[j] for j in ARM_JOINT_NAMES])[:3]
        errs.append(math.dist((cx, cy, cz), (mx, my, mz)))
    return errs


def _report_tcp(commanded, measured):
    """The disagreement in millimetres at the tool.

    Split moving from settled: averaging them answers neither question. Only
    the settled figure can move a grasp off the object.
    """
    moving = _tcp_errors(commanded, measured)
    settled = _tcp_errors(commanded, measured, settled_only=True)
    if not moving:
        return
    print('\nAt the tool (our own FK on both sides):')
    print('  mid-slew   peak %6.1f mm, rms %5.1f mm   (%d samples) -- mostly lag'
          % (max(moving) * 1000, _rms(moving) * 1000, len(moving)))
    if settled:
        print('  SETTLED    peak %6.1f mm, rms %5.1f mm   (%d samples) -- '
              'this is the grasp one'
              % (max(settled) * 1000, _rms(settled) * 1000, len(settled)))
        print('\nCompare the SETTLED row against BACK_STOP_CLEARANCE (10 mm) '
              'and the gripper\nCLEARANCE (3 mm). An error near those changes '
              'whether a grasp seats; the\nmid-slew row does not, because '
              'nothing is gripped mid-slew.')
    else:
        print('\nNo settled samples -- the command never stayed parked for '
              '%.1f s. Nothing here\nspeaks to where a grasp would land.'
              % SETTLE_QUIET)


def main(args=None):
    ap = argparse.ArgumentParser(
        description='Compare /joint_states against /servo_states. Read-only.')
    ap.parse_args(args if args is not None else sys.argv[1:])

    rclpy.init()
    node = Tracking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        report(node.commanded, node.measured, node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
