#!/usr/bin/env python3
# coding: utf-8
"""
Who else has the servo bus open.

pyserial takes no exclusive lock, so a second process on /dev/ttyTHS1 does not
fail -- it interleaves bytes with the first and both get corrupt replies. The
symptom is indistinguishable from a powered-off arm, and it is easy to cause by
accident: killing a `ros2 launch` kills the launcher, not always its children.

Lifted out of joint_state_mirror, which had the only copy, because every node
that opens the port wants to say the same thing when reads start failing --
moveit_bridge, measure_bus, gui_teleop, calibrate_zero.
"""

import os


def port_rivals(port):
    """Other processes holding `port`, as a list of 'name (pid N)' strings.

    Empty if none, and empty rather than raising if /proc cannot be walked --
    this is a diagnostic, and failing to produce one must not become a second
    fault on top of the one being diagnosed.
    """
    me = os.getpid()
    rivals = []
    try:
        pids = os.listdir('/proc')
    except OSError:
        return []
    for pid in pids:
        if not pid.isdigit() or int(pid) == me:
            continue
        fd_dir = '/proc/%s/fd' % pid
        try:
            holds = any(os.readlink(os.path.join(fd_dir, fd)) == port
                        for fd in os.listdir(fd_dir))
        except OSError:
            continue
        if not holds:
            continue
        try:
            with open('/proc/%s/cmdline' % pid, 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode().strip()
        except OSError:
            cmd = ''
        rivals.append('%s (pid %s)' % (cmd.split()[-1] if cmd else '?', pid))
    return rivals


def rival_warning(port):
    """A ready-to-log sentence naming the rivals, or '' if the port is ours."""
    rivals = port_rivals(port)
    if not rivals:
        return ''
    return (' ANOTHER PROCESS ALSO HAS %s OPEN: %s -- two nodes on one bus '
            "corrupt each other's replies and look identical to a dead arm. "
            'Kill it first.' % (port, ', '.join(rivals)))
