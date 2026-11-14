#!/usr/bin/env python3
# coding: utf-8
"""
Hold the wrist camera open and hand out frames, for as long as the arm is up.

    ros2 run dofbot_ctrl camera_server

pick_place.launch.py starts this and stops it, so the camera streams exactly
while the arm is enabled. It is not a ROS node: no topics, no parameters, only
a socket that vision.grab() asks before it falls back to opening the device
itself.

WHY IT EXISTS. Opening the device for one frame cost about 6 s of every look,
measured 2026-11-12: 1.5 s for OpenCV to build a GStreamer pipeline, 3.1 s to
rebuild it twice for the frame size, 1.2 s of warm-up frames. A stream that is
already running hands over a settled frame in two frame periods: 0.26 s in the
room light it was measured in, where auto-exposure holds the camera near 8 fps.

THE WIRE. A client connects to vision.CAMERA_SOCKET and sends nothing. The
server answers with height, width and channels as three big-endian uint32 and
then the raw BGR bytes, and closes. Raw, not JPEG, so a served frame is the
frame a direct grab would have returned. A connection closed with no bytes
means no frame could be had; the client treats that like no server.

A SERVED FRAME IS NEVER OLDER THAN THE REQUEST. The checks ask right after the
arm stops, and the frame in hand may have been exposed while it was still
moving. See FRESH_FRAMES.
"""

import signal
import socket
import struct
import sys
import threading
import time

from dofbot_ctrl import vision

# Frames that must finish arriving after a request before one is served. The
# first to arrive may have begun exposing before the request was made; the
# second cannot have. Costs one frame period.
FRESH_FRAMES = 2

# Seconds a request waits for its frame before the connection is closed empty.
# Under vision.CAMERA_SERVER_TIMEOUT, so the client hears the refusal rather
# than timing out on silence.
FRAME_WAIT_S = 1.5

# Seconds between attempts to open a camera that would not open or stopped
# reading -- unplugged, or held by a calibrate_view started before the launch.
REOPEN_S = 1.0

HEADER = struct.Struct('!III')


class Stream:
    """The open camera and its latest frame, read on a thread of its own.

    Read continuously, not on demand: the driver queues frames, so a read after
    an idle minute returns one from the start of that minute.
    """

    def __init__(self, opener, warmup=vision.WARMUP_FRAMES, reopen=REOPEN_S):
        self._opener = opener
        self._warmup = warmup
        self._reopen = reopen
        self._changed = threading.Condition()
        self._frame = None
        self._count = 0
        # The count from which frames are settled: auto-exposure starts over
        # each time the device is opened. See vision.WARMUP_FRAMES.
        self._settled = warmup
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=5.0)

    def _read(self):
        camera = None
        try:
            while not self._stop.is_set():
                if camera is None:
                    camera = self._opener()
                    if camera is None:
                        self._stop.wait(self._reopen)
                        continue
                    with self._changed:
                        self._settled = self._count + self._warmup
                    print('wrist camera open', flush=True)
                ok, image = camera.read()
                if not ok:
                    print('wrist camera stopped reading; reopening',
                          file=sys.stderr, flush=True)
                    camera.release()
                    camera = None
                    self._stop.wait(self._reopen)
                    continue
                with self._changed:
                    self._frame = image
                    self._count += 1
                    self._changed.notify_all()
        finally:
            if camera is not None:
                camera.release()

    def fresh(self, timeout=FRAME_WAIT_S):
        """A settled frame exposed after this call, or None within `timeout`."""
        deadline = time.monotonic() + timeout
        with self._changed:
            mark = self._count + FRESH_FRAMES
            while self._count < max(mark, self._settled + 1):
                left = deadline - time.monotonic()
                if left <= 0:
                    return None
                self._changed.wait(left)
            return self._frame


def listen(address=vision.CAMERA_SOCKET):
    """The listening socket. Raises OSError if a server already holds it."""
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(address)
    server.listen(4)
    return server


def serve_one(server, stream):
    """Answer one connection. One at a time: nothing asks concurrently."""
    conn, _ = server.accept()
    with conn:
        conn.settimeout(vision.CAMERA_SERVER_TIMEOUT)
        frame = stream.fresh()
        if frame is None or frame.ndim != 3:
            return
        try:
            conn.sendall(HEADER.pack(*frame.shape) + frame.tobytes())
        except OSError:
            # The client gave up or died. Its problem, not the next one's.
            pass


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    # launch stops its processes with SIGINT and escalates to SIGTERM; both
    # must release the device, or the next open finds it busy. SIGINT is set
    # by hand because a shell that starts this with `&` leaves it ignored,
    # and Python then installs no KeyboardInterrupt of its own.
    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
    try:
        server = listen()
    except OSError as exc:
        print('camera_server: cannot listen (%s) -- is one already running?'
              % exc, file=sys.stderr)
        return 1
    stream = Stream(lambda: vision.open_camera(vision.CAMERA_INDEX,
                                               vision.FRAME_SIZE))
    stream.start()
    try:
        while True:
            serve_one(server, stream)
    except KeyboardInterrupt:
        pass
    finally:
        server.close()
        stream.stop()
        print('wrist camera released', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
