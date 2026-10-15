"""X11 window capture/layout only. No renderer, ROS participant or robot inputs."""
import ctypes as C
import subprocess
import re

import numpy as np


class XImage(C.Structure):
    _fields_ = [(n, C.c_int) for n in ('width', 'height', 'xoffset', 'format')] + [
        ('data', C.c_void_p)] + [(n, C.c_int) for n in (
        'byte_order', 'bitmap_unit', 'bitmap_bit_order', 'bitmap_pad', 'depth',
        'bytes_per_line', 'bits_per_pixel')] + [(n, C.c_ulong) for n in (
        'red_mask', 'green_mask', 'blue_mask')]


def find_windows():
    tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True, timeout=3)
    found = {}
    for line in tree.splitlines():
        match = re.match(r'\s*(0x[0-9a-f]+) "([^"]*)"', line)
        if match:
            wid, title = int(match[1], 16), match[2]
            if title == 'MuJoCo': found['MuJoCo'] = wid
            if title == 'T1 Demo - RViz': found['RViz'] = wid
            if title.startswith('W15 display check'): found['MuJoCo'] = wid
    return found


class XWindows:
    def __init__(self):
        self.lib = C.CDLL('libX11.so.6')
        for name, result, args in (
            ('XOpenDisplay', C.c_void_p, [C.c_char_p]),
            ('XCloseDisplay', C.c_int, [C.c_void_p]),
            ('XGetGeometry', C.c_int, [C.c_void_p, C.c_ulong, C.POINTER(C.c_ulong),
                C.POINTER(C.c_int), C.POINTER(C.c_int)] + [C.POINTER(C.c_uint)]*4),
            ('XGetImage', C.POINTER(XImage), [C.c_void_p, C.c_ulong, C.c_int, C.c_int,
                C.c_uint, C.c_uint, C.c_ulong, C.c_int]),
            ('XDestroyImage', C.c_int, [C.POINTER(XImage)]),
            ('XMoveResizeWindow', C.c_int, [C.c_void_p, C.c_ulong, C.c_int, C.c_int, C.c_uint, C.c_uint]),
            ('XFlush', C.c_int, [C.c_void_p]),
        ):
            fn = getattr(self.lib, name); fn.restype = result; fn.argtypes = args
        # A window can disappear between discovery and capture during normal exit.
        self.errors = []
        self.handler_type = C.CFUNCTYPE(C.c_int, C.c_void_p, C.c_void_p)
        self.handler = self.handler_type(lambda *_: self.errors.append('X11 window unavailable') or 0)
        self.lib.XSetErrorHandler.argtypes = [self.handler_type]
        self.lib.XSetErrorHandler(self.handler)
        self.display = self.lib.XOpenDisplay(None)
        if not self.display: raise RuntimeError('Cannot connect to the current X11 display')

    def layout(self, windows):
        for name, (x, width) in {'MuJoCo': (30, 1100), 'RViz': (1140, 760)}.items():
            if name in windows:
                self.lib.XMoveResizeWindow(self.display, windows[name], x, 80, width, 740)
        self.lib.XFlush(self.display)

    def capture(self, window):
        root = C.c_ulong(); x = C.c_int(); y = C.c_int()
        width, height, border, depth = (C.c_uint() for _ in range(4))
        if not self.lib.XGetGeometry(self.display, window, C.byref(root), C.byref(x), C.byref(y),
                C.byref(width), C.byref(height), C.byref(border), C.byref(depth)):
            return None
        if not (1 <= width.value <= 4096 and 1 <= height.value <= 2160):
            raise RuntimeError('Unexpected capture dimensions')
        ptr = self.lib.XGetImage(self.display, window, 0, 0, width, height, C.c_ulong(-1), 2)
        if not ptr: return None
        try:
            im = ptr.contents
            if (im.bits_per_pixel, im.byte_order, im.red_mask, im.green_mask, im.blue_mask) != (32, 0, 0xff0000, 0xff00, 0xff):
                raise RuntimeError('Unsupported X11 pixel format')
            data = C.string_at(im.data, im.bytes_per_line*im.height)
            return np.frombuffer(data, np.uint8).reshape(im.height, im.bytes_per_line)[:, :im.width*4].reshape(im.height, im.width, 4)[:, :, :3].copy()
        finally:
            self.lib.XDestroyImage(ptr)

    def close(self):
        if self.display: self.lib.XCloseDisplay(self.display); self.display = None
