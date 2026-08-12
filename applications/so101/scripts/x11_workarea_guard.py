"""Temporarily repair X11 work-area geometry for vertically stacked displays."""

import argparse
import ctypes
import os
import signal
import time


class X11WorkAreaGuard:
    """Restore an X11 root property after Isaac's startup window is created."""

    def __init__(self, x11, display, root, property_atom, cardinal_atom, original):
        self._x11 = x11
        self._display = display
        self._root = root
        self._property_atom = property_atom
        self._cardinal_atom = cardinal_atom
        self._original = original

    def _replace(self, values):
        data = (ctypes.c_ulong * len(values))(*values)
        self._x11.XChangeProperty(
            self._display,
            self._root,
            self._property_atom,
            self._cardinal_atom,
            32,
            0,
            ctypes.cast(data, ctypes.POINTER(ctypes.c_ubyte)),
            len(values),
        )
        self._x11.XSync(self._display, False)

    def restore(self):
        if not self._display:
            return
        try:
            self._replace(self._original)
        finally:
            self._x11.XCloseDisplay(self._display)
            self._display = None


def expand_wrapped_x11_workarea(window_height):
    """Expand a clearly truncated work area and return its restoration guard.

    GNOME can expose a desktop-wide ``_NET_WORKAREA`` ending near the boundary
    between vertically stacked monitors. Kit then subtracts the lower primary
    monitor's origin and wraps the negative result into an unsigned X11 height.
    Normal panel/dock reductions are small and deliberately left untouched.
    """

    x11 = ctypes.CDLL("libX11.so.6")
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
    x11.XDefaultRootWindow.restype = ctypes.c_ulong
    x11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    x11.XInternAtom.restype = ctypes.c_ulong
    x11.XGetGeometry.argtypes = [
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.POINTER(ctypes.c_ulong),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_uint),
        ctypes.POINTER(ctypes.c_uint),
        ctypes.POINTER(ctypes.c_uint),
        ctypes.POINTER(ctypes.c_uint),
    ]
    x11.XGetWindowProperty.argtypes = [
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.c_ulong,
        ctypes.c_long,
        ctypes.c_long,
        ctypes.c_int,
        ctypes.c_ulong,
        ctypes.POINTER(ctypes.c_ulong),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_ulong),
        ctypes.POINTER(ctypes.c_ulong),
        ctypes.POINTER(ctypes.POINTER(ctypes.c_ubyte)),
    ]
    x11.XChangeProperty.argtypes = [
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.c_ulong,
        ctypes.c_ulong,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_ubyte),
        ctypes.c_int,
    ]
    x11.XSync.argtypes = [ctypes.c_void_p, ctypes.c_int]
    x11.XFree.argtypes = [ctypes.c_void_p]
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]

    display = x11.XOpenDisplay(None)
    if not display:
        return None

    root = x11.XDefaultRootWindow(display)
    geometry_root = ctypes.c_ulong()
    root_x = ctypes.c_int()
    root_y = ctypes.c_int()
    root_width = ctypes.c_uint()
    root_height = ctypes.c_uint()
    border_width = ctypes.c_uint()
    depth = ctypes.c_uint()
    if not x11.XGetGeometry(
        display,
        root,
        ctypes.byref(geometry_root),
        ctypes.byref(root_x),
        ctypes.byref(root_y),
        ctypes.byref(root_width),
        ctypes.byref(root_height),
        ctypes.byref(border_width),
        ctypes.byref(depth),
    ):
        x11.XCloseDisplay(display)
        return None

    property_atom = x11.XInternAtom(display, b"_NET_WORKAREA", False)
    cardinal_atom = x11.XInternAtom(display, b"CARDINAL", False)
    actual_type = ctypes.c_ulong()
    actual_format = ctypes.c_int()
    item_count = ctypes.c_ulong()
    bytes_after = ctypes.c_ulong()
    property_data = ctypes.POINTER(ctypes.c_ubyte)()
    status = x11.XGetWindowProperty(
        display,
        root,
        property_atom,
        0,
        1024,
        False,
        cardinal_atom,
        ctypes.byref(actual_type),
        ctypes.byref(actual_format),
        ctypes.byref(item_count),
        ctypes.byref(bytes_after),
        ctypes.byref(property_data),
    )
    if status != 0 or actual_format.value != 32 or item_count.value < 4:
        if property_data:
            x11.XFree(ctypes.cast(property_data, ctypes.c_void_p))
        x11.XCloseDisplay(display)
        return None

    try:
        cardinal_values = ctypes.cast(property_data, ctypes.POINTER(ctypes.c_ulong))
        original = [int(cardinal_values[index]) for index in range(item_count.value)]
    finally:
        x11.XFree(ctypes.cast(property_data, ctypes.c_void_p))

    workarea_height = max(original[index] for index in range(3, len(original), 4))
    truncated_by = int(root_height.value) - workarea_height
    if truncated_by <= max(128, int(window_height) // 2):
        x11.XCloseDisplay(display)
        return None

    expanded = list(original)
    for index in range(0, len(expanded) - 3, 4):
        expanded[index : index + 4] = [0, 0, root_width.value, root_height.value]

    guard = X11WorkAreaGuard(
        x11,
        display,
        root,
        property_atom,
        cardinal_atom,
        original,
    )
    guard._replace(expanded)
    return guard


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--parent-pid", type=int, required=True)
    parser.add_argument("--window-height", type=int, required=True)
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args()

    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    guard = expand_wrapped_x11_workarea(args.window_height)
    if guard is None:
        print("inactive", flush=True)
        return

    print("expanded", flush=True)
    deadline = time.monotonic() + args.timeout
    try:
        while (
            not stop_requested
            and time.monotonic() < deadline
            and os.getppid() == args.parent_pid
        ):
            time.sleep(0.1)
    finally:
        guard.restore()


if __name__ == "__main__":
    main()
