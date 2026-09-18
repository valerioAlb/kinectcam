"""Access to the Kinect v2 sensor through Kinect20.dll (Kinect for Windows Runtime 2.x).

COM vtables are called directly with ctypes rather than through comtypes:
fewer dependencies to package and no type library generation at runtime. The
vtable indices come from the interfaces declared in Kinect.h of SDK 2.0; the
first three slots are always IUnknown's (QueryInterface, AddRef, Release), so
an interface's own methods start at index 3.
"""

from __future__ import annotations

import ctypes
import time
from ctypes import (
    POINTER, byref, c_bool, c_float, c_int, c_long, c_longlong, c_ubyte, c_uint,
    c_ushort, c_void_p,
)

import numpy as np

E_PENDING = -0x7FFFFFF6  # 0x8000000A: no new frame available

COLOR_FORMAT_BGRA = 3  # ColorImageFormat_Bgra


class KinectError(RuntimeError):
    """A failure in the Kinect runtime or in the sensor itself."""


# --- vtable indices -----------------------------------------------------

class _Sensor:
    OPEN = 6
    CLOSE = 7
    IS_OPEN = 8
    IS_AVAILABLE = 9
    COLOR_SOURCE = 10
    DEPTH_SOURCE = 11
    BODY_INDEX_SOURCE = 13
    INFRARED_SOURCE = 14
    COORDINATE_MAPPER = 18


# ICoordinateMapper::MapColorFrameToDepthSpace
_MAP_COLOR_FRAME_TO_DEPTH_SPACE = 16

# ICoordinateMapper::MapDepthFrameToCameraSpace. A CameraSpacePoint is three
# floats in metres, so the runtime does the unprojection with the sensor's
# factory calibration and nothing here has to model the optics.
_MAP_DEPTH_FRAME_TO_CAMERA_SPACE = 14


class _Desc:
    WIDTH = 3
    HEIGHT = 4


# FrameDescription sits at a different index on every source, because
# IDepthFrameSource declares its two reliable-distance properties first and
# IInfraredFrameSource does not.
_OPEN_READER = 7
_COLOR_SOURCE_DESC = 9
_DEPTH_SOURCE_DESC = 10
_INFRARED_SOURCE_DESC = 8
_BODY_INDEX_SOURCE_DESC = 8

_READER_ACQUIRE = 6

_COLOR_FRAME_COPY_CONVERTED = 7
_PLAIN_FRAME_COPY = 3  # IDepthFrame / IInfraredFrame: CopyFrameDataToArray

# get_RelativeTime: the sensor's own clock, in 100 ns units. It sits at a
# different slot for colour because IColorFrame declares more methods first.
_COLOR_FRAME_RELATIVE_TIME = 10
_PLAIN_FRAME_RELATIVE_TIME = 6

# One RelativeTime tick is 100 ns.
TICKS_PER_SECOND = 10_000_000


# --- loading the runtime ------------------------------------------------

_dll = None


def _dll_handle():
    global _dll
    if _dll is None:
        try:
            lib = ctypes.WinDLL("Kinect20.dll")
        except OSError as exc:
            raise KinectError(
                "Kinect20.dll not found: the Kinect for Windows Runtime 2.x "
                "does not appear to be installed on this machine."
            ) from exc
        lib.GetDefaultKinectSensor.restype = c_long
        lib.GetDefaultKinectSensor.argtypes = [POINTER(c_void_p)]
        _dll = lib
    return _dll


def runtime_installed() -> bool:
    """True when Kinect20.dll can be loaded, i.e. the runtime is present."""
    try:
        _dll_handle()
    except KinectError:
        return False
    return True


# --- minimal COM wrapper ------------------------------------------------

_LPVTBL = POINTER(POINTER(c_void_p))


def _check(hr: int, what: str) -> None:
    if hr < 0:
        raise KinectError(f"{what} failed (HRESULT 0x{hr & 0xFFFFFFFF:08X})")


class _Com:
    """A COM interface pointer, callable by vtable index."""

    __slots__ = ("_ptr",)

    def __init__(self, ptr):
        self._ptr = ptr

    def __bool__(self) -> bool:
        return bool(self._ptr)

    def _invoke(self, index, argtypes, args, what):
        if not self._ptr:
            raise KinectError(what + ": interface already released")
        vtbl = ctypes.cast(self._ptr, _LPVTBL).contents
        proto = ctypes.WINFUNCTYPE(c_long, c_void_p, *argtypes)
        return proto(vtbl[index])(self._ptr, *args)

    def call(self, index, argtypes, *args, what="COM call"):
        _check(self._invoke(index, argtypes, args, what), what)

    def call_raw(self, index, argtypes, *args, what="COM call"):
        return self._invoke(index, argtypes, args, what)

    def iface(self, index, what="interface lookup"):
        out = c_void_p()
        self.call(index, [POINTER(c_void_p)], byref(out), what=what)
        if not out:
            raise KinectError(what + ": the runtime returned a null interface")
        return _Com(out)

    def get_value(self, index, ctype=c_int, what="property read"):
        out = ctype()
        self.call(index, [POINTER(ctype)], byref(out), what=what)
        return out.value

    def release(self):
        if self._ptr:
            vtbl = ctypes.cast(self._ptr, _LPVTBL).contents
            ctypes.WINFUNCTYPE(ctypes.c_ulong, c_void_p)(vtbl[2])(self._ptr)
            self._ptr = None


# --- streams ------------------------------------------------------------

class _Stream:
    """A reader over one frame source, reusing a single buffer across reads.

    The array returned by read() is always the same object: consume it (or
    copy it) before the next read.
    """

    # Slot of get_RelativeTime on the frame interface; colour differs.
    _RELATIVE_TIME_SLOT = _PLAIN_FRAME_RELATIVE_TIME

    def __init__(self, source, desc_index, shape_of, dtype, element_type):
        self._source = source
        self._reader = None
        # The sensor's timestamp for the last frame read, in 100 ns ticks.
        # It tells a frame that was never produced from one lost in transit.
        self.relative_time = 0
        desc = source.iface(desc_index, what="FrameDescription")
        try:
            self.width = desc.get_value(_Desc.WIDTH, what="Width")
            self.height = desc.get_value(_Desc.HEIGHT, what="Height")
        finally:
            desc.release()
        self._reader = source.iface(_OPEN_READER, what="OpenReader")
        self.buffer = np.zeros(shape_of(self.height, self.width), dtype=dtype)
        self.data_pointer = self.buffer.ctypes.data_as(POINTER(element_type))
        self._ptr = self.data_pointer
        self._capacity = self.buffer.size  # element count, not bytes

    def _acquire(self):
        """The most recent COM frame, or None when there is nothing new."""
        frame_ptr = c_void_p()
        hr = self._reader.call_raw(
            _READER_ACQUIRE, [POINTER(c_void_p)], byref(frame_ptr),
            what="AcquireLatestFrame",
        )
        if hr == E_PENDING or not frame_ptr:
            return None
        _check(hr, "AcquireLatestFrame")
        return _Com(frame_ptr)

    def _read_relative_time(self, frame):
        self.relative_time = frame.get_value(
            self._RELATIVE_TIME_SLOT, c_longlong, what="get_RelativeTime"
        )

    def close(self):
        if self._reader:
            self._reader.release()
            self._reader = None
        if self._source:
            self._source.release()
            self._source = None


class ColorStream(_Stream):
    """The 1920x1080 colour stream. read() returns a BGRA array (H, W, 4)."""

    _RELATIVE_TIME_SLOT = _COLOR_FRAME_RELATIVE_TIME

    def __init__(self, source):
        super().__init__(
            source, _COLOR_SOURCE_DESC,
            lambda h, w: (h, w, 4), np.uint8, c_ubyte,
        )

    def read(self):
        frame = self._acquire()
        if frame is None:
            return None
        try:
            frame.call(
                _COLOR_FRAME_COPY_CONVERTED,
                [c_uint, POINTER(c_ubyte), c_int],
                self._capacity, self._ptr, COLOR_FORMAT_BGRA,
                what="CopyConvertedFrameDataToArray",
            )
            self._read_relative_time(frame)
        finally:
            frame.release()
        return self.buffer


class RawStream(_Stream):
    """A single-channel 512x424 stream.

    Depth and infrared are uint16; the body index is uint8. The three
    interfaces have the same shape and the same vtable slots, so only the
    buffer's element type changes.
    """

    def __init__(self, source, desc_index, dtype=np.uint16, element_type=c_ushort):
        self._element_type = element_type
        super().__init__(
            source, desc_index,
            lambda h, w: (h, w), dtype, element_type,
        )

    def read(self):
        frame = self._acquire()
        if frame is None:
            return None
        try:
            frame.call(
                _PLAIN_FRAME_COPY,
                [c_uint, POINTER(self._element_type)],
                self._capacity, self._ptr,
                what="CopyFrameDataToArray",
            )
            self._read_relative_time(frame)
        finally:
            frame.release()
        return self.buffer


# --- mapping between the two cameras ------------------------------------

class CoordinateMapper:
    """Aligns the colour camera with the depth camera.

    The two cameras are physically separate and differ in resolution and
    field of view, so colour pixel (x, y) is not depth pixel (x, y). The
    runtime knows the sensor's factory calibration and does the conversion.
    """

    def __init__(self, com, color_width: int = 1920, color_height: int = 1080):
        self._com = com
        self._color_size = (color_height, color_width)
        self._color_count = color_width * color_height
        # Both output buffers are allocated on first use. The colour grid
        # alone is eight megabytes, and scanning never asks for it.
        self.points = None
        self._points_ptr = None
        self._camera_points = None
        self._camera_ptr = None

    def map_color_to_depth(self, depth_stream) -> np.ndarray:
        """Where each colour pixel falls in the depth image.

        Returns an (H, W, 2) array of depth-space coordinates. Pixels the
        sensor cannot map are -infinity.
        """
        if self.points is None:
            self.points = np.zeros((*self._color_size, 2), dtype=np.float32)
            self._points_ptr = self.points.ctypes.data_as(POINTER(c_float))

        self._com.call(
            _MAP_COLOR_FRAME_TO_DEPTH_SPACE,
            [c_uint, POINTER(c_ushort), c_uint, POINTER(c_float)],
            depth_stream.buffer.size, depth_stream.data_pointer,
            self._color_count, self._points_ptr,
            what="MapColorFrameToDepthSpace",
        )
        return self.points

    def map_depth_to_camera(self, depth_stream) -> np.ndarray:
        """Every depth pixel as a 3D point in metres.

        Returns an (H, W, 3) array in the sensor's camera space: x to the
        right, y up, z away from the lens. Pixels with no depth reading come
        back as -infinity.
        """
        if self._camera_points is None:
            height, width = depth_stream.buffer.shape
            self._camera_points = np.zeros((height, width, 3), dtype=np.float32)
            self._camera_ptr = self._camera_points.ctypes.data_as(POINTER(c_float))

        self._com.call(
            _MAP_DEPTH_FRAME_TO_CAMERA_SPACE,
            [c_uint, POINTER(c_ushort), c_uint, POINTER(c_float)],
            depth_stream.buffer.size, depth_stream.data_pointer,
            self._camera_points.shape[0] * self._camera_points.shape[1],
            self._camera_ptr,
            what="MapDepthFrameToCameraSpace",
        )
        return self._camera_points

    def close(self):
        if self._com:
            self._com.release()
            self._com = None


# --- sensor -------------------------------------------------------------

class KinectSensor:
    """The default Kinect v2 sensor.

    Use it as a context manager, or call close() when done: the runtime
    holds the device busy for as long as the sensor stays open.
    """

    def __init__(self):
        dll = _dll_handle()
        ptr = c_void_p()
        _check(dll.GetDefaultKinectSensor(byref(ptr)), "GetDefaultKinectSensor")
        if not ptr:
            raise KinectError(
                "The runtime exposes no sensor: check that the Kinect is "
                "attached to its adapter and to a USB 3.0 port."
            )
        self._com = _Com(ptr)
        self._streams = []
        self._opened = False

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_exc):
        self.close()

    def open(self):
        if not self._opened:
            self._com.call(_Sensor.OPEN, [], what="IKinectSensor::Open")
            self._opened = True

    @property
    def is_available(self) -> bool:
        """True once the sensor is physically connected and powered."""
        return bool(self._com.get_value(_Sensor.IS_AVAILABLE, c_bool, what="IsAvailable"))

    def wait_until_available(self, timeout: float = 5.0) -> bool:
        """The sensor takes a few seconds to come up after Open()."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.is_available:
                return True
            time.sleep(0.15)
        return self.is_available

    def _track(self, stream):
        self._streams.append(stream)
        return stream

    def color_stream(self) -> ColorStream:
        source = self._com.iface(_Sensor.COLOR_SOURCE, what="ColorFrameSource")
        return self._track(ColorStream(source))

    def depth_stream(self) -> RawStream:
        source = self._com.iface(_Sensor.DEPTH_SOURCE, what="DepthFrameSource")
        return self._track(RawStream(source, _DEPTH_SOURCE_DESC))

    def infrared_stream(self) -> RawStream:
        source = self._com.iface(_Sensor.INFRARED_SOURCE, what="InfraredFrameSource")
        return self._track(RawStream(source, _INFRARED_SOURCE_DESC))

    def body_index_stream(self) -> RawStream:
        """One byte per pixel: 0-5 identify a person, 255 is background.

        Opening this source starts the runtime's body tracking, which costs
        more than depth alone but tells people apart from furniture instead
        of cutting at a fixed distance.
        """
        source = self._com.iface(_Sensor.BODY_INDEX_SOURCE, what="BodyIndexFrameSource")
        return self._track(
            RawStream(source, _BODY_INDEX_SOURCE_DESC, dtype=np.uint8, element_type=c_ubyte)
        )

    def coordinate_mapper(self, color_width: int = 1920, color_height: int = 1080):
        com = self._com.iface(_Sensor.COORDINATE_MAPPER, what="CoordinateMapper")
        return self._track(CoordinateMapper(com, color_width, color_height))

    def close(self):
        for stream in self._streams:
            stream.close()
        self._streams.clear()
        if self._opened:
            try:
                self._com.call(_Sensor.CLOSE, [], what="IKinectSensor::Close")
            except KinectError:
                pass
            self._opened = False
        self._com.release()
