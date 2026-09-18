"""The bridge: reads frames from the Kinect and publishes them to the virtual camera."""

from __future__ import annotations

import ctypes
import threading
import time
from dataclasses import dataclass, field

from . import frames, scanning
from .kinect_native import KinectError, KinectSensor

COINIT_MULTITHREADED = 0x0

# The Kinect drops to 15 fps in low light. The virtual camera is kept at 30
# and the last frame is republished when no new one arrives, otherwise apps
# reading the virtual camera see the stream freeze.
TARGET_FPS = 30

# When the sensor stops sending frames the image stays on the last one and
# everything looks fine on screen. After this long, say so.
FRAME_TIMEOUT = 5.0

# Depth and infrared rely on the infrared emitter, which takes a few seconds
# to come back on after a mode change: the sensor reports itself available
# while sending nothing yet. Waiting for the first frame before declaring the
# stream live avoids a black preview and a 0 fps reading with no visible
# reason.
FIRST_FRAME_TIMEOUT = 15.0


class Mode:
    COLOR = "color"
    COLOR_NO_BACKGROUND = "color_nobg"
    DEPTH = "depth"
    INFRARED = "infrared"
    SCAN_PREVIEW = "scan_preview"


MODE_LABELS = {
    Mode.COLOR: "Colour (1920x1080)",
    Mode.COLOR_NO_BACKGROUND: "Colour, background removed",
    Mode.INFRARED: "Night vision (infrared)",
    Mode.DEPTH: "Depth (512x424)",
    Mode.SCAN_PREVIEW: "3D scan preview",
}

OUTPUT_SIZES = {
    "1280 x 720 (recommended)": (1280, 720),
    "1920 x 1080": (1920, 1080),
    "960 x 540": (960, 540),
    "640 x 360": (640, 360),
}

DEFAULT_OUTPUT_SIZE = "1280 x 720 (recommended)"

# Usable range for background removal. The minimum is the sensor's physical
# limit; the default maximum keeps a person at a desk and leaves the room out.
NEAR_LIMIT_MM = 500
FAR_LIMIT_MM = 4500
DEFAULT_FAR_MM = 2000


class MatteSource:
    DISTANCE = "distance"
    PEOPLE = "people"


MATTE_SOURCES = (MatteSource.DISTANCE, MatteSource.PEOPLE)


class VirtualCamUnavailable(RuntimeError):
    """The virtual camera is not installed, or is already in use."""


@dataclass
class CaptureSettings:
    mode: str = Mode.COLOR
    output_size: tuple = (1280, 720)
    mirror: bool = True
    rotate: bool = False
    background: str = frames.BACKGROUND_BLUR
    matte_source: str = MatteSource.DISTANCE
    near_mm: int = NEAR_LIMIT_MM
    far_mm: int = DEFAULT_FAR_MM
    # Where the 3D preview looks from, in degrees around the vertical axis.
    view_angle: float = 0.0


@dataclass
class _Pipeline:
    """The sources opened for one mode, and how to get an image out of them.

    It always returns the image at the sensor's native resolution, together
    with the foreground matte when the mode calls for one: scaling and
    compositing are left to the caller, which knows the output size.
    """

    sensor: KinectSensor
    settings: CaptureSettings
    notify: object = None
    color: object = None
    depth: object = None
    infrared: object = None
    body_index: object = None
    mapper: object = None
    normalizer: object = None
    _last_depth: object = field(default=None, init=False)
    _last_body: object = field(default=None, init=False)
    _fallback_announced: bool = field(default=False, init=False)

    def open(self):
        mode = self.settings.mode
        if mode in (Mode.COLOR, Mode.COLOR_NO_BACKGROUND):
            self.color = self.sensor.color_stream()
        if mode == Mode.COLOR_NO_BACKGROUND:
            # Depth is always needed: it feeds the coordinate mapper, even
            # when the body index is what decides the matte.
            self.depth = self.sensor.depth_stream()
            self.mapper = self.sensor.coordinate_mapper(self.color.width, self.color.height)
            if self.settings.matte_source == MatteSource.PEOPLE:
                self.body_index = self.sensor.body_index_stream()
        if mode == Mode.DEPTH:
            self.depth = self.sensor.depth_stream()
        if mode == Mode.SCAN_PREVIEW:
            # The same two sources a scan uses, so what the preview shows is
            # what a capture would record.
            self.depth = self.sensor.depth_stream()
            self.mapper = self.sensor.coordinate_mapper()
        if mode == Mode.INFRARED:
            self.infrared = self.sensor.infrared_stream()
            self.normalizer = frames.InfraredNormalizer()
        return self

    def _announce(self, message):
        if self.notify is not None:
            self.notify(message)

    def _color_image(self, raw):
        """Converts, and scales here too when the aspect ratios match.

        The colour camera and the output size are both 16:9, so a direct
        resize is normally enough. If someone adds an output format with a
        different ratio, conversion stays at native resolution and fit()
        takes care of the letterboxing.
        """
        target = self.settings.output_size
        native_ratio = self.color.width / self.color.height
        target_ratio = target[0] / target[1]
        if abs(native_ratio - target_ratio) < 0.01:
            return frames.bgra_to_rgb_resized(raw, target)
        return frames.bgra_to_rgb(raw)

    def next_frame(self):
        """(RGB image, matte or None). A None image means nothing new."""
        mode = self.settings.mode

        if mode == Mode.DEPTH:
            raw = self.depth.read()
            return (None, None) if raw is None else (frames.depth_to_rgb(raw), None)

        if mode == Mode.INFRARED:
            raw = self.infrared.read()
            return (None, None) if raw is None else (self.normalizer.apply(raw), None)

        if mode == Mode.SCAN_PREVIEW:
            if self.depth.read() is None:
                return None, None
            points = self.mapper.map_depth_to_camera(self.depth)
            settings = self.settings
            visible = scanning.subject_mask(
                points, settings.near_mm / 1000.0, settings.far_mm / 1000.0
            )
            return scanning.render_preview(
                points, visible, settings.output_size, settings.view_angle
            ), None

        raw = self.color.read()
        if raw is None:
            return None, None
        image = self._color_image(raw)

        if mode != Mode.COLOR_NO_BACKGROUND:
            return image, None

        # Depth arrives at 30 fps like colour but not in lockstep: reusing
        # the last map beats discarding the colour frame.
        fresh = self.depth.read()
        if fresh is not None:
            self._last_depth = fresh
        if self._last_depth is None:
            return image, None

        points = self.mapper.map_color_to_depth(self.depth)
        return image, self._build_matte(points)

    def _build_matte(self, points):
        """Matte by person where possible, by distance where necessary.

        Body tracking needs to see a reasonably complete figure: sitting too
        close to the sensor it locks onto nobody, and an empty matte would
        mean a screen that is entirely background. In that case it falls
        back to distance, and says so, rather than going black.
        """
        if self.body_index is None:
            return frames.depth_matte(
                points, self._last_depth, self.settings.near_mm, self.settings.far_mm
            )

        fresh = self.body_index.read()
        if fresh is not None:
            self._last_body = fresh

        if self._last_body is not None and frames.has_body(self._last_body):
            if self._fallback_announced:
                self._fallback_announced = False
                self._announce("Person detected: back to silhouette matting.")
            return frames.body_matte(points, self._last_body)

        if not self._fallback_announced:
            self._fallback_announced = True
            self._announce(
                "No person detected, falling back to distance matting. Body "
                "tracking needs to see head and torso, roughly 1.5-3 m from "
                "the sensor."
            )
        return frames.depth_matte(
            points, self._last_depth, self.settings.near_mm, self.settings.far_mm
        )


class CaptureEngine:
    """Owns the capture thread and the state the interface displays.

    The callbacks are invoked from the capture thread: the GUI must marshal
    them onto its own thread.
    """

    def __init__(self, on_status=None, on_error=None, on_stopped=None):
        self._on_status = on_status or (lambda _message: None)
        self._on_error = on_error or (lambda _message: None)
        self._on_stopped = on_stopped or (lambda: None)

        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._preview = None
        self._fps = 0.0
        self._output_fps = 0.0
        self._device_name = ""
        self._settings = CaptureSettings()

    # --- state ----------------------------------------------------------

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def fps(self) -> float:
        """New frames per second actually arriving from the sensor."""
        return self._fps

    @property
    def output_fps(self) -> float:
        """Frames per second published to the virtual camera."""
        return self._output_fps

    @property
    def device_name(self) -> str:
        return self._device_name

    def take_preview(self):
        """The last published frame, or None when there is nothing new."""
        with self._lock:
            preview, self._preview = self._preview, None
        return preview

    def update_live_settings(self, background=None, near_mm=None, far_mm=None,
                             view_angle=None):
        """Adjustments that apply while running, without reopening the sensor."""
        if background is not None:
            self._settings.background = background
        if near_mm is not None:
            self._settings.near_mm = near_mm
        if far_mm is not None:
            self._settings.far_mm = far_mm
        if view_angle is not None:
            self._settings.view_angle = view_angle

    # --- control --------------------------------------------------------

    def start(self, settings: CaptureSettings):
        if self.running:
            return
        self._settings = settings
        self._stop.clear()
        self._fps = 0.0
        self._thread = threading.Thread(
            target=self._run, name="kinectcam-capture", daemon=True
        )
        self._thread.start()

    def stop(self, timeout: float = 4.0):
        self._stop.set()
        thread = self._thread
        if thread and thread.is_alive():
            thread.join(timeout)
        self._thread = None

    # --- capture thread -------------------------------------------------

    def _run(self):
        # The Kinect's COM interfaces must be used on the thread that created them.
        ctypes.windll.ole32.CoInitializeEx(None, COINIT_MULTITHREADED)
        sensor = None
        try:
            self._on_status("Opening the sensor...")
            sensor = KinectSensor()
            sensor.open()
            if not sensor.wait_until_available(timeout=6.0):
                raise KinectError(
                    "The sensor is not responding. Check that the adapter is "
                    "powered and plugged into a USB 3.0 port."
                )
            pipeline = _Pipeline(sensor, self._settings, notify=self._on_status).open()
            self._on_status("Sensor active, waiting for the first frame...")
            first = self._warm_up(pipeline)
            if first[0] is None:
                if self._stop.is_set():
                    return
                raise KinectError(
                    "The sensor is connected but not streaming. If you just "
                    "changed mode, the infrared emitter can take a few seconds: "
                    "try again. Otherwise check the USB 3.0 cable and the "
                    "adapter's power supply."
                )
            self._pump(pipeline, first)
        except KinectError as exc:
            self._on_error(str(exc))
        except VirtualCamUnavailable as exc:
            self._on_error(str(exc))
        except Exception as exc:  # noqa: BLE001 - the GUI surfaces any failure
            self._on_error(f"Unexpected error: {exc}")
        finally:
            if sensor is not None:
                sensor.close()
            ctypes.windll.ole32.CoUninitialize()
            self._on_stopped()

    def _poll_for_frame(self, pipeline, deadline):
        while time.monotonic() < deadline and not self._stop.is_set():
            image, matte = pipeline.next_frame()
            if image is not None:
                return image, matte
            time.sleep(0.01)
        return None, None

    def _warm_up(self, pipeline):
        """Waits for the sensor to actually stream, not merely to respond.

        As soon as a stream opens, the runtime hands over the frame left in
        memory from the previous session, and only seconds later do real
        ones start arriving. Stopping at the first frame would start the
        count during the silence and show 0 fps over an image that looks
        alive. The second frame, by contrast, has to be new, because in the
        meantime AcquireLatestFrame has nothing else to give.

        The first frame is still shown in the preview straight away, so the
        wait is not a black screen.
        """
        deadline = time.monotonic() + FIRST_FRAME_TIMEOUT
        first = self._poll_for_frame(pipeline, deadline)
        if first[0] is None:
            return first

        with self._lock:
            self._preview = self._compose(*first)

        self._on_status("Waiting for the sensor to start streaming...")
        second = self._poll_for_frame(pipeline, deadline)
        return second if second[0] is not None else first

    def _compose(self, image, matte):
        settings = self._settings
        output = frames.fit(image, settings.output_size)
        if matte is not None:
            refined = frames.refine_matte(matte, settings.output_size)
            output = frames.apply_background(output, refined, settings.background)
        if settings.rotate:
            output = frames.rotate_180(output)
        if settings.mirror:
            output = frames.mirror(output)
        return frames.to_contiguous(output)

    def _pump(self, pipeline, first_frame):
        import pyvirtualcam

        width, height = self._settings.output_size
        try:
            cam_context = pyvirtualcam.Camera(
                width=width, height=height, fps=TARGET_FPS,
                fmt=pyvirtualcam.PixelFormat.RGB, print_fps=False,
            )
        except RuntimeError as exc:
            raise VirtualCamUnavailable(
                "Virtual camera unavailable. Install OBS Studio (it provides "
                "OBS Virtual Camera) and make sure nothing else is using it.\n\n"
                f"Details: {exc}"
            ) from exc

        with cam_context as cam:
            self._device_name = cam.device
            self._on_status(f"Live on: {cam.device}")

            # The first frame is already in hand, so there is something to
            # show immediately and the count excludes the startup wait.
            last_output = self._compose(*first_frame)
            with self._lock:
                self._preview = last_output

            # New frames are counted, not sent ones: the virtual camera always
            # outputs 30 fps because it republishes the last frame, so counting
            # sends would show a flat 30 even with a stalled sensor. What
            # matters is the true capture rate.
            fresh_frames = 0
            sent_frames = 0
            window_start = time.monotonic()
            last_fresh_at = window_start
            starved = False

            while not self._stop.is_set():
                image, matte = pipeline.next_frame()
                if image is not None:
                    last_output = self._compose(image, matte)
                    fresh_frames += 1
                    last_fresh_at = time.monotonic()
                    if starved:
                        starved = False
                        self._on_status(f"Frames arriving again on {cam.device}.")
                    with self._lock:
                        self._preview = last_output
                elif not starved and time.monotonic() - last_fresh_at > FRAME_TIMEOUT:
                    starved = True
                    self._on_status(
                        "No frames from the sensor for several seconds: the image "
                        "is frozen on the last one. Check the USB 3.0 cable and "
                        "the adapter's power supply."
                    )

                if last_output is None:
                    # Nothing yet: publish nothing and try again.
                    time.sleep(0.01)
                    continue

                cam.send(last_output)
                cam.sleep_until_next_frame()
                sent_frames += 1

                elapsed = time.monotonic() - window_start
                if elapsed >= 1.0:
                    self._fps = fresh_frames / elapsed
                    self._output_fps = sent_frames / elapsed
                    fresh_frames = 0
                    sent_frames = 0
                    window_start = time.monotonic()
