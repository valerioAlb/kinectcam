"""Turning Kinect frames into RGB images ready for the virtual camera."""

from __future__ import annotations

import numpy as np
from PIL import Image, ImageFilter

# Useful range of the time-of-flight sensor, in millimetres. Below the
# minimum and above the maximum the Kinect reports 0, which we render black.
DEPTH_MIN_MM = 500
DEPTH_MAX_MM = 4500

_LUT_SIZE = 1024

# Cold to warm gradient: near in purple/blue, far in yellow/red.
_GRADIENT_STOPS = (
    (0.00, (48, 18, 59)),
    (0.25, (30, 150, 220)),
    (0.50, (60, 220, 120)),
    (0.75, (250, 200, 50)),
    (1.00, (220, 50, 30)),
)


def _build_depth_lut() -> np.ndarray:
    positions = np.array([stop for stop, _ in _GRADIENT_STOPS])
    colors = np.array([rgb for _, rgb in _GRADIENT_STOPS], dtype=np.float64)
    x = np.linspace(0.0, 1.0, _LUT_SIZE)
    lut = np.stack(
        [np.interp(x, positions, colors[:, channel]) for channel in range(3)],
        axis=1,
    )
    return lut.astype(np.uint8)


_DEPTH_LUT = _build_depth_lut()


def bgra_to_rgb(frame: np.ndarray) -> np.ndarray:
    """Drops the alpha channel and reverses the colour channel order."""
    return frame[:, :, 2::-1]


def bgra_to_rgb_resized(frame: np.ndarray, size) -> np.ndarray:
    """From native BGRA to RGB at the requested size, in a single pass.

    Building the numpy BGRA->RGB view first and resizing afterwards is
    expensive: the view has a negative stride, so Pillow has to copy it
    before it can scale it. The "BGRX" raw decoder reorders the channels and
    drops alpha in C, reading the sensor's buffer exactly as it is.
    """
    height, width = frame.shape[:2]
    image = Image.frombuffer("RGB", (width, height), frame, "raw", "BGRX", 0, 1)
    target = tuple(size)
    if (width, height) != target:
        image = image.resize(target, Image.BILINEAR)
    return np.asarray(image)


def depth_to_rgb(frame: np.ndarray) -> np.ndarray:
    """Colours the depth map with the gradient, leaving unmeasured pixels black."""
    span = DEPTH_MAX_MM - DEPTH_MIN_MM
    normalized = (frame.astype(np.int32) - DEPTH_MIN_MM) * (_LUT_SIZE - 1) // span
    indices = np.clip(normalized, 0, _LUT_SIZE - 1)
    rgb = _DEPTH_LUT[indices]
    rgb[frame == 0] = 0
    return rgb


def infrared_to_rgb(frame: np.ndarray, gamma: float = 0.32) -> np.ndarray:
    """Infrared is 16-bit with nearly all its energy low down, so it needs gamma."""
    normalized = (frame.astype(np.float32) / 65535.0) ** gamma
    gray = (normalized * 255.0).astype(np.uint8)
    return np.repeat(gray[:, :, np.newaxis], 3, axis=2)


# Below this reference the image is essentially noise, and normalising
# against it would amplify that noise until it fills the screen.
IR_MIN_REFERENCE = 220.0

# The percentile treated as "white". The absolute maximum will not do: a
# single specular highlight (glasses, metal) would crush everything else to
# black.
IR_WHITE_PERCENTILE = 99.5


class InfraredNormalizer:
    """Night vision: adapts infrared exposure scene by scene.

    The Kinect lights the scene with its own infrared emitter, so it sees in
    complete darkness, but raw brightness varies by orders of magnitude
    between a dark room and a hand held close to the lens. The reference is
    smoothed over time, otherwise every movement makes the image pulse.
    """

    def __init__(self, gamma: float = 0.45, smoothing: float = 0.85):
        self.gamma = gamma
        self.smoothing = smoothing
        self._reference = None

    def reset(self):
        self._reference = None

    def apply(self, frame: np.ndarray) -> np.ndarray:
        measured = max(
            float(np.percentile(frame, IR_WHITE_PERCENTILE)), IR_MIN_REFERENCE
        )
        if self._reference is None:
            self._reference = measured
        else:
            self._reference = (
                self.smoothing * self._reference + (1.0 - self.smoothing) * measured
            )

        scaled = np.clip(frame.astype(np.float32) / self._reference, 0.0, 1.0)
        gray = (np.power(scaled, self.gamma) * 255.0).astype(np.uint8)
        return np.repeat(gray[:, :, np.newaxis], 3, axis=2)


def mirror(frame: np.ndarray) -> np.ndarray:
    """Mirror effect, the way laptop webcams behave."""
    return frame[:, ::-1]


def rotate_180(frame: np.ndarray) -> np.ndarray:
    """For a sensor mounted upside down, which is convenient above a monitor."""
    return frame[::-1, ::-1]


# --- depth-based background removal -------------------------------------

BACKGROUND_BLACK = "black"
BACKGROUND_GREEN = "green"
BACKGROUND_BLUR = "blur"

BACKGROUND_MODES = (BACKGROUND_BLACK, BACKGROUND_GREEN, BACKGROUND_BLUR)

# Chroma key green: saturated and far from skin tones.
GREEN_SCREEN_RGB = (0, 177, 64)

BLUR_RADIUS = 12

# The matte is computed on every third pixel per side. It is a background
# mask, not a precision trace: the edges get softened afterwards anyway, and
# the numpy work drops to a ninth.
MATTE_STRIDE = 3


# Body index value meaning "nobody here".
NO_BODY = 255


def _sample_positions(points, shape, stride):
    """For each sampled colour pixel, where to read in the 512x424 image.

    `points` are the depth-space coordinates produced by the
    CoordinateMapper. Returns indices already clamped to the bounds, plus a
    mask of the ones that are genuinely valid.
    """
    sampled = points[::stride, ::stride]
    height, width = shape

    # Unmappable points are -infinity, and converting an infinity to an
    # integer is undefined. nan_to_num pushes them out of bounds in a single
    # C pass, covering NaN as well: cheaper than isfinite() followed by
    # where() on each axis.
    xi = np.nan_to_num(
        sampled[..., 0], nan=-1.0, posinf=float(width), neginf=-1.0
    ).astype(np.int32)
    yi = np.nan_to_num(
        sampled[..., 1], nan=-1.0, posinf=float(height), neginf=-1.0
    ).astype(np.int32)

    inside = (xi >= 0) & (xi < width) & (yi >= 0) & (yi < height)
    np.clip(xi, 0, width - 1, out=xi)
    np.clip(yi, 0, height - 1, out=yi)
    return yi, xi, inside


def depth_matte(points, depth, near_mm, far_mm, stride: int = MATTE_STRIDE):
    """Foreground by distance: keeps whatever falls inside the given band."""
    yi, xi, inside = _sample_positions(points, depth.shape, stride)
    distance = depth[yi, xi]
    # distance == 0 means "not measured", which is not foreground.
    return inside & (distance >= near_mm) & (distance <= far_mm)


def has_body(body_index: np.ndarray) -> bool:
    """True when the runtime is currently tracking at least one person."""
    return bool((body_index != NO_BODY).any())


def body_matte(points, body_index, stride: int = MATTE_STRIDE):
    """Foreground by person: keeps only pixels belonging to a tracked body.

    Unlike distance it does not cut at a threshold, so the chair and the
    desk disappear even when they are as close as you are, and the edge
    follows the silhouette rather than a cutting plane.
    """
    yi, xi, inside = _sample_positions(points, body_index.shape, stride)
    return inside & (body_index[yi, xi] != NO_BODY)


def refine_matte(matte: np.ndarray, size) -> np.ndarray:
    """Despeckles the matte and brings it to the final resolution.

    Depth is noisy along object edges and leaves isolated pixels. A 3x3 mean
    followed by a threshold removes them: a lone pixel drops to 255/9 and
    never reaches half, while a solid silhouette survives. It costs a third
    of a median filter and does the same job here.
    """
    image = Image.fromarray((matte * 255).astype(np.uint8), mode="L")
    image = image.filter(ImageFilter.BoxBlur(1))
    target = tuple(size)
    if image.size != target:
        image = image.resize(target, Image.BILINEAR)
    return np.asarray(image) >= 128


# A blurred background has no detail to preserve by definition, so a reduced
# copy is blurred and scaled back up: the same result to the eye, but the
# work falls with the square of the factor.
BLUR_DOWNSCALE = 4


def _blurred_background(frame: np.ndarray) -> np.ndarray:
    height, width = frame.shape[:2]
    small = (max(1, width // BLUR_DOWNSCALE), max(1, height // BLUR_DOWNSCALE))
    image = Image.fromarray(frame).resize(small, Image.BILINEAR)
    image = image.filter(ImageFilter.GaussianBlur(BLUR_RADIUS / BLUR_DOWNSCALE))
    return np.asarray(image.resize((width, height), Image.BILINEAR))


def apply_background(frame: np.ndarray, matte: np.ndarray, mode: str) -> np.ndarray:
    """Replaces the background wherever the matte is false."""
    if mode == BACKGROUND_BLUR:
        background = _blurred_background(frame)
    elif mode == BACKGROUND_GREEN:
        background = np.empty_like(frame)
        background[:, :] = GREEN_SCREEN_RGB
    else:
        background = np.zeros_like(frame)
    return np.where(matte[:, :, np.newaxis], frame, background)


def fit(frame: np.ndarray, size) -> np.ndarray:
    """Scales preserving the aspect ratio and pads the rest with black.

    The Kinect is 16:9 on colour and roughly 4:3 on depth and infrared,
    while the virtual camera has one fixed resolution.
    """
    target_w, target_h = size
    height, width = frame.shape[:2]
    if (width, height) == (target_w, target_h):
        return frame

    scale = min(target_w / width, target_h / height)
    new_w = max(1, round(width * scale))
    new_h = max(1, round(height * scale))
    resized = np.asarray(
        Image.fromarray(frame).resize((new_w, new_h), Image.BILINEAR)
    )
    if (new_w, new_h) == (target_w, target_h):
        return resized

    canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)
    top = (target_h - new_h) // 2
    left = (target_w - new_w) // 2
    canvas[top:top + new_h, left:left + new_w] = resized
    return canvas


def to_contiguous(frame: np.ndarray) -> np.ndarray:
    """pyvirtualcam needs a contiguous buffer; reversed views are not."""
    return np.ascontiguousarray(frame)
