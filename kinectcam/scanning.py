"""Turning depth frames into a 3D model a slicer will accept.

The Kinect sees one side of a subject, so a capture is a height field rather
than a closed object: a face, not a head. Printing needs a solid, so the
surface is given a flat back and a rim joining the two, which turns the shell
into a watertight relief. That is the honest shape of what this sensor can
produce from a single viewpoint, and it prints well.

Distances are handled in metres, the units the runtime works in, and
converted to millimetres only on the way out, because that is what slicers
assume when an STL carries no units of its own.
"""

from __future__ import annotations

import struct
import time
import warnings

import numpy as np

METRES_TO_MM = 1000.0

# Depth is noisy frame to frame. Taking the median across a burst removes
# both the speckle and the occasional wild reading, which a mean would drag
# along with it.
DEFAULT_FRAMES = 30

# Neighbouring pixels further apart than this in depth belong to different
# surfaces, and joining them would stretch a triangle across the gap: it is
# what produces the melted webbing between a face and the wall behind it.
DEFAULT_MAX_STEP_M = 0.02

# How thick to make the backing slab, in millimetres.
DEFAULT_BACKING_MM = 5.0


# Sensible bounds for a subject sitting in front of the sensor. Closer than
# the near limit the time-of-flight reading is unreliable; beyond the far one
# there is usually only the room.
DEFAULT_NEAR_M = 0.5
DEFAULT_FAR_M = 1.2


class ScanError(RuntimeError):
    """The capture did not produce anything that can be turned into a model."""


def capture(frames: int = DEFAULT_FRAMES, near_m: float = DEFAULT_NEAR_M,
            far_m: float = DEFAULT_FAR_M, progress=None):
    """Collects a burst of depth frames and returns 3D points and a subject mask.

    The frames are combined before unprojecting rather than after: the median
    belongs on the raw depth readings, where a dropout is an unambiguous zero.
    """
    from .kinect_native import KinectError, KinectSensor

    def announce(message):
        if progress is not None:
            progress(message)

    sensor = KinectSensor()
    try:
        sensor.open()
        if not sensor.wait_until_available(timeout=8.0):
            raise KinectError("The sensor is not responding: cannot scan.")
        depth = sensor.depth_stream()
        mapper = sensor.coordinate_mapper()

        announce("Waiting for the sensor to settle...")
        collected = []
        deadline = time.monotonic() + 30.0
        while len(collected) < frames and time.monotonic() < deadline:
            frame = depth.read()
            if frame is None:
                time.sleep(0.003)
                continue
            collected.append(frame.copy())
            if len(collected) % 10 == 0:
                announce(f"Captured {len(collected)} of {frames} frames...")

        if len(collected) < 2:
            raise ScanError(
                "The sensor sent almost nothing. Run the freeze hunt: if the "
                "stream keeps dropping, scanning cannot work either."
            )

        announce("Combining frames and building the point cloud...")
        # Writing into the stream's own buffer keeps the pointer the mapper
        # already holds pointing at the data we want mapped.
        depth.buffer[:] = combine_frames(collected)
        points = mapper.map_depth_to_camera(depth).copy()
    finally:
        sensor.close()

    return points, subject_mask(points, near_m, far_m)


def combine_frames(frames) -> np.ndarray:
    """Median of a burst of depth frames, ignoring pixels with no reading.

    Zero means "not measured" rather than "at the lens", so averaging it in
    would pull the surface towards the camera wherever the sensor blinked.
    """
    stack = np.stack([np.asarray(frame, dtype=np.float32) for frame in frames])
    stack[stack == 0] = np.nan
    # A pixel the sensor never measured is all-NaN across the burst, which is
    # expected rather than exceptional: it simply stays unmeasured.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        combined = np.nanmedian(stack, axis=0)
    return np.nan_to_num(combined, nan=0.0).astype(np.uint16)


def subject_mask(points: np.ndarray, near_m: float, far_m: float) -> np.ndarray:
    """Which points belong to the thing being scanned.

    Everything nearer than `near_m` or further than `far_m` is background, as
    is anything the sensor could not measure, which the runtime returns as
    -infinity.
    """
    z = points[:, :, 2]
    return np.isfinite(z) & (z >= near_m) & (z <= far_m)


def _quad_grid(points: np.ndarray, mask: np.ndarray, max_step: float):
    """Cells of the pixel grid whose four corners form one continuous surface."""
    corners = (
        mask[:-1, :-1], mask[:-1, 1:], mask[1:, :-1], mask[1:, 1:],
    )
    usable = corners[0] & corners[1] & corners[2] & corners[3]

    z = points[:, :, 2]
    depths = np.stack([z[:-1, :-1], z[:-1, 1:], z[1:, :-1], z[1:, 1:]])
    with np.errstate(invalid="ignore"):
        spread = np.nanmax(depths, axis=0) - np.nanmin(depths, axis=0)
    return usable & (spread <= max_step)


def surface_triangles(points: np.ndarray, mask: np.ndarray, max_step: float):
    """Two triangles per usable cell, wound so the normals face the sensor."""
    usable = _quad_grid(points, mask, max_step)
    if not usable.any():
        raise ScanError(
            "Nothing to build a surface from. Check that the subject is inside "
            "the distance range and that the sensor can see it."
        )

    height, width = mask.shape
    index = np.arange(height * width).reshape(height, width)
    top_left = index[:-1, :-1][usable]
    top_right = index[:-1, 1:][usable]
    bottom_left = index[1:, :-1][usable]
    bottom_right = index[1:, 1:][usable]

    # Camera space has y up and z away from the lens, so this winding puts
    # the front face towards the viewer.
    return np.concatenate([
        np.stack([top_left, top_right, bottom_left], axis=1),
        np.stack([top_right, bottom_right, bottom_left], axis=1),
    ]).astype(np.int64)


def _boundary_edges(triangles: np.ndarray) -> np.ndarray:
    """Directed edges belonging to exactly one triangle: the open rim.

    Deriving the rim from the triangles rather than from the mask means holes
    in the middle of the surface are stitched as well as the outer edge, and
    a single unclosed hole is enough to make a model unprintable.
    """
    edges = np.concatenate([
        triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]],
    ])
    ordered = np.sort(edges, axis=1)
    _unique, inverse, counts = np.unique(
        ordered, axis=0, return_inverse=True, return_counts=True
    )
    return edges[counts[inverse.ravel()] == 1]


# A pinch usually takes one pass to clear; a couple more cover the rare case
# where removing one opens another next to it.
_PINCH_PASSES = 5


def remove_pinch_points(triangles: np.ndarray) -> np.ndarray:
    """Drops triangles meeting at a single vertex between separate regions.

    Where two patches of surface touch only at one corner, that vertex sits
    on two different rim loops at once. The wall raised from each loop then
    uses the same front-to-back edge, so the edge belongs to four faces and
    the model is not manifold, which is exactly what a slicer rejects.

    Real depth data produces these wherever the surface is ragged. Removing
    the offending triangles costs a pixel or two of detail and is invisible
    in a print.
    """
    for _ in range(_PINCH_PASSES):
        if triangles.size == 0:
            break
        rim = _boundary_edges(triangles)
        if rim.size == 0:
            break
        degree = np.bincount(rim.reshape(-1), minlength=int(triangles.max()) + 1)
        pinched = np.flatnonzero(degree > 2)
        if pinched.size == 0:
            return triangles
        triangles = triangles[~np.isin(triangles, pinched).any(axis=1)]

    if triangles.size == 0:
        raise ScanError(
            "The surface fell apart while being cleaned up. It was probably "
            "too fragmented to begin with: try a narrower distance range so "
            "only the subject is captured."
        )
    return triangles


def build_solid(points: np.ndarray, mask: np.ndarray,
                max_step: float = DEFAULT_MAX_STEP_M,
                backing_mm: float = DEFAULT_BACKING_MM):
    """A watertight mesh: the captured surface, a flat back, and a rim.

    Returns vertices in millimetres and triangles indexing them.
    """
    triangles = remove_pinch_points(surface_triangles(points, mask, max_step))

    used = np.unique(triangles)
    remap = np.full(mask.size, -1, dtype=np.int64)
    remap[used] = np.arange(used.size)
    front = triangles.reshape(-1)
    front = remap[front].reshape(triangles.shape)

    vertices = points.reshape(-1, 3)[used] * METRES_TO_MM
    # Mirror x so the model reads the right way round: the sensor's x axis
    # points left from the subject's own point of view.
    vertices = vertices.copy()
    vertices[:, 0] *= -1.0

    rim = _boundary_edges(triangles)
    rim = remap[rim.reshape(-1)].reshape(rim.shape)

    back_z = float(vertices[:, 2].max()) + float(backing_mm)
    back_vertices = vertices.copy()
    back_vertices[:, 2] = back_z
    offset = vertices.shape[0]

    # The back is the same surface flattened and turned inside out.
    back = front[:, ::-1] + offset

    # Each rim edge becomes a wall quad joining front to back. The winding
    # follows the front edge so the wall faces outwards like its neighbours.
    start, end = rim[:, 0], rim[:, 1]
    walls = np.concatenate([
        np.stack([start, end, end + offset], axis=1),
        np.stack([start, end + offset, start + offset], axis=1),
    ])

    all_vertices = np.concatenate([vertices, back_vertices]).astype(np.float32)
    all_triangles = np.concatenate([front, back, walls]).astype(np.int64)
    return all_vertices, all_triangles


def _face_normals(vertices: np.ndarray, triangles: np.ndarray) -> np.ndarray:
    a = vertices[triangles[:, 0]]
    b = vertices[triangles[:, 1]]
    c = vertices[triangles[:, 2]]
    normals = np.cross(b - a, c - a)
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    return np.divide(normals, lengths, out=np.zeros_like(normals), where=lengths > 0)


def write_stl(path, vertices: np.ndarray, triangles: np.ndarray, name: str = "KinectCam scan"):
    """Binary STL. Slicers read the units as millimetres."""
    normals = _face_normals(vertices, triangles)
    corners = vertices[triangles]  # (n, 3, 3)

    record = np.zeros(triangles.shape[0], dtype=np.dtype([
        ("normal", "<f4", 3),
        ("vertices", "<f4", (3, 3)),
        ("attributes", "<u2"),
    ]))
    record["normal"] = normals
    record["vertices"] = corners

    with open(path, "wb") as handle:
        handle.write(name.encode("ascii", "replace")[:80].ljust(80, b"\0"))
        handle.write(struct.pack("<I", triangles.shape[0]))
        handle.write(record.tobytes())


def describe(vertices: np.ndarray, triangles: np.ndarray) -> str:
    size = vertices.max(axis=0) - vertices.min(axis=0)
    return (
        f"{triangles.shape[0]} triangles, {vertices.shape[0]} vertices, "
        f"{size[0]:.0f} x {size[1]:.0f} x {size[2]:.0f} mm"
    )


def is_watertight(triangles: np.ndarray) -> bool:
    """Every edge shared by exactly two triangles, which is what a slicer wants."""
    edges = np.concatenate([
        triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]],
    ])
    _unique, counts = np.unique(np.sort(edges, axis=1), axis=0, return_counts=True)
    return bool((counts == 2).all())
