"""Tests for the 3D scanning geometry.

Watertightness carries most of the weight here: a slicer refuses a mesh with
a single open edge, and an open edge is invisible until someone tries to
print. The checks build surfaces with the awkward shapes that occur in real
captures, holes and depth steps included, and insist the result still closes.
"""

from __future__ import annotations

import struct
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from kinectcam import scanning  # noqa: E402


def dome(height=20, width=24, distance=0.8, bump=0.05):
    """A smooth bulge at `distance` metres, the shape of a face in miniature."""
    rows = np.linspace(-0.1, 0.1, height, dtype=np.float32)
    cols = np.linspace(-0.12, 0.12, width, dtype=np.float32)
    x, y = np.meshgrid(cols, rows)
    radius = np.sqrt(x ** 2 + y ** 2)
    z = distance - bump * np.exp(-(radius / 0.06) ** 2)
    return np.stack([x, y, z.astype(np.float32)], axis=2)


class SurfaceTests(unittest.TestCase):
    def test_a_flat_surface_produces_two_triangles_per_cell(self):
        points = dome(6, 8, bump=0.0)
        mask = np.ones(points.shape[:2], dtype=bool)
        triangles = scanning.surface_triangles(points, mask, scanning.DEFAULT_MAX_STEP_M)
        self.assertEqual(triangles.shape, ((6 - 1) * (8 - 1) * 2, 3))

    def test_a_steep_slope_loses_the_cells_that_span_it(self):
        # Not a defect: a gradient steeper than the step limit across one
        # cell is indistinguishable from two surfaces at different depths.
        points = dome(6, 8, bump=0.05)
        mask = np.ones(points.shape[:2], dtype=bool)
        triangles = scanning.surface_triangles(points, mask, scanning.DEFAULT_MAX_STEP_M)
        self.assertLess(triangles.shape[0], (6 - 1) * (8 - 1) * 2)
        # A looser limit keeps them, which is the knob to reach for when a
        # capture comes out with holes on steeply angled surfaces.
        relaxed = scanning.surface_triangles(points, mask, 0.2)
        self.assertEqual(relaxed.shape[0], (6 - 1) * (8 - 1) * 2)

    def test_background_is_excluded(self):
        points = dome()
        mask = scanning.subject_mask(points, 0.7, 0.9)
        self.assertTrue(mask.all())
        self.assertFalse(scanning.subject_mask(points, 0.1, 0.2).any())

    def test_unmeasured_pixels_are_not_subject(self):
        points = dome()
        points[5, 5, 2] = -np.inf
        self.assertFalse(scanning.subject_mask(points, 0.7, 0.9)[5, 5])

    def test_a_depth_step_is_not_bridged(self):
        # A face in front of a wall: joining them would stretch triangles
        # across the gap and print as webbing.
        points = dome(10, 10, distance=0.8)
        points[:, 5:, 2] = 1.4
        mask = np.ones(points.shape[:2], dtype=bool)
        triangles = scanning.surface_triangles(points, mask, 0.02)
        z = points[:, :, 2].reshape(-1)
        spans = z[triangles].max(axis=1) - z[triangles].min(axis=1)
        self.assertTrue((spans <= 0.02 + 1e-6).all())

    def test_an_empty_capture_is_refused_clearly(self):
        points = dome()
        empty = np.zeros(points.shape[:2], dtype=bool)
        with self.assertRaises(scanning.ScanError) as caught:
            scanning.surface_triangles(points, empty, scanning.DEFAULT_MAX_STEP_M)
        self.assertIn("distance range", str(caught.exception))


class SolidTests(unittest.TestCase):
    def _solid(self, points=None, mask=None):
        points = dome() if points is None else points
        mask = np.ones(points.shape[:2], dtype=bool) if mask is None else mask
        return scanning.build_solid(points, mask)

    def test_the_result_is_watertight(self):
        vertices, triangles = self._solid()
        self.assertTrue(scanning.is_watertight(triangles))

    def test_a_hole_in_the_middle_is_stitched_too(self):
        # Dropouts happen in the middle of real captures, on hair and on
        # anything shiny. An unstitched hole is just as unprintable as an
        # open outer edge.
        points = dome()
        mask = np.ones(points.shape[:2], dtype=bool)
        mask[8:12, 10:14] = False
        vertices, triangles = self._solid(points, mask)
        self.assertTrue(scanning.is_watertight(triangles))

    def test_two_separate_islands_both_close(self):
        points = dome()
        mask = np.zeros(points.shape[:2], dtype=bool)
        mask[2:8, 2:8] = True
        mask[12:18, 14:20] = True
        vertices, triangles = self._solid(points, mask)
        self.assertTrue(scanning.is_watertight(triangles))

    def test_two_patches_touching_at_a_corner_still_close(self):
        # The case real captures produce and synthetic ones do not: two
        # regions meeting at a single vertex put that vertex on two rim loops
        # at once, and the wall raised from each reuses the same front-to-back
        # edge. Four faces on one edge is precisely what a slicer rejects.
        points = dome(12, 12, bump=0.0)
        mask = np.zeros(points.shape[:2], dtype=bool)
        mask[1:5, 1:5] = True
        mask[5:9, 5:9] = True
        vertices, triangles = scanning.build_solid(points, mask)
        self.assertTrue(scanning.is_watertight(triangles))

    def test_pinch_removal_leaves_the_bulk_of_the_surface(self):
        points = dome(12, 12, bump=0.0)
        mask = np.zeros(points.shape[:2], dtype=bool)
        mask[1:5, 1:5] = True
        mask[5:9, 5:9] = True
        before = scanning.surface_triangles(points, mask, scanning.DEFAULT_MAX_STEP_M)
        after = scanning.remove_pinch_points(before)
        self.assertGreater(after.shape[0], before.shape[0] * 0.5)

    def test_a_clean_surface_is_left_alone(self):
        points = dome(10, 10, bump=0.0)
        mask = np.ones(points.shape[:2], dtype=bool)
        before = scanning.surface_triangles(points, mask, scanning.DEFAULT_MAX_STEP_M)
        after = scanning.remove_pinch_points(before)
        self.assertEqual(after.shape[0], before.shape[0])

    def test_the_model_has_a_flat_back(self):
        vertices, _triangles = self._solid()
        back = vertices[:, 2].max()
        on_back = np.isclose(vertices[:, 2], back)
        self.assertGreater(on_back.sum(), vertices.shape[0] // 3)

    def test_the_backing_adds_the_requested_thickness(self):
        points = dome()
        mask = np.ones(points.shape[:2], dtype=bool)
        thin, _ = scanning.build_solid(points, mask, backing_mm=2.0)
        thick, _ = scanning.build_solid(points, mask, backing_mm=12.0)
        self.assertAlmostEqual(
            (thick[:, 2].max() - thin[:, 2].max()), 10.0, places=3
        )

    def test_dimensions_come_out_in_millimetres(self):
        # The capture spans 0.24 m across, so the model must be 240 mm, not
        # 0.24: an STL carries no units and slicers assume millimetres.
        vertices, _triangles = self._solid()
        width = vertices[:, 0].max() - vertices[:, 0].min()
        self.assertAlmostEqual(width, 240.0, delta=1.0)

    def test_the_model_is_not_mirrored(self):
        # The sensor's x axis runs opposite to the subject's own left and
        # right, and a face printed inside out is a subtle thing to notice.
        points = dome(8, 8)
        mask = np.ones(points.shape[:2], dtype=bool)
        points[:, :, 0] = np.linspace(-0.1, 0.1, 8, dtype=np.float32)[None, :]
        vertices, _ = scanning.build_solid(points, mask)
        first_column_x = vertices[0, 0]
        self.assertGreater(first_column_x, 0.0)

    def test_front_faces_point_towards_the_sensor(self):
        vertices, triangles = self._solid()
        normals = scanning._face_normals(vertices, triangles)
        front_count = (normals[:, 2] < -0.5).sum()
        back_count = (normals[:, 2] > 0.5).sum()
        self.assertGreater(front_count, 0)
        self.assertGreater(back_count, 0)


class FrameCombiningTests(unittest.TestCase):
    def test_median_rejects_a_single_wild_reading(self):
        frames = [np.full((4, 4), 800, dtype=np.uint16) for _ in range(5)]
        frames[2][1, 1] = 4000
        combined = scanning.combine_frames(frames)
        self.assertEqual(int(combined[1, 1]), 800)

    def test_unmeasured_pixels_do_not_drag_the_surface_forward(self):
        # Zero means "no reading", and averaging it in would pull the point
        # towards the lens.
        frames = [np.full((4, 4), 800, dtype=np.uint16) for _ in range(5)]
        for frame in frames[:2]:
            frame[0, 0] = 0
        combined = scanning.combine_frames(frames)
        self.assertEqual(int(combined[0, 0]), 800)

    def test_a_pixel_never_measured_stays_zero(self):
        frames = [np.zeros((2, 2), dtype=np.uint16) for _ in range(3)]
        self.assertEqual(int(scanning.combine_frames(frames)[0, 0]), 0)


class StlTests(unittest.TestCase):
    def _write(self, vertices, triangles):
        path = Path(tempfile.gettempdir()) / "kinectcam-test.stl"
        scanning.write_stl(path, vertices, triangles)
        return path

    def test_binary_stl_header_and_count(self):
        points = dome(8, 8)
        mask = np.ones(points.shape[:2], dtype=bool)
        vertices, triangles = scanning.build_solid(points, mask)
        path = self._write(vertices, triangles)

        data = path.read_bytes()
        count = struct.unpack("<I", data[80:84])[0]
        self.assertEqual(count, triangles.shape[0])
        # 80 byte header, 4 byte count, then 50 bytes per triangle.
        self.assertEqual(len(data), 84 + 50 * count)
        path.unlink()

    def test_vertices_round_trip(self):
        points = dome(5, 5)
        mask = np.ones(points.shape[:2], dtype=bool)
        vertices, triangles = scanning.build_solid(points, mask)
        path = self._write(vertices, triangles)

        data = path.read_bytes()
        record = np.frombuffer(data[84:], dtype=np.dtype([
            ("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attributes", "<u2"),
        ]))
        np.testing.assert_allclose(
            record["vertices"][0], vertices[triangles[0]], rtol=1e-5
        )
        path.unlink()

    def test_normals_are_unit_length(self):
        points = dome(6, 6)
        mask = np.ones(points.shape[:2], dtype=bool)
        vertices, triangles = scanning.build_solid(points, mask)
        normals = scanning._face_normals(vertices, triangles)
        lengths = np.linalg.norm(normals, axis=1)
        np.testing.assert_allclose(lengths, 1.0, atol=1e-5)

    def test_describe_reports_millimetres(self):
        points = dome()
        mask = np.ones(points.shape[:2], dtype=bool)
        vertices, triangles = scanning.build_solid(points, mask)
        summary = scanning.describe(vertices, triangles)
        self.assertIn("mm", summary)
        self.assertIn("triangles", summary)


if __name__ == "__main__":
    unittest.main(verbosity=2)
