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


class PreviewTests(unittest.TestCase):
    """The live 3D view, which is also how the distance range gets tuned."""

    def _scene(self):
        points = dome(40, 48)
        return points, scanning.subject_mask(points, 0.7, 0.9)

    def test_the_frame_comes_out_at_the_requested_size(self):
        points, mask = self._scene()
        image = scanning.render_preview(points, mask, (320, 180))
        self.assertEqual(image.shape, (180, 320, 3))
        self.assertEqual(image.dtype, np.uint8)

    def test_an_empty_range_draws_only_the_backdrop(self):
        points, _mask = self._scene()
        nothing = np.zeros(points.shape[:2], dtype=bool)
        image = scanning.render_preview(points, nothing, (64, 48))
        self.assertTrue((image.reshape(-1, 3) == scanning._BACKDROP_RGB).all())

    def test_turning_the_view_changes_what_is_drawn(self):
        points, mask = self._scene()
        straight = scanning.render_preview(points, mask, (160, 120), yaw_deg=0)
        turned = scanning.render_preview(points, mask, (160, 120), yaw_deg=40)
        self.assertFalse(np.array_equal(straight, turned))

    def test_narrowing_the_range_removes_surface(self):
        # This is what makes the preview useful for setting the range: pull
        # the limit in and the far half of the scene should disappear.
        points = dome(40, 48)
        wide = scanning.subject_mask(points, 0.5, 1.2)
        narrow = scanning.subject_mask(points, 0.5, 0.77)
        self.assertLess(narrow.sum(), wide.sum())
        drawn = lambda m: (scanning.render_preview(points, m, (160, 120))
                           .reshape(-1, 3) != scanning._BACKDROP_RGB).any(axis=1).sum()
        self.assertLess(drawn(narrow), drawn(wide))

    def test_one_stray_point_does_not_shrink_the_subject(self):
        # The scale used to come from the maximum, so a single noisy point at
        # the edge of the range set it and the subject rendered as a speck.
        points, mask = self._scene()
        clean = scanning.render_preview(points, mask, (200, 150))
        points[0, 0] = (5.0, 5.0, 0.8)
        mask[0, 0] = True
        with_outlier = scanning.render_preview(points, mask, (200, 150))

        def coverage(image):
            return (image.reshape(-1, 3) != scanning._BACKDROP_RGB).any(axis=1).mean()

        self.assertGreater(coverage(with_outlier), coverage(clean) * 0.5)

    def test_the_splat_grows_when_points_are_sparse(self):
        # Few points over a large raster need spreading, or the surface reads
        # as scattered dust rather than a solid.
        self.assertGreaterEqual(scanning._splat_radius(500, 400, 300),
                                scanning._splat_radius(200000, 400, 300))
        self.assertGreaterEqual(scanning._splat_radius(0, 400, 300), 1)

    def test_normals_of_a_flat_surface_all_agree(self):
        points = dome(10, 10, bump=0.0)
        normals = scanning.surface_normals(points)
        reference = normals[5, 5]
        self.assertAlmostEqual(float(np.linalg.norm(reference)), 1.0, places=4)
        self.assertGreater(abs(float(reference[2])), 0.99)

    def test_a_sloped_surface_shades_differently_from_a_flat_one(self):
        flat = dome(30, 30, bump=0.0)
        bumped = dome(30, 30, bump=0.04)
        mask = np.ones(flat.shape[:2], dtype=bool)
        flat_image = scanning.render_preview(flat, mask, (160, 120))
        bumped_image = scanning.render_preview(bumped, mask, (160, 120))
        self.assertFalse(np.array_equal(flat_image, bumped_image))


def cylinder_views(n_views=8, radius=0.1, axis=(0.05, 0.9), height=0.3,
                   rows=40, cols=60, lobe=0.0):
    """Views of a known solid of revolution, as a turntable would collect them.

    Each view keeps only the half facing the sensor, which is what the real
    thing sees, so the merge has to combine them to recover the whole object.

    `lobe` swells one side, making the object asymmetric. A plain cylinder
    looks identical from every angle, so it cannot tell a correct set of
    angles from a wrong one: anything testing the angles needs a lobe.
    """
    views = []
    for k in range(n_views):
        angle = 2 * np.pi * k / n_views
        theta = np.linspace(-np.pi, np.pi, cols, endpoint=False)
        y = np.linspace(-height / 2, height / 2, rows)
        grid_theta, grid_y = np.meshgrid(theta, y)

        # Surface in the object's own frame, then turned to where the sensor
        # would have seen it for this view.
        local_radius = radius * (1.0 + lobe * np.cos(grid_theta))
        x = local_radius * np.sin(grid_theta)
        z = local_radius * np.cos(grid_theta)
        turned_x = x * np.cos(angle) - z * np.sin(angle) + axis[0]
        turned_z = x * np.sin(angle) + z * np.cos(angle) + axis[1]

        points = np.stack([turned_x, grid_y, turned_z], axis=2).astype(np.float32)
        # Only the near half is visible from the sensor.
        mask = turned_z < axis[1]
        views.append((points, mask, angle))
    return views


class TurntableTests(unittest.TestCase):
    """Merging views of a rotating subject into one closed model."""

    def test_the_axis_is_found_from_the_views_themselves(self):
        # Getting this wrong smears the merged surface, and asking the user
        # where their own axis of rotation is would be a strange question.
        views = cylinder_views()
        heights = np.concatenate([p[m][:, 1] for p, m, _ in views])
        bounds = (float(heights.min()), float(heights.max()))
        axis, score = scanning.estimate_axis(views, bounds)
        np.testing.assert_allclose(axis, (0.05, 0.9), atol=0.03)
        self.assertLess(score, 0.02)

    def test_a_cylinder_comes_back_as_a_cylinder(self):
        views = cylinder_views(radius=0.1)
        vertices, triangles = scanning.build_turntable_solid(views)
        # Radius about the axis, in millimetres, ignoring the two cap centres.
        centre = vertices[:-2].mean(axis=0)
        radii = np.hypot(vertices[:-2, 0] - centre[0], vertices[:-2, 2] - centre[2])
        self.assertAlmostEqual(float(np.median(radii)), 100.0, delta=8.0)

    def test_the_merged_model_is_watertight(self):
        views = cylinder_views()
        _vertices, triangles = scanning.build_turntable_solid(views)
        self.assertTrue(scanning.is_watertight(triangles))

    def test_the_model_closes_at_both_ends(self):
        # A lathe surface without caps is a tube, and a tube is not printable.
        views = cylinder_views()
        vertices, triangles = scanning.build_turntable_solid(views)
        heights = vertices[:, 1]
        self.assertAlmostEqual(float(heights.max() - heights.min()), 300.0, delta=15.0)

    def test_each_view_alone_covers_only_half_the_object(self):
        # Confirms the fixture is honest: if one view already saw everything,
        # the merge would not be being tested at all.
        views = cylinder_views(n_views=8)
        single = scanning.build_turntable_solid(views[:2])
        self.assertTrue(scanning.is_watertight(single[1]))

    def test_a_bad_turn_is_called_out_rather_than_silently_smeared(self):
        # Views that contradict each other still merge, into a smeared shape.
        # Saying so turns a mystifying result into one worth repeating.
        views = cylinder_views(n_views=8, lobe=0.5)
        scrambled = [(p, m, a * 2.7) for p, m, a in views]
        notes = []
        scanning.build_turntable_solid(scrambled, progress=notes.append)
        self.assertTrue(any("disagree by" in note for note in notes))

    def test_a_clean_turn_raises_no_warning(self):
        notes = []
        scanning.build_turntable_solid(cylinder_views(lobe=0.5), progress=notes.append)
        self.assertFalse(any("disagree by" in note for note in notes))

    def test_an_asymmetric_object_keeps_its_shape(self):
        # A cylinder would come back right even if the angles were nonsense,
        # so the merge is checked on something that has a front and a back.
        views = cylinder_views(n_views=12, radius=0.1, lobe=0.4)
        vertices, _triangles = scanning.build_turntable_solid(views)
        body = vertices[:-2]
        centre = body.mean(axis=0)
        radii = np.hypot(body[:, 0] - centre[0], body[:, 2] - centre[2])
        # The swollen side should be markedly further out than the flat one.
        self.assertGreater(float(radii.max()) / float(radii.min()), 1.4)

    def test_too_few_usable_views_is_refused_clearly(self):
        views = cylinder_views(n_views=4)
        blanked = [(p, np.zeros_like(m), a) for p, m, a in views[1:]]
        with self.assertRaises(scanning.ScanError) as caught:
            scanning.build_turntable_solid([views[0]] + blanked)
        self.assertIn("two views", str(caught.exception))

    def test_gaps_around_the_ring_are_filled(self):
        grid = np.zeros((4, 8), dtype=np.float32)
        counts = np.zeros((4, 8), dtype=np.int32)
        grid[:, 0] = 0.1
        grid[:, 4] = 0.2
        counts[:, 0] = counts[:, 4] = 1
        filled = scanning._fill_gaps(grid, counts)
        self.assertTrue((filled > 0).all())
        # Between the two known angles the radius should lie between them.
        self.assertGreater(filled[0, 2], 0.1)
        self.assertLess(filled[0, 2], 0.2)

    def test_a_row_nothing_was_seen_in_borrows_from_its_neighbour(self):
        grid = np.zeros((3, 6), dtype=np.float32)
        counts = np.zeros((3, 6), dtype=np.int32)
        grid[0, :] = 0.15
        counts[0, :] = 1
        filled = scanning._fill_gaps(grid, counts)
        self.assertTrue((filled[2] > 0).all())


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
