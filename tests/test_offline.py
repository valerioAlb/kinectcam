"""Tests that do not require the sensor to be connected.

The most important group covers the vtable indices: they are magic numbers,
and a mistake there does not raise an exception but crashes the process, so
they are checked against the method order declared in Kinect.h (SDK 2.0).
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from kinectcam import diagnostics, frames, kinect_native as kn, stability  # noqa: E402
from kinectcam.engine import (  # noqa: E402
    OUTPUT_SIZES,
    CaptureEngine,
    CaptureSettings,
    Mode,
)

# Every COM interface inherits from IUnknown, which occupies the first three slots.
IUNKNOWN = ["QueryInterface", "AddRef", "Release"]

# Method order exactly as declared in Kinect.h.
VTABLES = {
    "IKinectSensor": IUNKNOWN + [
        "SubscribeIsAvailableChanged", "UnsubscribeIsAvailableChanged",
        "GetIsAvailableChangedEventData", "Open", "Close", "get_IsOpen",
        "get_IsAvailable", "get_ColorFrameSource", "get_DepthFrameSource",
        "get_BodyFrameSource", "get_BodyIndexFrameSource",
        "get_InfraredFrameSource", "get_LongExposureInfraredFrameSource",
        "get_AudioSource", "OpenMultiSourceFrameReader", "get_CoordinateMapper",
        "get_UniqueKinectId", "get_KinectCapabilities",
    ],
    "IColorFrameSource": IUNKNOWN + [
        "SubscribeFrameCaptured", "UnsubscribeFrameCaptured",
        "GetFrameCapturedEventData", "get_IsActive", "OpenReader",
        "CreateFrameDescription", "get_FrameDescription", "get_KinectSensor",
    ],
    "IDepthFrameSource": IUNKNOWN + [
        "SubscribeFrameCaptured", "UnsubscribeFrameCaptured",
        "GetFrameCapturedEventData", "get_IsActive", "OpenReader",
        "get_DepthMinReliableDistance", "get_DepthMaxReliableDistance",
        "get_FrameDescription", "get_KinectSensor",
    ],
    "IInfraredFrameSource": IUNKNOWN + [
        "SubscribeFrameCaptured", "UnsubscribeFrameCaptured",
        "GetFrameCapturedEventData", "get_IsActive", "OpenReader",
        "get_FrameDescription", "get_KinectSensor",
    ],
    "IColorFrameReader": IUNKNOWN + [
        "SubscribeFrameArrived", "UnsubscribeFrameArrived",
        "GetFrameArrivedEventData", "AcquireLatestFrame", "get_IsPaused",
        "put_IsPaused", "get_ColorFrameSource",
    ],
    "IColorFrame": IUNKNOWN + [
        "get_RawColorImageFormat", "get_FrameDescription",
        "CopyRawFrameDataToArray", "AccessRawUnderlyingBuffer",
        "CopyConvertedFrameDataToArray", "CreateFrameDescription",
        "get_ColorCameraSettings", "get_RelativeTime", "get_ColorFrameSource",
    ],
    "IDepthFrame": IUNKNOWN + [
        "CopyFrameDataToArray", "AccessUnderlyingBuffer",
        "get_FrameDescription", "get_RelativeTime", "get_DepthFrameSource",
        "get_DepthMinReliableDistance", "get_DepthMaxReliableDistance",
    ],
    "IFrameDescription": IUNKNOWN + [
        "get_Width", "get_Height", "get_HorizontalFieldOfView",
        "get_VerticalFieldOfView", "get_DiagonalFieldOfView",
        "get_LengthInPixels", "get_BytesPerPixel",
    ],
    "IBodyIndexFrameSource": IUNKNOWN + [
        "SubscribeFrameCaptured", "UnsubscribeFrameCaptured",
        "GetFrameCapturedEventData", "get_IsActive", "OpenReader",
        "get_FrameDescription", "get_KinectSensor",
    ],
    "IBodyIndexFrame": IUNKNOWN + [
        "CopyFrameDataToArray", "AccessUnderlyingBuffer",
        "get_FrameDescription", "get_RelativeTime", "get_BodyIndexFrameSource",
    ],
    "ICoordinateMapper": IUNKNOWN + [
        "SubscribeCoordinateMappingChanged", "UnsubscribeCoordinateMappingChanged",
        "GetCoordinateMappingChangedEventData", "MapCameraPointToDepthSpace",
        "MapCameraPointToColorSpace", "MapDepthPointToCameraSpace",
        "MapDepthPointToColorSpace", "MapCameraPointsToDepthSpace",
        "MapCameraPointsToColorSpace", "MapDepthPointsToCameraSpace",
        "MapDepthPointsToColorSpace", "MapDepthFrameToCameraSpace",
        "MapDepthFrameToColorSpace", "MapColorFrameToDepthSpace",
        "MapColorFrameToCameraSpace", "GetDepthFrameToCameraSpaceTable",
        "GetDepthCameraIntrinsics",
    ],
}


class VtableIndexTests(unittest.TestCase):
    def assert_slot(self, interface, index, expected_method):
        self.assertEqual(
            VTABLES[interface][index], expected_method,
            f"{interface} slot {index} should be {expected_method}",
        )

    def test_sensor_slots(self):
        self.assert_slot("IKinectSensor", kn._Sensor.OPEN, "Open")
        self.assert_slot("IKinectSensor", kn._Sensor.CLOSE, "Close")
        self.assert_slot("IKinectSensor", kn._Sensor.IS_OPEN, "get_IsOpen")
        self.assert_slot("IKinectSensor", kn._Sensor.IS_AVAILABLE, "get_IsAvailable")
        self.assert_slot("IKinectSensor", kn._Sensor.COLOR_SOURCE, "get_ColorFrameSource")
        self.assert_slot("IKinectSensor", kn._Sensor.DEPTH_SOURCE, "get_DepthFrameSource")
        self.assert_slot("IKinectSensor", kn._Sensor.INFRARED_SOURCE, "get_InfraredFrameSource")

    def test_open_reader_is_same_slot_in_every_source(self):
        for interface in ("IColorFrameSource", "IDepthFrameSource", "IInfraredFrameSource"):
            self.assert_slot(interface, kn._OPEN_READER, "OpenReader")

    def test_frame_description_slots_differ_per_source(self):
        self.assert_slot("IColorFrameSource", kn._COLOR_SOURCE_DESC, "get_FrameDescription")
        self.assert_slot("IDepthFrameSource", kn._DEPTH_SOURCE_DESC, "get_FrameDescription")
        self.assert_slot("IInfraredFrameSource", kn._INFRARED_SOURCE_DESC, "get_FrameDescription")

    def test_reader_and_frame_slots(self):
        self.assert_slot("IColorFrameReader", kn._READER_ACQUIRE, "AcquireLatestFrame")
        self.assert_slot(
            "IColorFrame", kn._COLOR_FRAME_COPY_CONVERTED, "CopyConvertedFrameDataToArray"
        )
        self.assert_slot("IDepthFrame", kn._PLAIN_FRAME_COPY, "CopyFrameDataToArray")

    def test_description_slots(self):
        self.assert_slot("IFrameDescription", kn._Desc.WIDTH, "get_Width")
        self.assert_slot("IFrameDescription", kn._Desc.HEIGHT, "get_Height")

    def test_relative_time_slots_differ_between_colour_and_the_rest(self):
        # The sensor clock sits at different slots because IColorFrame has
        # more methods before it: reading the wrong slot would give nonsense
        # timestamps and skew the whole dropped-frame diagnosis.
        self.assert_slot("IColorFrame", kn._COLOR_FRAME_RELATIVE_TIME, "get_RelativeTime")
        self.assert_slot("IDepthFrame", kn._PLAIN_FRAME_RELATIVE_TIME, "get_RelativeTime")
        self.assert_slot("IBodyIndexFrame", kn._PLAIN_FRAME_RELATIVE_TIME, "get_RelativeTime")

    def test_body_index_slots(self):
        self.assert_slot("IKinectSensor", kn._Sensor.BODY_INDEX_SOURCE, "get_BodyIndexFrameSource")
        self.assert_slot("IBodyIndexFrameSource", kn._OPEN_READER, "OpenReader")
        self.assert_slot(
            "IBodyIndexFrameSource", kn._BODY_INDEX_SOURCE_DESC, "get_FrameDescription"
        )
        self.assert_slot("IBodyIndexFrame", kn._PLAIN_FRAME_COPY, "CopyFrameDataToArray")

    def test_coordinate_mapper_slots(self):
        self.assert_slot("IKinectSensor", kn._Sensor.COORDINATE_MAPPER, "get_CoordinateMapper")
        self.assert_slot(
            "ICoordinateMapper", kn._MAP_COLOR_FRAME_TO_DEPTH_SPACE,
            "MapColorFrameToDepthSpace",
        )

    def test_e_pending_matches_windows_value(self):
        self.assertEqual(kn.E_PENDING & 0xFFFFFFFF, 0x8000000A)


class FrameConversionTests(unittest.TestCase):
    def test_bgra_to_rgb_reorders_channels(self):
        bgra = np.zeros((2, 2, 4), dtype=np.uint8)
        bgra[..., 0] = 10   # blue
        bgra[..., 1] = 20   # green
        bgra[..., 2] = 30   # red
        bgra[..., 3] = 255  # alpha, to be dropped
        rgb = frames.bgra_to_rgb(bgra)
        self.assertEqual(rgb.shape, (2, 2, 3))
        np.testing.assert_array_equal(rgb[0, 0], [30, 20, 10])

    def test_depth_zero_is_black(self):
        depth = np.zeros((4, 4), dtype=np.uint16)
        rgb = frames.depth_to_rgb(depth)
        self.assertEqual(rgb.shape, (4, 4, 3))
        self.assertEqual(rgb.max(), 0)

    def test_depth_near_and_far_differ(self):
        depth = np.array([[frames.DEPTH_MIN_MM, frames.DEPTH_MAX_MM]], dtype=np.uint16)
        rgb = frames.depth_to_rgb(depth)
        self.assertFalse(np.array_equal(rgb[0, 0], rgb[0, 1]))

    def test_depth_out_of_range_is_clamped_not_wrapped(self):
        # Values below the minimum produced negative indices before clipping.
        depth = np.array([[1, 60000]], dtype=np.uint16)
        rgb = frames.depth_to_rgb(depth)
        np.testing.assert_array_equal(rgb[0, 0], frames._DEPTH_LUT[0])
        np.testing.assert_array_equal(rgb[0, 1], frames._DEPTH_LUT[-1])

    def test_infrared_produces_gray_rgb(self):
        ir = np.array([[0, 65535]], dtype=np.uint16)
        rgb = frames.infrared_to_rgb(ir)
        self.assertEqual(rgb.shape, (1, 2, 3))
        self.assertEqual(rgb[0, 0].tolist(), [0, 0, 0])
        self.assertEqual(rgb[0, 1].tolist(), [255, 255, 255])

    def test_mirror_flips_horizontally(self):
        frame = np.array([[[1, 1, 1], [2, 2, 2]]], dtype=np.uint8)
        np.testing.assert_array_equal(frames.mirror(frame)[0, 0], [2, 2, 2])

    def test_fit_pads_to_exact_size_preserving_aspect(self):
        # 512x424 (roughly 4:3) inside a 16:9 frame: black bars on the sides.
        source = np.full((424, 512, 3), 200, dtype=np.uint8)
        fitted = frames.fit(source, (1280, 720))
        self.assertEqual(fitted.shape, (720, 1280, 3))
        self.assertEqual(fitted[360, 0].tolist(), [0, 0, 0])
        self.assertGreater(int(fitted[360, 640].max()), 0)

    def test_fit_is_identity_at_matching_size(self):
        source = np.zeros((720, 1280, 3), dtype=np.uint8)
        self.assertIs(frames.fit(source, (1280, 720)), source)

    def test_every_output_size_is_produced_exactly(self):
        source = np.full((1080, 1920, 4), 128, dtype=np.uint8)
        rgb = frames.bgra_to_rgb(source)
        for label, size in OUTPUT_SIZES.items():
            with self.subTest(label):
                out = frames.to_contiguous(frames.fit(frames.mirror(rgb), size))
                self.assertEqual(out.shape, (size[1], size[0], 3))
                self.assertEqual(out.dtype, np.uint8)
                # pyvirtualcam rejects non-contiguous buffers.
                self.assertTrue(out.flags["C_CONTIGUOUS"])


class RuntimeAbsentTests(unittest.TestCase):
    """Without the runtime installed, the error must stay readable."""

    def test_runtime_installed_returns_bool(self):
        self.assertIsInstance(kn.runtime_installed(), bool)

    def test_sensor_creation_raises_kinect_error_when_runtime_missing(self):
        if kn.runtime_installed():
            self.skipTest("the runtime is installed: case not applicable")
        with self.assertRaises(kn.KinectError) as caught:
            kn.KinectSensor()
        self.assertIn("Runtime", str(caught.exception))

    def test_engine_reports_error_instead_of_crashing(self):
        if kn.runtime_installed():
            self.skipTest("the runtime is installed: case not applicable")
        errors = []
        stopped = []
        engine = CaptureEngine(
            on_error=errors.append,
            on_stopped=lambda: stopped.append(True),
        )
        engine.start(CaptureSettings(mode=Mode.COLOR, output_size=(1280, 720)))
        engine.stop(timeout=10)
        self.assertTrue(errors, "the engine should have reported an error")
        self.assertTrue(stopped, "the engine should have signalled that it stopped")
        self.assertFalse(engine.running)


class NightVisionTests(unittest.TestCase):
    """Infrared adapts its exposure: that is what makes night vision work."""

    def test_dark_scene_gets_brightened(self):
        # A dark scene has very low values: without adaptation it stays black.
        dark = np.full((32, 32), 400, dtype=np.uint16)
        adaptive = frames.InfraredNormalizer().apply(dark)
        fixed = frames.infrared_to_rgb(dark)
        self.assertGreater(int(adaptive.max()), int(fixed.max()))

    def test_noise_floor_is_not_amplified(self):
        # Total darkness: the minimum reference keeps sensor noise from
        # being turned into a white image.
        noise = np.full((32, 32), 5, dtype=np.uint16)
        result = frames.InfraredNormalizer().apply(noise)
        self.assertLess(int(result.max()), 80)

    def test_a_single_reflection_does_not_black_out_the_scene(self):
        # A specular highlight saturates at 65535: using the absolute
        # maximum as white would crush everything else towards black.
        frame = np.full((32, 32), 3000, dtype=np.uint16)
        frame[0, 0] = 65535
        result = frames.InfraredNormalizer().apply(frame)
        self.assertGreater(int(np.median(result)), 128)

    def test_reference_is_smoothed_across_frames(self):
        # Without smoothing, a change of scene would make the image pulse.
        normalizer = frames.InfraredNormalizer()
        normalizer.apply(np.full((16, 16), 1000, dtype=np.uint16))
        first = normalizer._reference
        normalizer.apply(np.full((16, 16), 20000, dtype=np.uint16))
        second = normalizer._reference
        self.assertLess(second, 20000)
        self.assertGreater(second, first)

    def test_reset_forgets_the_previous_scene(self):
        normalizer = frames.InfraredNormalizer()
        normalizer.apply(np.full((16, 16), 30000, dtype=np.uint16))
        normalizer.reset()
        self.assertIsNone(normalizer._reference)


class BackgroundRemovalTests(unittest.TestCase):
    """Depth matte and background replacement."""

    def _points(self, height, width, depth_x, depth_y):
        """Maps every colour pixel onto the same depth point."""
        points = np.empty((height, width, 2), dtype=np.float32)
        points[..., 0] = depth_x
        points[..., 1] = depth_y
        return points

    def test_subject_inside_the_range_is_kept(self):
        depth = np.full((424, 512), 1500, dtype=np.uint16)
        points = self._points(48, 48, 100, 100)
        matte = frames.depth_matte(points, depth, 500, 2000)
        self.assertTrue(matte.all())

    def test_subject_beyond_the_range_is_dropped(self):
        depth = np.full((424, 512), 3000, dtype=np.uint16)
        points = self._points(48, 48, 100, 100)
        self.assertFalse(frames.depth_matte(points, depth, 500, 2000).any())

    def test_unmeasured_depth_counts_as_background(self):
        # Zero means "not measured", not "extremely close".
        depth = np.zeros((424, 512), dtype=np.uint16)
        points = self._points(48, 48, 100, 100)
        self.assertFalse(frames.depth_matte(points, depth, 500, 2000).any())

    def test_unmappable_pixels_count_as_background(self):
        # The mapper returns -infinity outside the depth camera's field of
        # view; converting that to an integer carelessly would give random indices.
        depth = np.full((424, 512), 1500, dtype=np.uint16)
        points = self._points(48, 48, -np.inf, -np.inf)
        self.assertFalse(frames.depth_matte(points, depth, 500, 2000).any())

    def test_nan_coordinates_do_not_leak_into_the_matte(self):
        depth = np.full((424, 512), 1500, dtype=np.uint16)
        points = self._points(48, 48, np.nan, np.nan)
        self.assertFalse(frames.depth_matte(points, depth, 500, 2000).any())

    def test_coordinates_outside_the_depth_image_are_background(self):
        depth = np.full((424, 512), 1500, dtype=np.uint16)
        for x, y in ((9999, 100), (100, 9999), (-5, 100)):
            with self.subTest(x=x, y=y):
                points = self._points(48, 48, x, y)
                self.assertFalse(frames.depth_matte(points, depth, 500, 2000).any())

    def test_refine_matte_reaches_the_requested_size(self):
        matte = np.ones((180, 320), dtype=bool)
        refined = frames.refine_matte(matte, (1280, 720))
        self.assertEqual(refined.shape, (720, 1280))
        self.assertEqual(refined.dtype, np.bool_)

    def test_refine_matte_removes_isolated_speckles(self):
        matte = np.zeros((60, 60), dtype=bool)
        matte[30, 30] = True  # a single pixel: depth noise
        refined = frames.refine_matte(matte, (60, 60))
        self.assertFalse(refined.any())

    def test_refine_matte_keeps_a_solid_shape(self):
        matte = np.zeros((60, 60), dtype=bool)
        matte[20:40, 20:40] = True
        refined = frames.refine_matte(matte, (60, 60))
        self.assertTrue(refined[30, 30])

    def test_every_background_mode_replaces_only_the_background(self):
        frame = np.full((40, 40, 3), 200, dtype=np.uint8)
        matte = np.zeros((40, 40), dtype=bool)
        matte[10:30, 10:30] = True
        for mode in frames.BACKGROUND_MODES:
            with self.subTest(mode):
                out = frames.apply_background(frame, matte, mode)
                self.assertEqual(out.shape, frame.shape)
                np.testing.assert_array_equal(out[20, 20], [200, 200, 200])

    def test_green_background_uses_the_chroma_key_colour(self):
        frame = np.full((8, 8, 3), 200, dtype=np.uint8)
        matte = np.zeros((8, 8), dtype=bool)
        out = frames.apply_background(frame, matte, frames.BACKGROUND_GREEN)
        np.testing.assert_array_equal(out[0, 0], frames.GREEN_SCREEN_RGB)


class BodyIndexTests(unittest.TestCase):
    """Silhouette matting: the body index instead of distance."""

    def _points(self, height, width, depth_x, depth_y):
        points = np.empty((height, width, 2), dtype=np.float32)
        points[..., 0] = depth_x
        points[..., 1] = depth_y
        return points

    def _empty_body_frame(self):
        return np.full((424, 512), frames.NO_BODY, dtype=np.uint8)

    def test_has_body_is_false_on_an_empty_scene(self):
        self.assertFalse(frames.has_body(self._empty_body_frame()))

    def test_has_body_detects_any_player_index(self):
        # The Kinect tracks up to six people, numbered 0 to 5.
        for index in range(6):
            with self.subTest(index=index):
                body = self._empty_body_frame()
                body[100, 100] = index
                self.assertTrue(frames.has_body(body))

    def test_player_zero_is_a_person_not_background(self):
        # Zero is a valid index: mistaking it for "empty" would make the
        # first tracked person disappear.
        body = self._empty_body_frame()
        body[:] = 0
        points = self._points(48, 48, 100, 100)
        self.assertTrue(frames.body_matte(points, body).all())

    def test_body_matte_keeps_only_the_person(self):
        body = self._empty_body_frame()
        body[200:300, 200:300] = 2
        inside = frames.body_matte(self._points(48, 48, 250, 250), body)
        outside = frames.body_matte(self._points(48, 48, 50, 50), body)
        self.assertTrue(inside.all())
        self.assertFalse(outside.any())

    def test_body_matte_ignores_unmappable_pixels(self):
        body = self._empty_body_frame()
        body[:] = 1
        points = self._points(48, 48, -np.inf, -np.inf)
        self.assertFalse(frames.body_matte(points, body).any())

    def test_body_matte_is_independent_of_distance(self):
        # This is the advantage over distance cutting: a distant person stays,
        # a nearby piece of furniture goes, without touching any threshold.
        body = self._empty_body_frame()
        body[10:20, 10:20] = 3
        self.assertTrue(frames.body_matte(self._points(24, 24, 15, 15), body).all())


class GeometryTests(unittest.TestCase):
    def test_rotate_180_is_its_own_inverse(self):
        frame = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
        np.testing.assert_array_equal(frames.rotate_180(frames.rotate_180(frame)), frame)

    def test_rotate_180_moves_the_corner_across(self):
        frame = np.zeros((4, 4, 3), dtype=np.uint8)
        frame[0, 0] = (255, 0, 0)
        np.testing.assert_array_equal(frames.rotate_180(frame)[3, 3], [255, 0, 0])

    def test_fast_conversion_matches_the_plain_one(self):
        # bgra_to_rgb_resized relies on Pillow's "BGRX" decoder: if the
        # channel order were wrong, red and blue would swap.
        rng = np.random.default_rng(0)
        bgra = rng.integers(0, 256, size=(12, 16, 4), dtype=np.uint8)
        bgra = np.ascontiguousarray(bgra)
        expected = frames.bgra_to_rgb(bgra)
        actual = frames.bgra_to_rgb_resized(bgra, (16, 12))
        np.testing.assert_array_equal(actual, expected)


class StabilityTests(unittest.TestCase):
    """Dropped-frame diagnosis: the part that can reach the wrong verdict."""

    def _report(self, name, period_ms, frames=300, gaps=None, dropped_gaps=0):
        report = stability.StreamReport(name=name, started=True, frames=frames)
        sensor_gaps = gaps if gaps is not None else [period_ms] * (frames - 1)
        sensor_gaps = list(sensor_gaps) + [period_ms * 2] * dropped_gaps
        report.sensor_gaps = sensor_gaps
        report.arrival_gaps = list(sensor_gaps)
        report.duration = sum(sensor_gaps) / 1000.0
        report.dropped = stability._count_dropped(sensor_gaps)
        return report

    def test_no_drops_on_a_regular_stream(self):
        self.assertEqual(stability._count_dropped([33.0] * 100), 0)

    def test_a_doubled_interval_is_one_lost_frame(self):
        self.assertEqual(stability._count_dropped([33.0] * 50 + [66.0]), 1)

    def test_a_long_gap_counts_every_missing_frame(self):
        # 10 periods of silence = 9 frames that never arrived.
        self.assertEqual(stability._count_dropped([33.0] * 50 + [330.0]), 9)

    def test_half_rate_is_not_counted_as_loss(self):
        # In low light the camera runs at 15 fps: 66 ms intervals, all
        # equal. Measuring those against the nominal 33 ms would claim "half
        # the frames are lost", which is false: the reference is the median.
        self.assertEqual(stability._count_dropped([66.0] * 100), 0)

    def test_exposure_halved_is_detected(self):
        self.assertTrue(self._report(stability.COLOR, 66.0).exposure_halved)
        self.assertFalse(self._report(stability.COLOR, 33.0).exposure_halved)

    def test_fps_ignores_the_startup_gap(self):
        # The gap before the stream starts must not halve the average.
        report = stability.StreamReport(name=stability.COLOR, started=True, frames=301)
        report.duration = 10.0
        self.assertAlmostEqual(report.fps, 30.0, places=1)

    def test_light_is_blamed_when_only_colour_slows_down(self):
        reports = {
            stability.COLOR: self._report(stability.COLOR, 66.0),
            stability.DEPTH: self._report(stability.DEPTH, 33.0),
        }
        verdict = " ".join(stability.interpret(reports))
        self.assertIn("LIGHT", verdict)

    def test_link_is_blamed_when_both_streams_lose_frames(self):
        reports = {
            stability.COLOR: self._report(stability.COLOR, 33.0, frames=100, dropped_gaps=40),
            stability.DEPTH: self._report(stability.DEPTH, 33.0, frames=100, dropped_gaps=40),
        }
        verdict = " ".join(stability.interpret(reports))
        self.assertIn("LINK", verdict)

    def test_link_is_not_blamed_when_only_depth_is_measured(self):
        # The comparison only holds if both produced data: without colour
        # there is no basis for blaming the cable.
        reports = {
            stability.COLOR: stability.StreamReport(name=stability.COLOR, started=False),
            stability.DEPTH: self._report(stability.DEPTH, 33.0, frames=100, dropped_gaps=40),
        }
        verdict = " ".join(stability.interpret(reports))
        self.assertNotIn("LINK", verdict)
        self.assertIn("never started", verdict)

    def test_healthy_setup_gets_a_clean_verdict(self):
        reports = {
            stability.COLOR: self._report(stability.COLOR, 33.0),
            stability.DEPTH: self._report(stability.DEPTH, 33.0),
        }
        verdict = " ".join(stability.interpret(reports))
        self.assertIn("no significant loss", verdict.lower())

    def test_background_loss_does_not_raise_an_alarm(self):
        # Around 2% is the background noise measured on a healthy setup.
        reports = {
            stability.COLOR: self._report(stability.COLOR, 33.0, frames=300, dropped_gaps=6),
            stability.DEPTH: self._report(stability.DEPTH, 33.0, frames=300, dropped_gaps=6),
        }
        verdict = " ".join(stability.interpret(reports))
        self.assertNotIn("LINK", verdict)


class FreezeMonitorTests(unittest.TestCase):
    """Telling a cyclic freeze from a random one: that is what gives the cause."""

    def _report(self, starts, length=2.0, frames=900, on_battery=False):
        report = stability.MonitorReport(
            duration=90.0, frames=frames, started=True, on_battery=on_battery
        )
        report.events = [stability.FreezeEvent(at=s, duration=length) for s in starts]
        return report

    def test_regular_freezes_are_recognised_as_a_cycle(self):
        report = self._report([10, 20, 30, 40, 50])
        self.assertTrue(report.is_periodic)
        self.assertAlmostEqual(report.median_interval, 10.0)

    def test_small_jitter_still_counts_as_a_cycle(self):
        report = self._report([10, 20.6, 29.4, 40.3, 50.1])
        self.assertTrue(report.is_periodic)

    def test_scattered_freezes_are_not_a_cycle(self):
        report = self._report([3, 27, 31, 68, 71])
        self.assertFalse(report.is_periodic)

    def test_too_few_freezes_to_judge(self):
        # With only two episodes any pair would look regular.
        self.assertFalse(self._report([10, 20]).is_periodic)

    def test_a_present_device_losing_frames_points_at_the_link(self):
        # Regular freezes with the device still present and its clock running
        # mean the frames were produced and lost on the way here.
        report = self._report([10, 20, 30, 40, 50], length=2.0)
        for event in report.events:
            event.clock_advanced_ms = 2000.0
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertIn("points at the link", verdict)


    def test_battery_is_always_called_out(self):
        verdict = " ".join(
            stability.interpret_monitor(self._report([10, 20, 30, 40], on_battery=True))
        )
        self.assertIn("ON BATTERY", verdict)

    def _offline_report(self, starts=(10, 20, 30, 40)):
        report = self._report(list(starts))
        for event in report.events:
            event.sensor_offline = True
        return report

    def test_a_device_that_drops_off_is_blamed_on_power(self):
        # The runtime losing sight of the sensor means it left the bus. No
        # amount of software can cause that, so the verdict must say hardware.
        report = self._offline_report()
        report.windows_usb_events = 12
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertIn("lost the device entirely", verdict)
        self.assertIn("12 V", verdict)

    def test_the_wrong_power_brick_is_called_out_by_its_rating(self):
        # The Kinect v1 supply looks nearly identical and gives under half the
        # current, which is a mistake worth naming rather than hinting at.
        report = self._offline_report()
        report.windows_usb_events = 12
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertIn("1.08 A", verdict)
        self.assertIn("v1", verdict)

    def test_a_correct_label_does_not_end_the_investigation(self):
        # A genuine, correctly rated supply can still sag after a decade, so
        # the advice has to continue past the label rather than stop there.
        report = self._offline_report()
        report.windows_usb_events = 12
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertIn("correct label does not clear it", verdict)
        self.assertIn("arriving at the sensor", verdict)

    def test_working_on_an_xbox_is_not_treated_as_exoneration(self):
        # The console powers the sensor itself, so it never exercises the
        # adapter or its brick.
        report = self._offline_report()
        report.windows_usb_events = 12
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertIn("never uses", verdict)

    def test_windows_logging_the_dropouts_confirms_an_electrical_fault(self):
        report = self._offline_report()
        report.windows_usb_events = 9
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertIn("re-enumerating", verdict)
        self.assertIn("electrical", verdict)

    def test_depth_surviving_points_at_bandwidth(self):
        # Colour needs several times the bandwidth of depth. Depth running
        # straight through the same instants proves the sensor was alive.
        report = self._offline_report()
        report.windows_usb_events = 0
        report.depth_watched = True
        report.depth_frames = 2600
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertIn("DEPTH KEPT RUNNING", verdict)
        self.assertIn("bandwidth", verdict)

    def test_depth_stalling_too_points_at_the_device(self):
        report = self._offline_report()
        report.windows_usb_events = 0
        report.depth_watched = True
        report.depth_frames = 900
        report.depth_events = [
            stability.FreezeEvent(at=at, duration=6.0) for at in (10, 20, 30, 40)
        ]
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertIn("DEPTH STALLED TOO", verdict)
        self.assertIn("KinectMonitor", verdict)

    def test_a_depth_pass_that_produced_nothing_is_still_reported(self):
        # The gate is whether the second pass ran, not whether it got frames:
        # depth failing to produce anything at all is itself a finding.
        report = self._offline_report()
        report.windows_usb_events = 0
        report.depth_watched = True
        report.depth_frames = 0
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertIn("depth took 0 frames", verdict)

    def test_no_depth_data_means_no_comparison_is_claimed(self):
        report = self._offline_report()
        report.windows_usb_events = 0
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertNotIn("DEPTH KEPT RUNNING", verdict)
        self.assertNotIn("DEPTH STALLED TOO", verdict)

    def test_silence_in_the_log_is_not_overstated(self):
        # Windows does not log a link-level reset, so the absence of events
        # cannot be presented as proof that nothing electrical happened.
        report = self._offline_report()
        report.windows_usb_events = 0
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertIn("does not rule out a link-level reset", verdict)

    def test_windows_silence_moves_the_blame_off_power(self):
        # A device that truly leaves the bus makes Windows notice. If nothing
        # was logged, only the runtime lost it, and telling the user to chase
        # the power supply would send them the wrong way entirely.
        report = self._offline_report()
        report.windows_usb_events = 0
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertIn("NO USB or PnP events", verdict)
        self.assertIn("staying on the bus", verdict)
        self.assertNotIn("POWER IS THE FIRST THING", verdict)

    def test_an_unread_event_log_keeps_the_power_advice(self):
        # Unknown is not the same as zero: without the log, power stays the
        # first thing to rule out.
        report = self._offline_report()
        report.windows_usb_events = -1
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertIn("POWER IS THE FIRST THING", verdict)

    def test_dropping_off_suppresses_the_other_explanations(self):
        # Two contradictory causes on screen at once would be worse than one.
        report = self._offline_report()
        report.unsupported_controller = True
        report.on_battery = True
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertIn("lost the device entirely", verdict)
        self.assertNotIn("LIKELY CAUSE", verdict)
        self.assertNotIn("ON BATTERY", verdict)

    def test_an_occasional_dropout_does_not_override_the_rest(self):
        # One freeze out of four is not the same finding as all of them.
        report = self._report([10, 20, 30, 40], length=2.0)
        report.events[0].sensor_offline = True
        for event in report.events:
            event.clock_advanced_ms = 2000.0
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertNotIn("lost the device entirely", verdict)

    def test_a_present_device_with_a_stopped_clock_points_at_the_sensor(self):
        report = self._report([10, 20, 30, 40], length=2.0)
        for event in report.events:
            event.sensor_offline = False
            event.clock_advanced_ms = 0.0
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertIn("stopped producing frames", verdict)

    def test_a_clock_that_ran_through_points_at_the_link(self):
        report = self._report([10, 20, 30, 40], length=2.0)
        for event in report.events:
            event.sensor_offline = False
            event.clock_advanced_ms = 2000.0
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertIn("never reached this machine", verdict)

    def test_clock_comparison_is_against_the_freeze_length(self):
        # A clock that moved a few milliseconds across a six second gap did
        # not keep running, even though it did move.
        stalled = stability.FreezeEvent(at=0, duration=6.0, clock_advanced_ms=30.0)
        ran = stability.FreezeEvent(at=0, duration=6.0, clock_advanced_ms=5900.0)
        self.assertFalse(stalled.clock_kept_running())
        self.assertTrue(ran.clock_kept_running())

    def test_counts_are_reported_per_event(self):
        report = self._report([10, 20, 30, 40], length=2.0)
        report.events[0].sensor_offline = True
        report.events[1].clock_advanced_ms = 2000.0
        self.assertEqual(report.dropped_off_count, 1)
        self.assertEqual(report.clock_kept_running_count, 1)

    def test_an_unsupported_controller_is_named_as_the_likely_cause(self):
        # Regular freezes on a desktop plugged into mains are far more often
        # the host controller than power management, and changing port does
        # not help when every port is on the same chipset.
        report = self._report([10, 20, 30, 40, 50], length=2.0)
        for event in report.events:
            event.clock_advanced_ms = 2000.0
        report.unsupported_controller = True
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertIn("LIKELY CAUSE", verdict)
        self.assertIn("Renesas", verdict)

    def test_a_supported_controller_is_not_blamed(self):
        verdict = " ".join(stability.interpret_monitor(self._report([10, 20, 30, 40, 50])))
        self.assertNotIn("LIKELY CAUSE", verdict)

    def test_irregular_freezes_with_the_device_present_blame_the_contact(self):
        report = self._report([3, 27, 31, 68, 71], length=2.0)
        for event in report.events:
            event.clock_advanced_ms = 2000.0
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertIn("IRREGULAR", verdict)

    def test_no_freezes_suggests_repeating_under_load(self):
        verdict = " ".join(stability.interpret_monitor(self._report([])))
        self.assertIn("No freezes", verdict)

    def test_a_stream_that_never_started_is_not_diagnosed(self):
        report = stability.MonitorReport(started=False)
        verdict = " ".join(stability.interpret_monitor(report))
        self.assertIn("never started", verdict)
        self.assertNotIn("VERDICT", verdict)


class PowerCheckTests(unittest.TestCase):
    def test_reports_a_boolean_or_none(self):
        self.assertIn(diagnostics.on_ac_power(), (True, False, None))

    def test_both_power_profiles_are_read(self):
        # A desktop has no battery profile, so reading only the battery setting
        # reports "unreadable" and tells the user nothing about the one that
        # actually applies to them.
        ac, dc = diagnostics.usb_selective_suspend()
        for value in (ac, dc):
            self.assertIn(value, (True, False, None))

    def test_check_runs_and_explains_itself(self):
        check = diagnostics.check_power()
        self.assertEqual(check.name, "Power source")
        self.assertTrue(check.detail)
        if not check.ok:
            self.assertTrue(check.hint)


class UsbControllerTests(unittest.TestCase):
    """The Kinect v2 only works properly on Intel or Renesas controllers."""

    def test_vendor_table_marks_only_intel_and_renesas_as_supported(self):
        supported = {
            vendor for vendor, ok in diagnostics.USB_CONTROLLER_VENDORS.values() if ok
        }
        self.assertEqual(supported, {"Intel", "Renesas", "NEC/Renesas"})

    def test_the_known_problem_chipsets_are_listed(self):
        # These are the ones users actually hit, and the reason the check
        # exists at all: on them the sensor streams in fits and starts.
        names = {vendor for vendor, _ in diagnostics.USB_CONTROLLER_VENDORS.values()}
        for chipset in ("AMD", "ASMedia", "VIA", "Fresco Logic"):
            self.assertIn(chipset, names)

    # Vendor ids as they appear in a PCI instance id.
    INTEL = "8086"
    RENESAS = "1912"
    AMD = "1022"
    ASMEDIA = "1b21"

    def test_an_intel_controller_passes(self):
        check = diagnostics.classify_usb_controllers([(self.INTEL, "Intel xHCI")])
        self.assertTrue(check.ok)
        self.assertIn("Intel", check.detail)

    def test_an_amd_only_machine_fails_with_the_fix(self):
        # The case that prompted this check: an all-AMD desktop streaming in
        # seven-second bursts. The hint has to name the remedy, not just the
        # problem, because changing port cannot help here.
        check = diagnostics.classify_usb_controllers([
            (self.AMD, "AMD USB 3.10 eXtensible Host Controller"),
            (self.AMD, "AMD USB 3.20 eXtensible Host Controller"),
        ])
        self.assertFalse(check.ok)
        self.assertIn("AMD", check.detail)
        self.assertIn("unsupported", check.detail)
        self.assertIn("Renesas", check.hint)

    def test_a_supported_controller_alongside_an_unsupported_one_passes(self):
        # Plenty of machines have both; one usable controller is enough.
        check = diagnostics.classify_usb_controllers([
            (self.AMD, "AMD xHCI"), (self.RENESAS, "Renesas xHCI"),
        ])
        self.assertTrue(check.ok)

    def test_other_problem_chipsets_are_rejected_too(self):
        for vendor_id, name in ((self.ASMEDIA, "ASMedia"), ("1106", "VIA")):
            with self.subTest(name):
                self.assertFalse(
                    diagnostics.classify_usb_controllers([(vendor_id, name)]).ok
                )

    def test_an_unknown_vendor_is_treated_as_unsupported(self):
        check = diagnostics.classify_usb_controllers([("ffff", "Mystery controller")])
        self.assertFalse(check.ok)
        self.assertIn("Renesas", check.hint)

    def test_no_controllers_reports_the_usb3_requirement(self):
        # A virtual machine enumerates no PCI USB controllers at all. That is
        # a different failure from an unsupported chipset, and suggesting a
        # Renesas card there would be beside the point.
        check = diagnostics.classify_usb_controllers([])
        self.assertFalse(check.ok)
        self.assertIn("USB 3.0", check.hint)

    def test_the_live_check_returns_a_usable_verdict(self):
        check = diagnostics.check_usb3()
        self.assertEqual(check.name, "USB 3.0 controller")
        self.assertTrue(check.detail)
        if not check.ok:
            self.assertTrue(check.hint)

    def test_support_query_agrees_with_the_check(self):
        supported = diagnostics.has_supported_usb_controller()
        self.assertIn(supported, (True, False, None))
        if supported is not None:
            self.assertEqual(diagnostics.check_usb3().ok, supported)


class RuntimePresentTests(unittest.TestCase):
    """Checks the vtable indices against the real runtime, with no sensor.

    The runtime answers even with the sensor unplugged: the sources and their
    FrameDescriptions exist regardless. A wrong index here would produce absurd
    dimensions or a crash, so these tests cover the whole native path except
    the pixel copy, which needs real frames.
    """

    @classmethod
    def setUpClass(cls):
        if not kn.runtime_installed():
            raise unittest.SkipTest("Kinect Runtime not installed")

    def setUp(self):
        self.sensor = kn.KinectSensor()
        self.sensor.open()
        self.addCleanup(self.sensor.close)

    def test_is_available_returns_bool(self):
        self.assertIsInstance(self.sensor.is_available, bool)

    def test_stream_dimensions_match_the_hardware_spec(self):
        cases = (
            ("colour", self.sensor.color_stream, 1920, 1080),
            ("depth", self.sensor.depth_stream, 512, 424),
            ("infrared", self.sensor.infrared_stream, 512, 424),
        )
        for name, open_stream, width, height in cases:
            with self.subTest(name):
                stream = open_stream()
                self.assertEqual((stream.width, stream.height), (width, height))

    def test_buffers_are_sized_from_the_description(self):
        color = self.sensor.color_stream()
        self.assertEqual(color.buffer.shape, (1080, 1920, 4))
        depth = self.sensor.depth_stream()
        self.assertEqual(depth.buffer.shape, (424, 512))

    def test_body_index_stream_is_one_byte_per_pixel(self):
        body = self.sensor.body_index_stream()
        self.assertEqual((body.width, body.height), (512, 424))
        self.assertEqual(body.buffer.shape, (424, 512))
        self.assertEqual(body.buffer.dtype, np.uint8)

    def test_coordinate_mapper_fills_the_colour_grid(self):
        color = self.sensor.color_stream()
        depth = self.sensor.depth_stream()
        mapper = self.sensor.coordinate_mapper(color.width, color.height)
        points = mapper.map_color_to_depth(depth)
        self.assertEqual(points.shape, (color.height, color.width, 2))
        self.assertEqual(points.dtype, np.float32)
        # Without valid depth the runtime marks everything as unmappable,
        # never with random coordinates: any finite value must land inside
        # the depth image.
        finite = points[np.isfinite(points[..., 0])]
        if finite.size:
            self.assertGreaterEqual(float(points[..., 0][np.isfinite(points[..., 0])].min()), -1.0)

    def test_read_returns_none_without_a_connected_sensor(self):
        # AcquireLatestFrame returns E_PENDING, which is not an error.
        if self.sensor.is_available:
            self.skipTest("sensor connected: read() may return a frame")
        self.assertIsNone(self.sensor.color_stream().read())
        self.assertIsNone(self.sensor.depth_stream().read())


class DiagnosticsTests(unittest.TestCase):
    def test_checks_run_and_report(self):
        checks = diagnostics.run_checks(include_sensor=False)
        self.assertTrue(checks)
        for check in checks:
            self.assertIsInstance(check.ok, bool)
            self.assertTrue(check.name)
        report = diagnostics.format_report(checks)
        self.assertIn("KinectCam diagnostics", report)

    def test_failed_checks_carry_a_hint(self):
        for check in diagnostics.run_checks(include_sensor=False):
            if not check.ok and check.name != "Operating system":
                self.assertTrue(check.hint, f"{check.name} has no hint")


if __name__ == "__main__":
    unittest.main(verbosity=2)
