"""Which buttons the window offers, against a stand-in engine.

The rest of the suite leaves the window alone, because a widget layout is not
worth pinning down. What is worth pinning down is when a button can be
pressed: the 360 scan and the capture-this-view shot are only offered while
the preview is live, and an answer that is a moment out of date takes the
button away for the whole session.

Tk needs a desktop to open a window on, so these skip where there is none.
"""

from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from kinectcam.engine import MODE_LABELS, Mode  # noqa: E402

try:
    import tkinter as tk

    _root = tk.Tk()
    _root.destroy()
    HAS_TK = True
except Exception:  # pragma: no cover - depends on the host, not the code
    HAS_TK = False


class FakeEngine:
    """Enough of CaptureEngine for the window to be built and started.

    `running` lags deliberately: the real capture thread is still alive for a
    moment after it reports that it has stopped, and it does not exist yet at
    the instant start() is called from the window.
    """

    def __init__(self, on_status=None, on_error=None, on_stopped=None):
        self.on_status = on_status
        self.on_error = on_error
        self.on_stopped = on_stopped
        self.running = False
        self.started_with = None
        self.live_settings = {}

    def start(self, settings):
        self.started_with = settings
        # The thread is spawned, but the window is not told: it has to know
        # from having started the engine itself.
        self.running = True

    def stop(self, timeout=4.0):
        self.running = False

    def update_live_settings(self, **kwargs):
        self.live_settings.update(kwargs)

    def take_preview(self):
        return None

    def request_scan(self, *args, **kwargs):
        return None

    def request_turntable_scan(self, *args, **kwargs):
        return None

    fps = 0.0
    output_fps = 0.0
    device_name = "fake"


@unittest.skipUnless(HAS_TK, "no display for Tk")
class ScanButtonsTest(unittest.TestCase):
    def setUp(self):
        from kinectcam import app as app_module

        self.app_module = app_module
        self._real_engine = app_module.CaptureEngine
        app_module.CaptureEngine = FakeEngine
        # Diagnostics reach for WMI and the registry, which is a different
        # subject and slow besides.
        self._real_diagnostics = app_module.KinectCamApp._run_diagnostics
        app_module.KinectCamApp._run_diagnostics = lambda self, include_sensor: None

        self.root = tk.Tk()
        self.root.withdraw()
        self.app = app_module.KinectCamApp(self.root)

    def tearDown(self):
        self.root.destroy()
        self.app_module.CaptureEngine = self._real_engine
        self.app_module.KinectCamApp._run_diagnostics = self._real_diagnostics

    def _select_preview(self):
        self.app.mode_var.set(MODE_LABELS[Mode.SCAN_PREVIEW])
        self.app._on_mode_changed()

    def test_turntable_needs_the_preview_running(self):
        self._select_preview()
        self.assertEqual(str(self.app.turntable_button["state"]), "disabled")

    def test_turntable_is_offered_once_the_preview_starts(self):
        self._select_preview()
        self.app._start()
        self.assertEqual(str(self.app.turntable_button["state"]), "normal")

    def test_scan_button_renames_itself_while_previewing(self):
        self._select_preview()
        self.app._start()
        self.assertEqual(self.app.scan_button["text"], "Capture this view to STL")

    def test_turntable_stays_shut_in_the_other_modes(self):
        self.app.mode_var.set(MODE_LABELS[Mode.COLOR])
        self.app._on_mode_changed()
        self.app._start()
        self.assertEqual(str(self.app.turntable_button["state"]), "disabled")
        self.assertEqual(self.app.scan_button["text"], "Scan to 3D model (STL)")

    def test_turntable_is_withdrawn_again_on_stop(self):
        self._select_preview()
        self.app._start()
        # The engine reports it has stopped while its thread is still
        # unwinding, so asking the engine here would still answer "running".
        self.app._on_engine_stopped()
        self.assertEqual(str(self.app.turntable_button["state"]), "disabled")
        self.assertEqual(self.app.scan_button["text"], "Scan to 3D model (STL)")


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
