"""The KinectCam window: starts the bridge and shows what the sensor sees."""

from __future__ import annotations

import threading
import tkinter as tk
from tkinter import messagebox, ttk

from PIL import Image, ImageTk

from . import diagnostics, frames, stability
from .engine import (
    DEFAULT_OUTPUT_SIZE,
    FAR_LIMIT_MM,
    MATTE_SOURCES,
    MODE_LABELS,
    NEAR_LIMIT_MM,
    OUTPUT_SIZES,
    CaptureEngine,
    CaptureSettings,
    MatteSource,
    Mode,
)
from .kinect_native import KinectError

PREVIEW_SIZE = (480, 270)
PREVIEW_INTERVAL_MS = 33  # ~30 fps

LABEL_TO_MODE = {label: mode for mode, label in MODE_LABELS.items()}

# Background removal costs: 1280x720 runs at 22-28 fps, 960x540 reaches 30.
# Worth saying up front rather than leaving it to be discovered.
BACKGROUND_HINT = (
    "Tip: with background removal, 960x540 keeps a full 30 fps."
)

PEOPLE_HINT = (
    "Silhouette matting follows the outline of the person instead of cutting "
    "at a fixed distance, but body tracking needs to see head and torso at "
    "roughly 1.5-3 m. If it locks onto nobody, KinectCam falls back to distance."
)


class KinectCamApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.engine = CaptureEngine(
            on_status=self._post_status,
            on_error=self._post_error,
            on_stopped=self._post_stopped,
        )
        self._preview_image = None  # tkinter keeps no reference of its own

        root.title("KinectCam - Kinect v2 as a webcam")
        root.resizable(False, False)
        self._build_ui()
        self._blank_preview()
        self._sync_background_controls()
        self.root.after(PREVIEW_INTERVAL_MS, self._refresh_preview)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._run_diagnostics(include_sensor=False)

    # --- building the interface -----------------------------------------

    def _build_ui(self):
        outer = ttk.Frame(self.root, padding=12)
        outer.grid(row=0, column=0, sticky="nsew")

        self.preview_label = ttk.Label(outer, relief="sunken", anchor="center")
        self.preview_label.grid(row=0, column=0, rowspan=4, padx=(0, 12), sticky="n")

        controls = ttk.LabelFrame(outer, text="Settings", padding=10)
        controls.grid(row=0, column=1, sticky="new")
        controls.columnconfigure(0, weight=1)

        ttk.Label(controls, text="Mode").grid(row=0, column=0, sticky="w")
        self.mode_var = tk.StringVar(value=MODE_LABELS[Mode.COLOR])
        self.mode_box = ttk.Combobox(
            controls, textvariable=self.mode_var, state="readonly", width=28,
            values=list(MODE_LABELS.values()),
        )
        self.mode_box.grid(row=1, column=0, pady=(0, 8), sticky="ew")
        self.mode_box.bind("<<ComboboxSelected>>", self._on_mode_changed)

        ttk.Label(controls, text="Output resolution").grid(row=2, column=0, sticky="w")
        self.size_var = tk.StringVar(value=DEFAULT_OUTPUT_SIZE)
        self.size_box = ttk.Combobox(
            controls, textvariable=self.size_var, state="readonly", width=28,
            values=list(OUTPUT_SIZES.keys()),
        )
        self.size_box.grid(row=3, column=0, pady=(0, 8), sticky="ew")

        self.background_label = ttk.Label(controls, text="Background")
        self.background_label.grid(row=4, column=0, sticky="w")
        self.background_var = tk.StringVar(value=frames.BACKGROUND_BLUR)
        self.background_box = ttk.Combobox(
            controls, textvariable=self.background_var, state="readonly", width=28,
            values=list(frames.BACKGROUND_MODES),
        )
        self.background_box.grid(row=5, column=0, pady=(0, 8), sticky="ew")
        self.background_box.bind("<<ComboboxSelected>>", self._on_live_changed)

        self.matte_label = ttk.Label(controls, text="Cut out by")
        self.matte_label.grid(row=6, column=0, sticky="w")
        self.matte_var = tk.StringVar(value=MatteSource.DISTANCE)
        self.matte_box = ttk.Combobox(
            controls, textvariable=self.matte_var, state="readonly", width=28,
            values=list(MATTE_SOURCES),
        )
        self.matte_box.grid(row=7, column=0, pady=(0, 8), sticky="ew")
        self.matte_box.bind("<<ComboboxSelected>>", self._on_matte_source_changed)

        self.distance_label = ttk.Label(controls, text="Keep whatever is within 2.0 m")
        self.distance_label.grid(row=8, column=0, sticky="w")
        self.distance_var = tk.DoubleVar(value=2.0)
        self.distance_scale = ttk.Scale(
            controls, from_=NEAR_LIMIT_MM / 1000.0, to=FAR_LIMIT_MM / 1000.0,
            variable=self.distance_var, command=self._on_distance_moved,
        )
        self.distance_scale.grid(row=9, column=0, pady=(0, 8), sticky="ew")

        self.mirror_var = tk.BooleanVar(value=True)
        self.mirror_check = ttk.Checkbutton(
            controls, text="Mirror image", variable=self.mirror_var
        )
        self.mirror_check.grid(row=10, column=0, sticky="w")

        self.rotate_var = tk.BooleanVar(value=False)
        self.rotate_check = ttk.Checkbutton(
            controls, text="Rotate 180 degrees (sensor mounted upside down)",
            variable=self.rotate_var,
        )
        self.rotate_check.grid(row=11, column=0, sticky="w")

        buttons = ttk.Frame(outer)
        buttons.grid(row=1, column=1, pady=10, sticky="ew")
        buttons.columnconfigure(0, weight=1)
        self.toggle_button = ttk.Button(
            buttons, text="Start virtual camera", command=self._toggle
        )
        self.toggle_button.grid(row=0, column=0, sticky="ew")
        ttk.Button(
            buttons, text="Diagnostics", command=lambda: self._run_diagnostics(True)
        ).grid(row=1, column=0, pady=(6, 0), sticky="ew")
        ttk.Button(
            buttons, text="Stability test", command=self._run_stability
        ).grid(row=2, column=0, pady=(6, 0), sticky="ew")
        ttk.Button(
            buttons, text="Hunt for freezes (90 s)", command=self._run_monitor
        ).grid(row=3, column=0, pady=(6, 0), sticky="ew")

        self.status_var = tk.StringVar(value="Ready.")
        ttk.Label(outer, textvariable=self.status_var, wraplength=280).grid(
            row=2, column=1, sticky="nw"
        )

        log_frame = ttk.LabelFrame(outer, text="Log", padding=6)
        log_frame.grid(row=4, column=0, columnspan=2, pady=(12, 0), sticky="ew")
        self.log = tk.Text(log_frame, height=9, width=96, wrap="word", state="disabled")
        self.log.grid(row=0, column=0, sticky="ew")
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=scrollbar.set)

    # --- log and status -------------------------------------------------

    def _append_log(self, text: str):
        self.log.configure(state="normal")
        self.log.insert("end", text.rstrip() + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _set_status(self, message: str):
        self.status_var.set(message)
        self._append_log(message)

    def _post_status(self, message):
        self.root.after(0, self._set_status, message)

    def _post_error(self, message):
        def show():
            self._set_status("Error.")
            self._append_log(message)
            messagebox.showerror("KinectCam", message, parent=self.root)
        self.root.after(0, show)

    def _post_stopped(self):
        self.root.after(0, self._on_engine_stopped)

    # --- controls that depend on the mode -------------------------------

    @property
    def _selected_mode(self) -> str:
        return LABEL_TO_MODE[self.mode_var.get()]

    def _sync_background_controls(self):
        """Background, matte source and distance only matter in that one mode.

        Distance is also disabled when cutting by silhouette, where it is not
        what decides: it stays only as the fallback when nobody is detected.
        """
        running = self.engine.running
        active = self._selected_mode == Mode.COLOR_NO_BACKGROUND
        by_distance = self.matte_var.get() == MatteSource.DISTANCE

        self.background_box.configure(state="readonly" if active else "disabled")
        self.background_label.configure(state="normal" if active else "disabled")

        # Changing the criterion means opening or closing body tracking, so it
        # is picked while the engine is stopped.
        can_pick = active and not running
        self.matte_box.configure(state="readonly" if can_pick else "disabled")
        self.matte_label.configure(state="normal" if can_pick else "disabled")

        tunable = active and by_distance
        self.distance_scale.configure(state="normal" if tunable else "disabled")
        self.distance_label.configure(state="normal" if tunable else "disabled")

    def _on_mode_changed(self, _event=None):
        self._sync_background_controls()
        if self._selected_mode == Mode.COLOR_NO_BACKGROUND:
            self._append_log(BACKGROUND_HINT)

    def _on_matte_source_changed(self, _event=None):
        self._sync_background_controls()
        if self.matte_var.get() == MatteSource.PEOPLE:
            self._append_log(PEOPLE_HINT)

    def _on_distance_moved(self, _value=None):
        self.distance_label.configure(
            text=f"Keep whatever is within {self.distance_var.get():.1f} m"
        )
        self._on_live_changed()

    def _on_live_changed(self, _event=None):
        """Background and distance can be changed while the engine runs."""
        if self.engine.running:
            self.engine.update_live_settings(
                background=self.background_var.get(),
                far_mm=int(self.distance_var.get() * 1000),
            )

    # --- start and stop -------------------------------------------------

    def _toggle(self):
        if self.engine.running:
            self._set_status("Stopping...")
            self.toggle_button.configure(state="disabled")
            threading.Thread(target=self.engine.stop, daemon=True).start()
        else:
            self._start()

    def _start(self):
        settings = CaptureSettings(
            mode=self._selected_mode,
            output_size=OUTPUT_SIZES[self.size_var.get()],
            mirror=self.mirror_var.get(),
            rotate=self.rotate_var.get(),
            background=self.background_var.get(),
            matte_source=self.matte_var.get(),
            near_mm=NEAR_LIMIT_MM,
            far_mm=int(self.distance_var.get() * 1000),
        )
        self._set_controls_enabled(False)
        self.toggle_button.configure(text="Stop virtual camera")
        self.engine.start(settings)

    def _on_engine_stopped(self):
        self._set_controls_enabled(True)
        self.toggle_button.configure(text="Start virtual camera", state="normal")
        self._blank_preview()
        if not self.status_var.get().startswith("Error"):
            self._set_status("Stopped.")

    def _set_controls_enabled(self, enabled: bool):
        """Changing mode or resolution requires reopening the sensor."""
        state = "readonly" if enabled else "disabled"
        self.mode_box.configure(state=state)
        self.size_box.configure(state=state)
        check_state = "normal" if enabled else "disabled"
        self.mirror_check.configure(state=check_state)
        self.rotate_check.configure(state=check_state)
        self._sync_background_controls()

    # --- diagnostics ----------------------------------------------------

    def _busy_with_sensor(self, what: str) -> bool:
        """The sensor can only be opened by one process at a time."""
        if not self.engine.running:
            return False
        messagebox.showinfo(
            "KinectCam",
            f"Stop the virtual camera before {what}: the sensor can only be "
            "opened by one process at a time.",
            parent=self.root,
        )
        return True

    def _run_diagnostics(self, include_sensor: bool):
        if include_sensor and self._busy_with_sensor("running full diagnostics"):
            return

        self._set_status("Running diagnostics...")

        def work():
            checks = diagnostics.run_checks(include_sensor=include_sensor)
            report = diagnostics.format_report(checks)
            failed = [check for check in checks if not check.ok]

            def show():
                self._append_log(report)
                if failed:
                    self._set_status(f"{len(failed)} check(s) failed: see the log.")
                else:
                    self._set_status("All good.")
            self.root.after(0, show)

        threading.Thread(target=work, name="kinectcam-diagnostics", daemon=True).start()

    def _run_stability(self):
        """Measures whether frames are dropping, and whether light or cable is to blame."""
        if self._busy_with_sensor("running the stability test"):
            return

        self._set_status("Stability test running, about half a minute...")

        def work():
            try:
                reports = stability.run(progress=self._post_status)
                report = stability.format_report(reports)
            except KinectError as exc:
                report = f"Test failed: {exc}"

            def show():
                self._append_log(report)
                self._set_status("Stability test complete: see the log.")
            self.root.after(0, show)

        threading.Thread(target=work, name="kinectcam-stability", daemon=True).start()

    def _run_monitor(self):
        """Watches for a long stretch to catch freezes that come and go."""
        if self._busy_with_sensor("hunting for freezes"):
            return

        self._set_status("Hunting for freezes: 90 seconds, leave everything alone...")

        def work():
            try:
                report = stability.monitor(
                    progress=self._post_status, on_event=self._post_log
                )
                text = stability.format_monitor(report)
            except KinectError as exc:
                text = f"Observation failed: {exc}"

            def show():
                self._append_log(text)
                self._set_status("Freeze hunt complete: see the log.")
            self.root.after(0, show)

        threading.Thread(target=work, name="kinectcam-monitor", daemon=True).start()

    def _post_log(self, message):
        self.root.after(0, self._append_log, message)

    # --- preview --------------------------------------------------------

    def _blank_preview(self):
        placeholder = Image.new("RGB", PREVIEW_SIZE, (28, 30, 38))
        self._preview_image = ImageTk.PhotoImage(placeholder)
        self.preview_label.configure(image=self._preview_image)

    def _refresh_preview(self):
        frame = self.engine.take_preview()
        if frame is not None:
            image = Image.fromarray(frame).resize(PREVIEW_SIZE, Image.BILINEAR)
            self._preview_image = ImageTk.PhotoImage(image)
            self.preview_label.configure(image=self._preview_image)
            if self.engine.fps:
                # Two different numbers on purpose: the sensor drops to 15 fps
                # in low light, while the output stays at 30 because the last
                # frame is republished. Showing only one of them misleads.
                self.status_var.set(
                    f"Live on {self.engine.device_name}\n"
                    f"sensor {self.engine.fps:.0f} fps - output {self.engine.output_fps:.0f} fps"
                )
        self.root.after(PREVIEW_INTERVAL_MS, self._refresh_preview)

    # --- shutdown -------------------------------------------------------

    def _on_close(self):
        if self.engine.running:
            self.engine.stop()
        self.root.destroy()


def main():
    root = tk.Tk()
    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:  # noqa: BLE001 - DPI scaling is a nice-to-have
        pass
    KinectCamApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
