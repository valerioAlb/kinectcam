"""Stability testing: why frames go missing.

The point is to separate three causes that all look the same on screen:

- **Exposure.** In low light the colour camera doubles its exposure time and
  goes from 30 to 15 fps. That is normal and expected: the intervals stay
  regular, they are simply longer.
- **Dropped frames.** When the link cannot keep up, the sensor produces the
  frame but it never arrives. It shows up as a jump in the sensor's own
  clock: its timestamp advances by two or more periods while nothing was
  seen in between.
- **Software falling behind.** The frame did arrive but the application did
  not collect it in time. Here the sensor clock is regular while the arrival
  times are not.

The distinction comes from comparing the sensor's timestamps (RelativeTime,
which advances at a fixed rate regardless of us) with arrival times.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

from .kinect_native import TICKS_PER_SECOND, KinectError, KinectSensor

COLOR = "colour"
DEPTH = "depth"

# Nominal sensor period: 30 fps.
NOMINAL_PERIOD_MS = 1000.0 / 30.0

# Tolerance when recognising an interval as a multiple of the period.
PERIOD_TOLERANCE = 0.35

# Above this share of dropped frames the link becomes suspect. On a healthy
# setup measured with this same tool, background loss sits around 2%: that is
# the normal cost of isochronous USB transfer, where late packets are dropped
# rather than retransmitted. The threshold is deliberately higher so the tool
# does not cry failure over that noise.
DROP_WARNING_PERCENT = 5.0

DEFAULT_DURATION = 12.0


@dataclass
class StreamReport:
    name: str
    frames: int = 0
    duration: float = 0.0
    sensor_gaps: list = field(default_factory=list)
    arrival_gaps: list = field(default_factory=list)
    dropped: int = 0
    started: bool = False

    @property
    def fps(self) -> float:
        # Divided by the span the measured frames actually cover, not by the
        # requested window: a gap before the stream starts used to drag the
        # average down and make a full rate look halved.
        if self.frames < 2 or self.duration <= 0:
            return 0.0
        return (self.frames - 1) / self.duration

    @property
    def median_period_ms(self) -> float:
        return float(np.median(self.sensor_gaps)) if self.sensor_gaps else 0.0

    @property
    def drop_percent(self) -> float:
        expected = self.frames + self.dropped
        return 100.0 * self.dropped / expected if expected else 0.0

    @property
    def worst_arrival_gap_ms(self) -> float:
        return float(np.max(self.arrival_gaps)) if self.arrival_gaps else 0.0

    @property
    def exposure_halved(self) -> bool:
        """The sensor is running at half rate because of low light."""
        return self.median_period_ms > NOMINAL_PERIOD_MS * 1.5


# Consecutive close-together frames that mean "the stream is flowing now".
WARMUP_STEADY_FRAMES = 3
WARMUP_STEADY_GAP = 0.15


def _warm_up(stream, timeout: float) -> bool:
    """Waits for the stream to genuinely flow before starting the clock.

    Counting the first frames is not enough: on opening, the stream hands
    over the frame left in memory (sometimes two in a row) and then goes
    quiet for several seconds. Seeing a few closely spaced frames in
    succession is the only reliable sign that the cadence has started.
    """
    deadline = time.perf_counter() + timeout
    steady = 0
    previous = None
    while time.perf_counter() < deadline:
        if stream.read() is None:
            time.sleep(0.002)
            continue
        now = time.perf_counter()
        if previous is not None and now - previous < WARMUP_STEADY_GAP:
            steady += 1
            if steady >= WARMUP_STEADY_FRAMES:
                return True
        else:
            steady = 0
        previous = now
    return False


def _measure(stream, duration: float, warmup_timeout: float = 15.0) -> StreamReport:
    report = StreamReport(name="")
    if not _warm_up(stream, warmup_timeout):
        return report
    report.started = True

    last_ticks = None
    last_arrival = None
    first_arrival = None
    start = time.perf_counter()

    while time.perf_counter() - start < duration:
        if stream.read() is None:
            time.sleep(0.002)
            continue

        now = time.perf_counter()
        ticks = stream.relative_time
        report.frames += 1
        if first_arrival is None:
            first_arrival = now

        if last_ticks is not None and ticks > last_ticks:
            sensor_gap_ms = (ticks - last_ticks) * 1000.0 / TICKS_PER_SECOND
            report.sensor_gaps.append(sensor_gap_ms)
            report.arrival_gaps.append((now - last_arrival) * 1000.0)
        last_ticks = ticks
        last_arrival = now

    if first_arrival is not None and last_arrival is not None:
        report.duration = last_arrival - first_arrival
    report.dropped = _count_dropped(report.sensor_gaps)
    return report


def _count_dropped(sensor_gaps) -> int:
    """Frames the sensor produced that we never saw.

    The reference period is the median of the intervals, not the nominal
    33 ms: that way a camera legitimately running at 15 fps is not counted as
    losing every other frame.
    """
    if not sensor_gaps:
        return 0
    period = float(np.median(sensor_gaps))
    if period <= 0:
        return 0
    missing = 0
    for gap in sensor_gaps:
        multiples = gap / period
        if multiples > 1.0 + PERIOD_TOLERANCE:
            missing += int(round(multiples)) - 1
    return missing


def run(duration: float = DEFAULT_DURATION, progress=None) -> dict:
    """Measures colour and depth and returns both reports.

    The two streams share the cable, the adapter and the USB port: if only
    colour degrades, the link cannot be the cause.
    """
    def announce(message):
        if progress is not None:
            progress(message)

    reports = {}
    sensor = KinectSensor()
    try:
        sensor.open()
        if not sensor.wait_until_available(timeout=8.0):
            raise KinectError("The sensor is not responding: cannot run the test.")

        for label, open_stream in (
            (COLOR, sensor.color_stream),
            (DEPTH, sensor.depth_stream),
        ):
            announce(f"Measuring the {label} stream for {duration:.0f} seconds...")
            stream = open_stream()
            report = _measure(stream, duration)
            report.name = label
            reports[label] = report
    finally:
        sensor.close()
    return reports


def interpret(reports: dict) -> list:
    """From the numbers to a verdict."""
    color = reports.get(COLOR)
    depth = reports.get(DEPTH)
    lines = []

    for report in (color, depth):
        if report is None:
            continue
        if not report.started:
            lines.append(f"{report.name}: no frames, the stream never started")
            continue
        lines.append(
            f"{report.name}: {report.fps:.1f} fps, one frame every "
            f"{report.median_period_ms:.0f} ms, {report.dropped} dropped "
            f"({report.drop_percent:.1f}%), longest wait "
            f"{report.worst_arrival_gap_ms:.0f} ms"
        )

    lines.append("")
    if color is None or depth is None:
        lines.append("Not enough data for a verdict.")
        return lines

    # The comparison only holds if both produced data: it is the fact that
    # they share cable and adapter that makes the diagnosis possible.
    if not color.started or not depth.started:
        silent = COLOR if not color.started else DEPTH
        lines.append(
            f"VERDICT: the {silent} stream never started, so there is nothing "
            "to compare. Usually this is the slow restart after a mode change: "
            "close other applications using the Kinect, wait a few seconds and "
            "repeat the test."
        )
        return lines

    both_dropping = (
        color.drop_percent > DROP_WARNING_PERCENT
        and depth.drop_percent > DROP_WARNING_PERCENT
    )
    link_suspect = both_dropping

    if both_dropping:
        lines.append(
            "VERDICT: frames are being lost on both streams. That points at the "
            "LINK, not the sensor: cable, adapter or USB port. Try a different "
            "direct USB 3.0 port, without a hub, and check the adapter's power "
            "supply."
        )
    elif color.exposure_halved and depth.drop_percent <= DROP_WARNING_PERCENT:
        lines.append(
            "VERDICT: this is LIGHT, not a fault. Depth runs steadily over the "
            "same cable and adapter, so the link is fine. The colour camera has "
            "doubled its exposure time because there is little light, and at "
            "that point it can only halve its frame rate. Turn a light on, or "
            "use night vision, which lights the scene itself and stays at 30 fps."
        )
    elif color.drop_percent > DROP_WARNING_PERCENT:
        lines.append(
            "VERDICT: only colour is losing frames while depth is steady. The "
            "link holds; the problem is in the colour stream, which is the "
            "heaviest in bandwidth. Try freeing the USB 3.0 port of other "
            "high-speed devices."
        )
    else:
        lines.append(
            "VERDICT: no significant loss. The sensor and the link are behaving "
            "as they should."
        )

    worst_wait = max(color.worst_arrival_gap_ms, depth.worst_arrival_gap_ms)
    if worst_wait > 500 and not link_suspect:
        lines.append(
            f"Note: an isolated {worst_wait:.0f} ms wait with no dropped frames "
            "is typical of the sensor spinning up after opening, not of a fault."
        )
    return lines


def format_report(reports: dict) -> str:
    lines = ["=== Stability test ===", ""]
    lines.extend(interpret(reports))
    return "\n".join(lines)


# --- freeze hunting ------------------------------------------------------

# Below this threshold it is a dropped frame; above it, a visible freeze.
STALL_SECONDS = 0.5

MONITOR_DURATION = 90.0

# How regular the intervals between freezes must be before calling it a
# cycle. This is relative spread: 0.35 means the times stay within a third
# of their typical value.
REGULARITY_LIMIT = 0.35

# How often to ask the runtime whether the sensor is still there while frozen.
AVAILABILITY_PROBE_SECONDS = 0.25


@dataclass
class FreezeEvent:
    at: float
    duration: float
    # Did the runtime report the sensor as gone while it was frozen?
    sensor_offline: bool = False
    # How far the sensor's own clock moved across the gap, in milliseconds.
    # A clock that kept running means the device stayed alive and the frames
    # simply did not reach us; a clock that stood still means the device
    # itself stopped producing them.
    clock_advanced_ms: float = 0.0

    def clock_kept_running(self, tolerance: float = 0.5) -> bool:
        expected = self.duration * 1000.0
        if expected <= 0:
            return True
        return self.clock_advanced_ms >= expected * tolerance


@dataclass
class MonitorReport:
    duration: float = 0.0
    frames: int = 0
    events: list = field(default_factory=list)
    started: bool = False
    on_battery: bool = False
    unsupported_controller: bool = False
    # USB/PnP events Windows logged while watching. -1 means unread.
    windows_usb_events: int = -1
    # The depth stream watched over the same timeline. Colour is by far the
    # heaviest stream in bandwidth, so comparing the two separates a link
    # that cannot carry colour from a device that stops altogether.
    depth_frames: int = 0
    depth_events: list = field(default_factory=list)

    @property
    def fps(self) -> float:
        return self.frames / self.duration if self.duration else 0.0

    @property
    def intervals(self) -> list:
        """Seconds between the start of one freeze and the start of the next."""
        starts = [event.at for event in self.events]
        return [b - a for a, b in zip(starts, starts[1:])]

    @property
    def median_interval(self) -> float:
        return float(np.median(self.intervals)) if self.intervals else 0.0

    @property
    def median_duration(self) -> float:
        return float(np.median([e.duration for e in self.events])) if self.events else 0.0

    @property
    def dropped_off_count(self) -> int:
        """Freezes during which the runtime stopped seeing the device at all."""
        return sum(1 for event in self.events if event.sensor_offline)

    @property
    def clock_kept_running_count(self) -> int:
        """Freezes the sensor's own clock ran straight through."""
        return sum(1 for event in self.events if event.clock_kept_running())

    @property
    def is_periodic(self) -> bool:
        """The freezes return at regular intervals, i.e. it is a cycle."""
        intervals = self.intervals
        if len(intervals) < 3:
            return False
        typical = float(np.median(intervals))
        if typical <= 0:
            return False
        spread = float(np.std(intervals)) / typical
        return spread < REGULARITY_LIMIT


def monitor(duration: float = MONITOR_DURATION, stall_seconds: float = STALL_SECONDS,
            progress=None, on_event=None) -> MonitorReport:
    """Watches the colour stream for a long stretch and logs every freeze.

    The stability test averages over a few seconds: a freeze that returns
    every ten either escapes it or poisons its statistics. Here the
    individual episodes are recorded instead, with how long they last and how
    often they return, because their regularity is what reveals the cause.
    """
    from . import diagnostics

    report = MonitorReport()
    report.on_battery = diagnostics.on_ac_power() is False
    report.unsupported_controller = diagnostics.has_supported_usb_controller() is False

    sensor = KinectSensor()
    try:
        sensor.open()
        if not sensor.wait_until_available(timeout=8.0):
            raise KinectError("The sensor is not responding: nothing to observe.")
        stream = sensor.color_stream()
        depth = sensor.depth_stream()

        if progress:
            progress("Waiting for the stream to start...")
        if not _warm_up(stream, timeout=20.0):
            return report
        report.started = True

        if progress:
            progress(f"Watching for {duration:.0f} seconds, leave everything alone...")

        start = time.perf_counter()
        last_frame_at = start
        last_ticks = stream.relative_time
        stall_from = None
        went_offline = False
        next_probe = 0.0

        depth_last_at = start
        depth_stall_from = None

        while time.perf_counter() - start < duration:
            # Depth is watched on the same timeline, in the same loop, so the
            # two streams can be compared instant by instant.
            if depth.read() is not None:
                depth_now = time.perf_counter()
                report.depth_frames += 1
                if depth_stall_from is not None:
                    report.depth_events.append(FreezeEvent(
                        at=depth_stall_from - start,
                        duration=depth_now - depth_stall_from,
                    ))
                    depth_stall_from = None
                depth_last_at = depth_now
            elif (depth_stall_from is None
                  and time.perf_counter() - depth_last_at > stall_seconds):
                depth_stall_from = depth_last_at

            if stream.read() is None:
                now = time.perf_counter()
                if stall_from is None and now - last_frame_at > stall_seconds:
                    stall_from = last_frame_at
                    went_offline = False
                    next_probe = now
                # While frozen, ask the runtime whether the device is still
                # there. A sensor that drops off the bus and comes back is a
                # different fault from one that stays present but stops
                # sending, and only this tells them apart.
                if stall_from is not None and now >= next_probe:
                    next_probe = now + AVAILABILITY_PROBE_SECONDS
                    if not sensor.is_available:
                        went_offline = True
                time.sleep(0.002)
                continue

            now = time.perf_counter()
            report.frames += 1
            if stall_from is not None:
                advanced = (stream.relative_time - last_ticks) * 1000.0 / TICKS_PER_SECOND
                event = FreezeEvent(
                    at=stall_from - start,
                    duration=now - stall_from,
                    sensor_offline=went_offline,
                    clock_advanced_ms=max(0.0, advanced),
                )
                report.events.append(event)
                if on_event:
                    on_event(
                        f"freeze at {event.at:.0f}s, lasting {event.duration:.1f}s "
                        f"(device {'dropped off' if went_offline else 'stayed present'}, "
                        f"sensor clock {'kept running' if event.clock_kept_running() else 'stood still'})"
                    )
                stall_from = None
            last_ticks = stream.relative_time
            last_frame_at = now

        report.duration = time.perf_counter() - start
    finally:
        sensor.close()

    if report.events:
        if progress:
            progress("Checking what Windows logged while watching...")
        report.windows_usb_events = diagnostics.recent_usb_events(
            minutes=max(2.0, report.duration / 60.0 + 1.0)
        )
    return report


def interpret_monitor(report: MonitorReport) -> list:
    lines = []
    if not report.started:
        lines.append(
            "The stream never started, so there is nothing to observe. Close "
            "other applications using the Kinect and try again."
        )
        return lines

    lines.append(
        f"Watched {report.duration:.0f} seconds, {report.frames} frames "
        f"({report.fps:.1f} fps), freezes detected: {len(report.events)}"
    )

    if not report.events:
        lines.append("")
        lines.append(
            "No freezes in this session. If the problem only appears while the "
            "virtual camera is streaming, start it and repeat: the higher load "
            "may be exactly what triggers it."
        )
        return lines

    lines.append(
        f"Typical freeze length: {report.median_duration:.1f}s"
        + (f", roughly every {report.median_interval:.0f}s" if report.intervals else "")
    )

    offline = report.dropped_off_count
    running = report.clock_kept_running_count
    total = len(report.events)
    lines.append(
        f"The runtime lost sight of the device in {offline} of {total} freezes. "
        f"Its frame clock ran through the gap in {running} of {total}."
    )
    lines.append("")

    # The availability flag is the decisive one. The clock is corroboration:
    # it confirms the gap was real elapsed time and the runtime session
    # survived, rather than our own reader having stalled.
    if offline > total / 2:
        lines.append(
            "VERDICT: the runtime lost the device entirely, every time. The "
            "regularity fits a reset loop: it comes up, runs for a few "
            "seconds, drops, and does it again."
        )
        lines.append("")

        # Windows notices a device that really leaves the bus. Its silence
        # says the reset happened above the USB layer, which points somewhere
        # completely different from a power fault.
        logged = report.windows_usb_events
        if logged > 0:
            lines.append(
                f"Windows logged {logged} USB or PnP events while watching, so "
                "the device really is re-enumerating at the operating system "
                "level. That is electrical: power delivery, the adapter, or a "
                "connection."
            )
        elif logged == 0:
            lines.append(
                "Windows logged NO USB or PnP events while watching. A device "
                "that is torn down and re-detected makes Windows load its "
                "driver again and say so, so the device is staying on the bus "
                "and it is the streaming pipeline that keeps dropping. That "
                "does not rule out a link-level reset, which Windows does not "
                "log, but it does rule out the sensor disappearing outright."
            )
        lines.append("")

        # Colour is several times the bandwidth of depth. If depth survives
        # the same instants that kill colour, the device is fine and the link
        # cannot carry the heavy stream.
        depth_stalls = len(report.depth_events)
        if report.depth_frames:
            lines.append(
                f"Watched alongside colour, depth took {report.depth_frames} "
                f"frames and stalled {depth_stalls} times."
            )
            if depth_stalls == 0:
                lines.append(
                    "DEPTH KEPT RUNNING THROUGH ALL OF IT. The sensor was alive "
                    "and delivering the whole time, so neither power nor the "
                    "device is at fault: only the colour stream is failing, and "
                    "it is the one that needs several times the bandwidth. Look "
                    "at the USB 3 link itself: the adapter's cable, the port, "
                    "and anything else sharing that controller. As a workaround "
                    "the infrared and depth modes will run normally."
                )
            elif depth_stalls >= max(1, len(report.events) - 1):
                lines.append(
                    "DEPTH STALLED TOO, at much the same rate. The whole "
                    "pipeline is going down, not just the heavy stream, which "
                    "points at the sensor or the Kinect service rather than at "
                    "bandwidth. Restarting the KinectMonitor service is worth a "
                    "try, and so is checking that the sensor's fan spins."
                )
        lines.append("")
        if logged != 0:
            lines.append(
                "POWER IS THE FIRST THING TO RULE OUT. The Kinect v2 needs 12 V "
                "at 2.67 A (32 W). Read the label on the brick: 12 V 1.08 A is "
                "the Xbox 360 Kinect v1 supply, which looks almost identical "
                "and delivers under half the current, and third-party adapters "
                "are often underrated."
            )
            lines.append("")
            lines.append(
                "A correct label does not clear it, and neither does a good "
                "reading at the brick. Measure while the sensor is streaming, "
                "and remember the voltage that matters is the one arriving at "
                "the sensor: a worn connector drops over a volt at nearly 3 A "
                "while the supply itself still reads 12. If a 6 second freeze "
                "shows no dip at all, the supply is holding and the fault is "
                "further along: the adapter, its connector, or the cable."
            )
            lines.append("")
            lines.append(
                "If the sensor behaves on an Xbox, that does not clear any of "
                "this. An Xbox powers the sensor from the console, so the "
                "adapter and its brick are exactly the parts it never uses."
            )
    elif not report.is_periodic:
        lines.append(
            "VERDICT: the freezes are IRREGULAR and the device stayed present. "
            "Typical of an unreliable contact: try another direct USB 3.0 port "
            "and check that the adapter's power supply is firmly seated."
        )
    elif total and running == 0:
        lines.append(
            "VERDICT: the device stayed present but its frame clock stood "
            "still, so it stopped producing frames rather than losing them in "
            "transit. That points at the sensor or the runtime, not the cable."
        )
    else:
        lines.append(
            "VERDICT: the device stayed present and its clock ran straight "
            "through the gap, so the frames never reached this machine. That "
            "points at the link: bandwidth, the host controller, or power "
            "management on the port."
        )

    # These only matter when the device did not simply drop off the bus,
    # which would otherwise leave two contradictory explanations on screen.
    if offline <= total / 2:
        if report.unsupported_controller:
            lines.append("")
            lines.append(
                "LIKELY CAUSE: this machine has no Intel or Renesas USB 3.0 "
                "controller, and those are the only ones Microsoft supports for "
                "the Kinect v2. Changing port will not help if every port hangs "
                "off the same chipset; a PCIe card with a Renesas uPD720202 is "
                "the usual fix."
            )
        if report.on_battery:
            lines.append("")
            lines.append(
                "ALSO: this machine is running ON BATTERY. Windows suspends USB "
                "ports in that state. Plug into mains power and repeat the "
                "test: it costs nothing to rule out."
            )
    return lines


def format_monitor(report: MonitorReport) -> str:
    lines = ["=== Freeze hunt ===", ""]
    lines.extend(interpret_monitor(report))
    return "\n".join(lines)
