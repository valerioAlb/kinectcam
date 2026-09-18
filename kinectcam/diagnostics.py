"""Environment checks: nearly every Kinect v2 problem lives here."""

from __future__ import annotations

import ctypes
import os
import platform
import re
import subprocess
import winreg
from dataclasses import dataclass

from .kinect_native import KinectError, KinectSensor, runtime_installed

# USB identifiers of the Kinect v2 and of its internal hub.
KINECT_V2_HARDWARE_IDS = ("VID_045E&PID_02C4", "VID_045E&PID_02D8", "VID_045E&PID_02D9")

# DirectShow filter registered by OBS for its virtual camera.
OBS_VIRTUAL_CAM_CLSID = "{A3FCE0F5-3493-419F-958A-ABA1250EC20B}"

_NO_WINDOW = 0x08000000


@dataclass
class Check:
    name: str
    ok: bool
    detail: str
    hint: str = ""


def _registry_key_exists(root, path) -> bool:
    try:
        with winreg.OpenKey(root, path):
            return True
    except OSError:
        return False


def _run_powershell(script: str, timeout: float = 25.0) -> str:
    """Runs PowerShell without flashing a console window."""
    try:
        result = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command", script],
            capture_output=True, text=True, timeout=timeout,
            creationflags=_NO_WINDOW,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return f"(could not query the system: {exc})"
    return (result.stdout or "").strip() or (result.stderr or "").strip()


# --- individual checks --------------------------------------------------

def check_windows() -> Check:
    is_64bit = platform.machine().endswith("64")
    detail = f"{platform.system()} {platform.release()} ({platform.machine()})"
    return Check(
        "Operating system", is_64bit, detail,
        "" if is_64bit else "The Kinect runtime is 64-bit only.",
    )


def check_runtime() -> Check:
    installed = runtime_installed()
    return Check(
        "Kinect for Windows Runtime 2.x",
        installed,
        "Kinect20.dll loaded" if installed else "Kinect20.dll not found",
        "" if installed else "Run install.ps1 to install the runtime.",
    )


# PCI vendor IDs of USB host controllers, and whether Microsoft lists them as
# supported for the Kinect v2. The sensor is unusually picky here: the official
# requirement is an Intel or Renesas controller, and the documented symptom on
# anything else is a degraded frame rate or streams that stop working.
# Vendor IDs are used rather than device names because names are localised.
USB_CONTROLLER_VENDORS = {
    "8086": ("Intel", True),
    "1912": ("Renesas", True),
    "1033": ("NEC/Renesas", True),
    "1022": ("AMD", False),
    "1b21": ("ASMedia", False),
    "1106": ("VIA", False),
    "1b73": ("Fresco Logic", False),
}

SUPPORTED_VENDOR_NAMES = "Intel or Renesas"


def _usb_host_controllers():
    """(vendor id, name) for every USB host controller present."""
    output = _run_powershell(
        "(Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | "
        "Where-Object { $_.Class -eq 'USB' -and $_.InstanceId -like 'PCI\\*' } | "
        "ForEach-Object { \"$($_.InstanceId)|$($_.FriendlyName)\" }) -join ';;'"
    )
    if not output or output.startswith("("):
        return []

    controllers = []
    for entry in output.split(";;"):
        instance_id, _, name = entry.partition("|")
        match = re.search(r"VEN_([0-9A-Fa-f]{4})", instance_id)
        controllers.append((match.group(1).lower() if match else "", name.strip()))
    return controllers


def classify_usb_controllers(controllers) -> Check:
    """Turns a list of (vendor id, name) into a verdict.

    Kept separate from querying the system so the decision can be tested
    against every chipset, including ones no development machine has.
    """
    if not controllers:
        return Check(
            "USB 3.0 controller", False, "no USB host controller detected",
            "The Kinect v2 requires USB 3.0; it will not start on USB 2.0.",
        )

    seen = {}
    for vendor_id, name in controllers:
        vendor, supported = USB_CONTROLLER_VENDORS.get(vendor_id, ("unknown", False))
        seen.setdefault(vendor, supported)

    supported_vendors = sorted(v for v, ok in seen.items() if ok)
    unsupported = sorted(v for v, ok in seen.items() if not ok)

    detail = ", ".join(
        f"{vendor}{'' if seen[vendor] else ' (unsupported)'}" for vendor in sorted(seen)
    )
    if supported_vendors:
        return Check("USB 3.0 controller", True, detail)

    return Check(
        "USB 3.0 controller", False, detail,
        f"Microsoft only supports {SUPPORTED_VENDOR_NAMES} USB 3.0 controllers for "
        f"the Kinect v2. On {', '.join(unsupported)} the documented symptom is "
        "exactly this: a degraded frame rate, or streams that keep stopping and "
        "restarting. If the sensor keeps freezing, a PCIe USB 3.0 card with a "
        "Renesas uPD720202 chipset is the usual fix.",
    )


def check_usb3() -> Check:
    """Which USB controllers are present, and whether any is one the Kinect likes."""
    return classify_usb_controllers(_usb_host_controllers())


def check_device_present() -> Check:
    conditions = " -or ".join(
        f"$_.InstanceId -match '{hw_id}'" for hw_id in KINECT_V2_HARDWARE_IDS
    )
    output = _run_powershell(
        "(Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | "
        f"Where-Object {{ {conditions} -or $_.FriendlyName -match 'Kinect' }} | "
        "ForEach-Object { \"$($_.FriendlyName) [$($_.Status)]\" }) -join '; '"
    )
    found = bool(output) and not output.startswith("(")
    return Check(
        "Sensor recognised by Windows",
        found,
        output if found else "no Kinect device present",
        "" if found else (
            "Connect the Kinect through the official adapter (power supply + "
            "USB 3.0). Without it the proprietary Xbox connector cannot reach a PC."
        ),
    )


class _SystemPowerStatus(ctypes.Structure):
    _fields_ = [
        ("ACLineStatus", ctypes.c_ubyte),
        ("BatteryFlag", ctypes.c_ubyte),
        ("BatteryLifePercent", ctypes.c_ubyte),
        ("SystemStatusFlag", ctypes.c_ubyte),
        ("BatteryLifeTime", ctypes.c_ulong),
        ("BatteryFullLifeTime", ctypes.c_ulong),
    ]


def on_ac_power():
    """True on mains, False on battery, None when the machine cannot say."""
    status = _SystemPowerStatus()
    if not ctypes.windll.kernel32.GetSystemPowerStatus(ctypes.byref(status)):
        return None
    if status.ACLineStatus == 255:
        return None
    return bool(status.ACLineStatus)


def usb_selective_suspend():
    """(on mains, on battery): True, False or None when unreadable.

    Both settings matter. A desktop has no battery profile at all, so reading
    only the battery one reports "unreadable" and tells the user nothing,
    while the setting that actually applies to them is the mains one.
    """
    output = _run_powershell(
        "$g = (powercfg /getactivescheme) -replace '.*GUID: ([0-9a-f-]+).*','$1'; "
        "(powercfg /query $g 2a737441-1930-4402-8d77-b2bebba308a3 "
        "48e6b7a6-50f5-4782-a5d4-53bb8f07e226) -join \"`n\""
    )

    def read(label):
        match = re.search(rf"Current {label} Power Setting Index:\s*(0x[0-9a-fA-F]+)", output)
        return None if not match else int(match.group(1), 16) != 0

    return read("AC"), read("DC")


def check_power() -> Check:
    """Windows powers USB ports down, and the Kinect cannot cope with that.

    The sensor streams isochronously at full bandwidth and draws a lot of
    current: exactly the kind of peripheral that selective suspend pushes
    into a cyclic stall, with freezes of a few seconds that keep repeating.
    """
    ac = on_ac_power()
    suspend_ac, suspend_dc = usb_selective_suspend()

    # The setting that applies right now is the one worth judging.
    on_mains = ac is not False
    active = suspend_ac if on_mains else suspend_dc
    source = "desktop or on mains" if ac is None else ("on mains" if ac else "ON BATTERY")

    if active is None:
        detail = f"{source}, USB selective suspend setting unreadable"
    elif active:
        detail = f"{source}, USB selective suspend ENABLED"
    else:
        detail = f"{source}, USB selective suspend disabled"

    # Only a setting read as enabled is worth a warning. An unreadable one is
    # not evidence of a problem, and failing the check on it would send people
    # chasing a fault that may not exist.
    hints = []
    if active is True:
        hints.append(
            "Disable USB selective suspend: Windows can power the port down "
            "under the sensor and the stream freezes and restarts in cycles."
        )
    if ac is False and suspend_dc is True:
        hints.append(
            "Plug the laptop into mains power: on battery Windows is far more "
            "aggressive about suspending USB ports."
        )

    return Check("Power source", not hints, detail, " ".join(hints))


def recent_usb_events(minutes: float = 5.0) -> int:
    """How many USB or PnP events Windows logged in the last few minutes.

    Independent corroboration for a device that appears to leave the bus. If
    the sensor is really being reset, Windows notices the device going away
    and coming back and records it, whatever our own runtime thinks.
    Returns -1 when the log cannot be read.
    """
    output = _run_powershell(
        f"$since = (Get-Date).AddMinutes(-{minutes:.0f}); "
        "@(Get-WinEvent -FilterHashtable @{LogName='System'; StartTime=$since} "
        "-ErrorAction SilentlyContinue | Where-Object { "
        "$_.ProviderName -match 'USB|Kernel-PnP|DriverFrameworks' -and "
        "$_.Message -match 'USB|Kinect|NUI' }).Count",
        timeout=40.0,
    )
    try:
        return int(output.strip().splitlines()[-1])
    except (ValueError, IndexError):
        return -1


def has_supported_usb_controller():
    """True when at least one Intel or Renesas controller is present.

    None when the controllers cannot be enumerated, so callers can stay quiet
    instead of guessing.
    """
    controllers = _usb_host_controllers()
    if not controllers:
        return None
    return any(
        USB_CONTROLLER_VENDORS.get(vendor_id, ("", False))[1]
        for vendor_id, _name in controllers
    )


def check_microphone() -> Check:
    """The microphone array does not go through KinectCam; the runtime installs it.

    The Kinect driver registers the four microphones as an ordinary Windows
    audio endpoint, so it is selected directly inside applications. This
    check only confirms it is there, and under what name.
    """
    output = _run_powershell(
        "(Get-PnpDevice -Class AudioEndpoint -PresentOnly -ErrorAction SilentlyContinue | "
        "Where-Object { $_.FriendlyName -match 'NUI|Kinect' } | "
        "ForEach-Object { \"$($_.FriendlyName) [$($_.Status)]\" }) -join '; '"
    )
    found = bool(output) and not output.startswith("(")
    return Check(
        "Kinect microphones",
        found,
        output if found else "microphone array not detected",
        "" if found else (
            "Not a blocker for video. If you need audio, reconnect the sensor "
            "and wait for Windows to finish installing the audio driver."
        ),
    )


def check_virtual_camera() -> Check:
    obs_installed = os.path.isdir(r"C:\Program Files\obs-studio")
    filter_registered = _registry_key_exists(
        winreg.HKEY_CLASSES_ROOT, f"CLSID\\{OBS_VIRTUAL_CAM_CLSID}"
    )
    ok = obs_installed and filter_registered
    if ok:
        detail = "OBS Virtual Camera registered"
    elif obs_installed:
        detail = "OBS installed but the virtual camera is not registered"
    else:
        detail = "OBS Studio not found"
    return Check(
        "Virtual camera", ok, detail,
        "" if ok else (
            "Install OBS Studio and launch it once: that registers the virtual "
            "camera driver KinectCam publishes to."
        ),
    )


def check_sensor_responds() -> Check:
    """Actually opens the sensor: the only check that really settles it."""
    if not runtime_installed():
        return Check("Sensor response", False, "runtime missing", "")
    sensor = None
    try:
        sensor = KinectSensor()
        sensor.open()
        available = sensor.wait_until_available(timeout=6.0)
        return Check(
            "Sensor response", available,
            "the sensor is streaming" if available else "opened but no data arriving",
            "" if available else (
                "Try another direct USB 3.0 port (not a hub) and check that the "
                "light on the adapter's power supply is on."
            ),
        )
    except KinectError as exc:
        return Check("Sensor response", False, str(exc), "")
    finally:
        if sensor is not None:
            sensor.close()


def run_checks(include_sensor: bool = True) -> list[Check]:
    checks = [
        check_windows(),
        check_runtime(),
        check_usb3(),
        check_power(),
        check_device_present(),
        check_microphone(),
        check_virtual_camera(),
    ]
    if include_sensor:
        checks.append(check_sensor_responds())
    return checks


def format_report(checks) -> str:
    lines = ["=== KinectCam diagnostics ===", ""]
    for check in checks:
        lines.append(f"[{'PASS' if check.ok else 'FAIL'}] {check.name}: {check.detail}")
        if check.hint and not check.ok:
            lines.append(f"       -> {check.hint}")
    return "\n".join(lines)
