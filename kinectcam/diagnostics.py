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


def check_usb3() -> Check:
    output = _run_powershell(
        "(Get-PnpDevice -Class USB -PresentOnly -ErrorAction SilentlyContinue | "
        "Where-Object { $_.FriendlyName -match 'xHCI|USB 3' } | "
        "Select-Object -ExpandProperty FriendlyName) -join '; '"
    )
    found = bool(output) and not output.startswith("(")
    return Check(
        "USB 3.0 controller",
        found,
        output if found else "no xHCI controller detected",
        "" if found else "The Kinect v2 requires USB 3.0; it will not start on USB 2.0.",
    )


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


def usb_suspend_on_battery():
    """True when Windows may suspend USB ports once running on battery."""
    output = _run_powershell(
        "$g = (powercfg /getactivescheme) -replace '.*GUID: ([0-9a-f-]+).*','$1'; "
        "(powercfg /query $g 2a737441-1930-4402-8d77-b2bebba308a3 "
        "48e6b7a6-50f5-4782-a5d4-53bb8f07e226) -join \"`n\""
    )
    match = re.search(r"Current DC Power Setting Index:\s*(0x[0-9a-fA-F]+)", output)
    if not match:
        return None
    return int(match.group(1), 16) != 0


def check_power() -> Check:
    """On battery Windows powers USB ports down, and the Kinect cannot cope.

    The sensor streams isochronously at full bandwidth and draws a lot of
    current: exactly the kind of peripheral that selective suspend pushes
    into a cyclic stall, with freezes of a few seconds that keep repeating.
    """
    ac = on_ac_power()
    suspend = usb_suspend_on_battery()

    if ac is None:
        return Check("Power source", True, "desktop, or state not detectable")

    source = "on mains" if ac else "ON BATTERY"
    if suspend is None:
        detail = f"{source}, USB setting unreadable"
    elif suspend:
        detail = f"{source}, USB selective suspend on battery ENABLED"
    else:
        detail = f"{source}, USB selective suspend on battery disabled"

    risky = (not ac) and suspend is not False
    return Check(
        "Power source", not risky, detail,
        "" if not risky else (
            "Plug the laptop into mains power before using the Kinect. On "
            "battery Windows suspends the USB ports and the sensor freezes "
            "intermittently. Alternatively, disable USB selective suspend on "
            "battery as well."
        ),
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
