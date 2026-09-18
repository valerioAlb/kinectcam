# KinectCam

[![tests](https://github.com/valerioAlb/kinectcam/actions/workflows/tests.yml/badge.svg)](https://github.com/valerioAlb/kinectcam/actions/workflows/tests.yml)
[![license: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Use a **Kinect v2 (Xbox One)** sensor as a webcam on **Windows 10/11**.

One executable: open it, press a button, and the sensor shows up as a camera in
Teams, Zoom, Meet, Discord, OBS or anything else that reads a webcam. It also
does things an ordinary webcam cannot: it sees in complete darkness, and it
removes your background using depth instead of a green screen.

> Not affiliated with or endorsed by Microsoft. "Kinect" and "Xbox" are
> trademarks of Microsoft Corporation, used here only to describe the hardware
> this project supports.

---

## Features

| Mode | What it does |
|---|---|
| **Colour** | The 1920x1080 RGB camera. The one you want as a normal webcam. |
| **Colour, background removed** | Same image, but the background is cut out using the depth sensor. No green screen. |
| **Night vision (infrared)** | The 512x424 infrared camera. The Kinect lights the scene with its own emitter, so it **sees with the lights off**. Exposure adapts to the scene automatically. |
| **Depth** | The raw 512x424 depth map, colour-coded from near (purple/blue) to far (yellow/red). |
| **3D scan preview** | A live, shaded 3D view of what a scan would capture, turnable to any angle. |
| **3D scan** | Captures the subject and writes a watertight STL, ready to slice and print. |
| **360 scan** | Merges a full turn's worth of views into one closed model. |

Plus: selectable output resolution, mirror, 180-degree rotation for an
upside-down sensor, a live preview, and three built-in diagnostic tools for
when the hardware misbehaves.

---

## Before anything else: you need the adapter

The Kinect v2 has a proprietary Xbox connector. **It is not a USB cable** and it
does not physically fit a PC. You need the **Kinect Adapter for Windows**
(external power supply + USB 3.0 cable). Without one, no software can help,
including this one. Microsoft discontinued it, but it is available second-hand
and as compatible clones.

| Requirement | Detail |
|---|---|
| Adapter | Kinect Adapter for Windows (or an equivalent clone) |
| Port | A **direct** USB 3.0 port on the machine, not a hub |
| Controller | An **Intel or Renesas** USB 3.0 host controller — see below |
| Power | The adapter's power supply must be connected |
| OS | 64-bit Windows 10 or 11 |

The Kinect v2 consumes most of a USB 3.0 controller's bandwidth: if you have
other fast devices on the same controller, move them.

### The USB controller matters more than you would expect

This is the requirement that catches people out. Microsoft only supports the
Kinect v2 on **Intel or Renesas** USB 3.0 host controllers. On AMD, ASMedia,
VIA or Fresco Logic chipsets the documented symptom is a degraded frame rate or
streams that keep stopping and restarting — and it is not subtle: on an
unsupported controller the sensor has been observed streaming for seven seconds
and freezing for seven, over and over.

Changing port does not help when every port hangs off the same chipset. The
usual fix is a PCIe USB 3.0 card with a **Renesas uPD720202** chipset, which
costs very little. KinectCam's diagnostics identify your controller's vendor
and say so outright.

---

## Why no new drivers are needed

A common assumption is that a Kinect v2 needs "updated drivers" to work on
modern Windows. It does not:

- Microsoft's **Kinect for Windows Runtime 2.x** driver is WHQL-signed and
  still installs and runs on 64-bit Windows 11.
- Writing a USB kernel driver from scratch would not be realistic, and on
  Windows 11 x64 it would not even load without Microsoft EV signing.

The piece that is genuinely missing is different: the Kinect v2 **does not
present itself as a UVC webcam**, so no application lists it among the
available cameras. It exposes its streams only through the runtime's COM API.

KinectCam is exactly that missing piece. It reads frames from the native API
and republishes them to a standard virtual camera every application recognises.

```
Kinect v2  ->  Microsoft driver  ->  Kinect20.dll  ->  KinectCam  ->  virtual camera  ->  Teams/Zoom/...
               (already exists)      (COM API)        (this)         (OBS Virtual Camera)
```

---

## Installation

1. **Unplug the Kinect.**
2. Right-click `install.ps1` -> *Run with PowerShell*. The script elevates
   itself, shows what it is about to download and asks for confirmation before
   doing anything. It installs:
   - **Kinect for Windows Runtime 2.2** (from microsoft.com, ~72 MB). Microsoft
     ships it as a raw redistributable package rather than a single installer,
     so the script installs the pieces in order: **VC++ 2012** (the runtime is
     built against it and it is usually missing on Windows 11, and newer
     redistributables do not replace it), then `KinectRuntime-x64.msi`, and
     finally it registers `kinectsensor.inf` into the driver store with
     `pnputil` so Windows binds it automatically.
   - **OBS Studio** via winget, which provides **OBS Virtual Camera**.
   - `KinectCam.exe` into `%LOCALAPPDATA%\Programs\KinectCam`, with a Start
     menu shortcut.
3. **Plug the Kinect back in** and wait for Windows to finish installing the
   driver (a couple of minutes the first time).
4. Launch **KinectCam** from the Start menu.

The order matters: the runtime must be installed with the sensor disconnected.
It is the most common reason installations fail.

You can also just run `dist\KinectCam.exe` directly, provided the runtime and a
virtual camera are already present.

Prefer to build it yourself? See [Development](#development).

---

## Usage

1. Open KinectCam. It runs a diagnostic pass at startup and writes the result
   to the log.
2. Pick a **mode** (see [Features](#features)).
3. Pick an **output resolution** and, if you like, **mirror** or **rotate 180
   degrees** (handy when the sensor is mounted upside down above a monitor).
4. Press **Start virtual camera**.
5. In your video app, select the **OBS Virtual Camera** device.

The preview in the window shows exactly what other applications are receiving.

### Background removal without a green screen

This is the thing an ordinary webcam cannot do. The Kinect measures the
**distance** of every point, so it separates you from the background
geometrically rather than by colour: no green screen, no fringing on hair
caused by an awkward background colour, and it works with the background in
near-darkness.

**Background**: `black`, `green` (to chroma key downstream, in OBS or Teams) or
`blur`. Changeable while the camera is running.

**Cut out by**: chosen while the camera is stopped, because it changes which
sensor sources have to be opened.

- `distance` — keeps everything within the metres set by the slider, adjustable
  live. Two metres keeps a person at a desk and leaves the room out.
  Predictable, and it works for objects too.
- `people` — uses the Kinect's body tracking, which follows up to six people.
  It follows the silhouette rather than cutting at a threshold, so the chair and
  the desk disappear even when they are as close as you are. In exchange, body
  tracking needs to see **head and torso, at roughly 1.5-3 m**: sitting too
  close it locks onto nobody. When that happens KinectCam **falls back to
  distance matting by itself** and says so in the log, rather than leaving you
  with a screen that is entirely background.

The two cameras are physically separate and have different fields of view, so
depth covers about **72% of the colour frame**: at the extreme edges there is no
distance data and those pixels become background. Stay reasonably centred.

### Scanning to a 3D-printable model

**Scan to 3D model (STL)** captures a burst of depth frames and writes a mesh
a slicer will accept.

What you get is a **relief, not a bust**. The Kinect is a time-of-flight
camera, not a scanner that orbits its subject: it sees one side of what is in
front of it, so a capture is a height field. Your face, not your head. The
surface is given a flat back and a rim joining the two, which turns an open
shell into a watertight solid, and that prints well as a plaque.

**Set the range with the preview, not by guessing.** Pick the **3D scan
preview** mode and press Start: the sensor's view appears as a shaded 3D
surface showing only what falls inside the distance range. Pull the distance
slider in until the room drops away and only you are left standing, and drag
the **3D view angle** slider to turn the model and see yourself in relief
before committing to a capture. What the preview shows is what the scan
records.

The preview runs at the sensor's full rate, so both sliders respond as you
move them.

How to get a good one:

- Sit **about 70 cm** from the sensor and hold still for a couple of seconds.
- Drag the **distance slider** down to just past yourself. It doubles as the
  scan's far limit, and leaving it at two metres captures the room as well.
- Even, indirect light. The depth camera does not care about light, but it
  does struggle with anything glossy, very dark, or hairy, which come out as
  holes. The holes are stitched shut rather than left open.

The model is written at **life size in millimetres**, so a face comes out
around 200 mm tall. Scale it in your slicer to whatever you actually want to
print.

### Scanning all the way round

**360 scan** merges a full turn into one closed model, so you get an object
rather than a relief. Start the preview, set the distance range, then press it
and **turn steadily through one full circle over thirty seconds**. Twelve views
are taken along the way, one every two and a half seconds, and the preview
keeps running in between so you can see yourself and hold position.

The views are merged as a **lathe around your axis of rotation**: for every
angle and height, the distance out to the surface. That closes all the way
round by construction and needs no seam, but it also means **undercuts fill
in**. A head works. A teapot with a handle does not: the gap under the handle
has no radius to record.

Two things decide whether it comes out sharp:

- **A steady pace.** The views are assumed to be evenly spaced around the
  circle, so rushing one part and dawdling in another puts the surface in the
  wrong place. Finishing exactly at 360 degrees matters less than keeping the
  speed constant.
- **Turning about a fixed axis.** Rotating on a swivel chair is ideal. Shuffling
  round on your feet moves the axis with you and smears the result.

The axis itself is worked out from the views rather than assumed, by finding
the position that makes them agree with each other. The log reports how well
they ended up agreeing: **under 10 mm is a good turn**, and above 15 mm
KinectCam says outright that the model will be smeared and why.

Thirty frames are combined with a median rather than an average, which
removes the sensor's speckle and discards the occasional wild reading instead
of averaging it in. Neighbouring pixels more than 2 cm apart in depth are not
joined, so the subject does not end up webbed to the wall behind it.

### The microphones: nothing to do

The Kinect has a **four-microphone array** with better noise cancellation than
most webcams, and it already works: the runtime installs an audio driver too, so
Windows exposes it as an ordinary microphone named
**"Microphone Array (Xbox NUI Sensor)"**.

It does not go through KinectCam and needs no configuration. Select it directly
in Teams, Zoom, Meet or in Windows sound settings, like any other microphone.
The diagnostics list it so you can confirm it is there.

---

## Diagnostics

Three buttons, for three different questions.

### Diagnostics

Checks the operating system, the runtime, the USB 3.0 controller, the power
source, whether Windows sees the sensor, the microphone array, the virtual
camera, and finally tries to actually open the sensor. Every failed check
explains what to do.

### If the camera freezes and restarts periodically

A different symptom from dropped frames, and a different cause: the image
freezes for a few seconds, then resumes, and it keeps happening. Work through
this in order.

**1. Turn off audio enhancements on the Kinect's microphone.** This is the
most common cause and the least intuitive one, because it has nothing to do
with video. With Windows audio enhancements enabled on the Kinect's microphone
array, the SDK restarts the sensor in a loop: it streams for a few seconds,
drops, is re-detected, and starts over. Muting that microphone does the same.

Settings → System → Sound → **More sound settings** → *Recording* tab →
right-click **Microphone Array - Xbox NUI Sensor** → Properties → *Advanced*
→ untick **Enable audio enhancements**. Check it is not muted while you are
there. KinectCam's diagnostics flag this directly.

Measured on a setup that had been failing for days: with enhancements on, the
sensor streamed for about six seconds and froze for about six, over and over,
every thirteen seconds. With them off, the same machine held **29.9 fps with
zero freezes**. Everything else had already been ruled out, including the
power supply, the USB controller and the cable.

**2. Check your USB controller.** Run **Diagnostics** and read the "USB 3.0
controller" line. If it says `(unsupported)`, that is very likely your answer:
see [The USB controller matters](#the-usb-controller-matters-more-than-you-would-expect).
No amount of cable swapping fixes an unsupported chipset.

**3. Plug the laptop into mains power.** Cheap to rule out, and the most common
cause on laptops. On battery Windows enables **USB selective suspend**, and the
Kinect v2 is precisely the peripheral that cannot cope:
it streams isochronously at full bandwidth and draws a lot of current. The port
gets suspended, the sensor stalls, the port wakes, the sensor restarts.
KinectCam's diagnostics check for and flag this condition.

To disable it on battery too, from an elevated PowerShell:

```powershell
powercfg /setdcvalueindex SCHEME_CURRENT 2a737441-1930-4402-8d77-b2bebba308a3 48e6b7a6-50f5-4782-a5d4-53bb8f07e226 0; powercfg /setactive SCHEME_CURRENT
```

**4. Press "Hunt for freezes".** It watches the stream for ninety seconds and
logs every episode: when it happens and how long it lasts. What matters is the
**regularity**:

| What the numbers show | Where it comes from |
|---|---|
| Freezes at regular intervals, e.g. every 10 s | **Something cycling**: the host controller renegotiating the link, or power management suspending the port. A hardware fault is not punctual like a clock. |
| Scattered, irregular freezes | **Marginal contact or power**: USB port, cable, adapter power supply. |
| No freezes while idle | Try again with the virtual camera running: the higher load may be what triggers it. |

**5. Change USB 3.0 port**, choosing a direct one on a different controller,
and check that the adapter's power supply is firmly seated.

A note on "it works fine on the Xbox": useful information, but read it
carefully. It clears **the sensor**, not the adapter, because the adapter is
only used on a PC. On an Xbox the sensor plugs into its dedicated port, which
supplies power and data without involving any of the hardware you use on a
computer.

### If frames go missing: the stability test

The **Stability test** answers "is it the adapter, the cable, or what?" without
guessing. It measures the colour stream and the depth stream for a dozen
seconds each, then gives a verdict.

Here is how it works. Every frame carries a timestamp set by the sensor, which
advances at a fixed rate regardless of what the PC does. Comparing those
timestamps with arrival times separates three causes that look identical on
screen:

| What the numbers show | What it means |
|---|---|
| Regular but long intervals (66 ms instead of 33) | **Low light.** The camera doubles its exposure time and halves its frame rate. Normal. |
| The sensor clock jumps, on **both** streams | **The link**: cable, adapter or USB port. |
| Jumps on **colour only** | Bandwidth: colour is the heaviest stream. |
| Regular clock, irregular arrivals | The PC is not collecting frames in time. |

Comparing the two streams is the key: **they share the same cable, the same
adapter and the same USB port**. If only colour degrades while depth stays
steady, the link is fine by definition, and the cause lies in the colour camera,
almost always the light.

On expectations: a loss of around **2% is normal** and invisible. The Kinect
transfers isochronously, where late packets are dropped rather than
retransmitted; the test only calls it a fault above 5%, and only when everything
degrades together.

---

## Practical notes

- The sensor can be opened by **one process at a time**: if OBS or another
  Kinect application is running, KinectCam cannot start.
- **Night vision and depth can take a few seconds to start**, especially coming
  from another mode. They rely on the infrared emitter, which has to warm back
  up after a close-together restart. Worse, the runtime immediately hands over a
  stale frame left in memory, so the preview looks alive while nothing is
  actually arriving yet. KinectCam detects this and writes it to the log ("No
  frames from the sensor for several seconds"), then reports when it recovers.
  There is nothing to fix: it is the sensor, just wait a few seconds.
- The window shows **two different frame rates, on purpose**: `sensor` is how
  many new frames actually arrive, `output` is how many the virtual camera
  publishes. In low light the colour camera drops from 30 to **15 fps** (the
  sensor does that, not the software), but the output stays at 30 by
  republishing the last frame, otherwise applications would see the stream
  freeze.
- Cost of background removal, measured on a laptop in good light:

  | Resolution | Black/green background | Blurred background |
  |---|---|---|
  | 1280x720 | 25-28 fps | 22 fps |
  | 960x540 | **30 fps** | 26 fps |
  | 640x360 | **30 fps** | **30 fps** |

  For a full 30 fps with background removal, use **960x540**. Every other mode
  runs at 30 fps.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| "no Kinect device present" | Adapter missing, unpowered, or on USB 2.0 |
| "opened but no data arriving" | Shared or hubbed USB 3.0 port: try a different direct port |
| "Virtual camera unavailable" | Open OBS Studio once and close it: the first launch registers the driver |
| "Kinect20.dll not found" | Runtime not installed: rerun `install.ps1` |
| The camera streams a few seconds, drops, repeats | Audio enhancements on the Kinect microphone. See [If the camera freezes](#if-the-camera-freezes-and-restarts-periodically) |
| "USB 3.0 controller: AMD (unsupported)" | Microsoft only supports Intel and Renesas controllers. A Renesas uPD720202 PCIe card is the usual fix. |
| "No frames from the sensor for several seconds" | The sensor stopped streaming and the image is frozen on the last frame. Almost always the USB 3.0 cable or the adapter's power supply. Stop and restart the virtual camera. |
| The camera freezes and restarts every few seconds | See [If the camera freezes and restarts periodically](#if-the-camera-freezes-and-restarts-periodically) |
| The runtime will not install | Clear `C:\ProgramData\Package Cache` and retry with the sensor disconnected |

---

## Development

```
kinectcam/
  kinect_native.py   access to Kinect20.dll via ctypes, calling COM vtables
  frames.py          frame conversion (colour, depth, infrared, mattes)
  engine.py          capture thread and publishing to the virtual camera
  diagnostics.py     environment checks
  stability.py       dropped-frame and freeze analysis
  scanning.py        depth frames to a watertight STL
  app.py             tkinter interface
tests/test_offline.py  tests that do not need the sensor
tests/test_scanning.py the scanning geometry, watertightness above all
```

The native module **does not use comtypes**: it calls COM vtable slots
directly. Those indices are magic numbers taken from the interfaces in
`Kinect.h` (SDK 2.0), and getting one wrong does not raise an exception, it
crashes the process. `tests/test_offline.py` therefore checks every index
against the method order declared in the header. If you touch those indices,
run the tests.

Build:

```powershell
.\build.ps1
```

It runs the test suite and produces `dist\KinectCam.exe` (~31 MB, single file).

During the build PyInstaller prints:

```
WARNING: Library Kinect20.dll required via ctypes not found
```

That is correct and should be ignored. `Kinect20.dll` **must not** end up inside
the executable: it belongs to the Microsoft runtime and has to be loaded from
`System32`, where the driver installs it. Bundling it would pin a version
different from the driver in use.

Run without building:

```powershell
.\.venv\Scripts\python.exe KinectCam.py
```

### Requirements

Python 3.10+, plus `numpy`, `pillow` and `pyvirtualcam` (see
`requirements.txt`). PyInstaller is only needed to build the executable.

---

## Known limitations

- **Audio does not go through here.** The microphones already work on their own
  as a Windows device (see above), so KinectCam does not touch them. What stays
  out of reach are the Kinect API's advanced audio features, such as knowing
  **which direction a voice is coming from**: using those would also require a
  virtual microphone driver.
- **A single scan is one-sided.** A time-of-flight camera sees the surface
  facing it, so one capture is a relief. Use the 360 scan for a closed model.
- **The 360 merge is a lathe, not a general reconstruction.** It records one
  radius per angle and height, which closes reliably and cheaply but cannot
  represent undercuts or anything hollow. The angles are assumed evenly spaced
  rather than measured, so the turn has to be steady.
- **No skeleton tracking.** The Kinect can track 25 joints per person
  (`IBodyFrameSource`). Only the body index is used here, i.e. which pixels
  belong to whom, not where the hands or the head are.
- **The matte has hard edges.** It is a binary threshold: along the outline you
  can see a step of a couple of pixels. A continuous alpha instead of a boolean
  mask would look cleaner, at a higher cost.
- **No image backgrounds.** You can pick black, green or blur, but not load your
  own photo.
- The virtual camera depends on OBS Studio. Any other driver supported by
  `pyvirtualcam` would do (Unity Capture, for instance), but the installer only
  sets up OBS.

The whole native path has been verified against a real `Kinect20.dll` **with the
sensor connected**: the four sources (colour, depth, infrared, body index), the
coordinate mapper, the pixel copy and publishing to the virtual camera in every
mode.

One thing could not be fully verified: the `people` matte **with a person
actually being tracked**. During testing nobody was in frame at full height, so
what was confirmed is that the stream opens, that frames arrive and that the
**fallback to distance triggers correctly** — but not how the outline looks on a
real person.

---

## Acknowledgements

The COM interface layouts were cross-checked against
[PyKinect2](https://github.com/Kinect/PyKinect2), which documents the vtable
ordering of the Kinect SDK 2.0 interfaces.

## License

MIT — see [LICENSE](LICENSE).
