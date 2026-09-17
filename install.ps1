# Prepares the machine for KinectCam: Kinect runtime, virtual camera, shortcut.
# Elevates itself. Before downloading anything it shows exactly what it will
# fetch and from where.
#
#   -Yes       skip the confirmation prompt and the final "press Enter"
#   -LogPath   also mirror all output to a file

param(
    [switch]$Yes,
    [string]$LogPath
)

$ErrorActionPreference = "Stop"

$KinectRuntimeUrl  = "https://download.microsoft.com/download/e/6/8/e688a436-5937-41cb-af76-498e99e19191/KinectRuntime-v2.2_1905.zip"
$KinectRuntimeSize = "about 72 MB"
$ObsWingetId       = "OBSProject.OBSStudio"

# --- elevation ----------------------------------------------------------

$identity = [Security.Principal.WindowsIdentity]::GetCurrent()
$principal = New-Object Security.Principal.WindowsPrincipal($identity)
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    Write-Host "Administrator rights are required, relaunching..." -ForegroundColor Yellow
    $relaunch = @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", "`"$PSCommandPath`"")
    if ($Yes)     { $relaunch += "-Yes" }
    if ($LogPath) { $relaunch += @("-LogPath", "`"$LogPath`"") }
    $elevated = Start-Process powershell -Verb RunAs -ArgumentList $relaunch -Wait -PassThru
    exit $elevated.ExitCode
}

# The transcript starts only here, after elevation: starting it earlier would
# leave the elevated process finding the file already open by its parent, and
# the log would capture the wrong window, the one that installs nothing.
if ($LogPath) {
    try { Start-Transcript -LiteralPath $LogPath -Force | Out-Null } catch { }
}

function Write-Step($text) {
    Write-Host ""
    Write-Host "=== $text ===" -ForegroundColor Cyan
}

# --- summary and consent ------------------------------------------------

Write-Host ""
Write-Host "KinectCam setup" -ForegroundColor Green
Write-Host ""
Write-Host "This will install:"
Write-Host "  1. Kinect for Windows Runtime 2.2 - downloaded from microsoft.com ($KinectRuntimeSize)"
Write-Host "     $KinectRuntimeUrl"
Write-Host "  2. OBS Studio (via winget) - it provides OBS Virtual Camera, the"
Write-Host "     virtual webcam driver KinectCam publishes to"
Write-Host "  3. KinectCam.exe into %LOCALAPPDATA%\Programs\KinectCam, with a Start menu shortcut"
Write-Host ""
Write-Host "IMPORTANT: unplug the Kinect before continuing." -ForegroundColor Yellow
Write-Host "The runtime must be installed with the sensor disconnected; plug it back in afterwards."
Write-Host ""
if ($Yes) {
    Write-Host "Continue? [y/N]: y (confirmed with -Yes)"
    $answer = "y"
} else {
    $answer = Read-Host "Continue? [y/N]"
}
if ($answer -notmatch '^[yY]') {
    Write-Host "Cancelled."
    return
}

# $env:TEMP can be in 8.3 short form (C:\Users\ABCDE~1.XYZ\...). With a tilde
# in the path, Remove-Item -Recurse fails with a PSArgumentException that not
# even -ErrorAction SilentlyContinue suppresses, so always work on the long form.
$workDir = Join-Path (Get-Item $env:TEMP).FullName "KinectCam-setup"
New-Item -ItemType Directory -Force $workDir | Out-Null

function Remove-TreeIfPresent([string]$Path) {
    if (Test-Path -LiteralPath $Path) {
        Remove-Item -LiteralPath $Path -Recurse -Force
    }
}

# --- 1. Kinect runtime --------------------------------------------------

Write-Step "Kinect for Windows Runtime 2.2"

$runtimePresent = Test-Path (Join-Path $env:WINDIR "System32\Kinect20.dll")
if ($runtimePresent) {
    Write-Host "Already present, skipping." -ForegroundColor Green
} else {
    $zip = Join-Path $workDir "KinectRuntime.zip"
    # A previous attempt may have downloaded the archive already: 72 MB should
    # not be fetched every time. Below 70 MB the file is truncated.
    $alreadyDownloaded = (Test-Path -LiteralPath $zip) -and
        ((Get-Item -LiteralPath $zip).Length -gt 70MB)
    if ($alreadyDownloaded) {
        Write-Host "Archive already downloaded, reusing it." -ForegroundColor Green
    } else {
        Write-Host "Downloading the runtime..."
        Invoke-WebRequest -Uri $KinectRuntimeUrl -OutFile $zip -UseBasicParsing
    }

    $extracted = Join-Path $workDir "runtime"
    Remove-TreeIfPresent $extracted
    Expand-Archive -LiteralPath $zip -DestinationPath $extracted -Force

    # The 2.2.1905 archive is not a bootstrapper: it is the raw redistributable
    # package, with the MSI, the driver and the VC++ runtimes as separate files.
    $msi = Get-ChildItem -LiteralPath $extracted -Filter "KinectRuntime-x64.msi" -Recurse |
        Select-Object -First 1
    if (-not $msi) {
        throw "KinectRuntime-x64.msi not found inside the downloaded archive."
    }
    $payload = $msi.Directory.FullName

    # The runtime is built against VC++ 2012, which is often missing on
    # Windows 11: the newer redistributables already installed do not replace it.
    # 1638 = a newer version is already present, which is a valid outcome.
    foreach ($name in "vcredist_x64.exe", "vc_redist.x64.exe") {
        $redist = Join-Path $payload $name
        if (-not (Test-Path -LiteralPath $redist)) { continue }
        Write-Host "Installing $name..."
        $proc = Start-Process -FilePath $redist `
            -ArgumentList "/install", "/quiet", "/norestart" -Wait -PassThru
        if ($proc.ExitCode -in @(0, 3010)) {
            Write-Host "  installed." -ForegroundColor Green
        } elseif ($proc.ExitCode -eq 1638) {
            Write-Host "  a newer version is already present, skipping." -ForegroundColor Green
        } else {
            Write-Host "  exit code $($proc.ExitCode): continuing anyway." -ForegroundColor Yellow
        }
    }

    Write-Host "Installing KinectRuntime-x64.msi..."
    $proc = Start-Process -FilePath "msiexec.exe" `
        -ArgumentList "/i", "`"$($msi.FullName)`"", "/quiet", "/norestart" -Wait -PassThru
    if ($proc.ExitCode -notin @(0, 3010)) {
        Write-Host "msiexec returned $($proc.ExitCode)." -ForegroundColor Yellow
        Write-Host "Retrying interactively: follow the installer windows." -ForegroundColor Yellow
        Start-Process -FilePath "msiexec.exe" -ArgumentList "/i", "`"$($msi.FullName)`"" -Wait
    }

    # The driver goes into the Windows driver store, so Windows binds it by
    # itself when the sensor is plugged back in.
    $inf = Get-ChildItem -LiteralPath $payload -Filter "kinectsensor.inf" | Select-Object -First 1
    if ($inf) {
        Write-Host "Registering the sensor driver..."
        $proc = Start-Process -FilePath "pnputil.exe" `
            -ArgumentList "/add-driver", "`"$($inf.FullName)`"", "/install" -Wait -PassThru
        if ($proc.ExitCode -in @(0, 259, 3010)) {
            Write-Host "  driver registered." -ForegroundColor Green
        } else {
            Write-Host "  pnputil returned $($proc.ExitCode)." -ForegroundColor Yellow
            Write-Host "  Not a blocker: Windows can still install it when the sensor is connected." -ForegroundColor Yellow
        }
    }

    if (Test-Path (Join-Path $env:WINDIR "System32\Kinect20.dll")) {
        Write-Host "Runtime installed." -ForegroundColor Green
    } else {
        throw "Setup finished but Kinect20.dll is not in System32: the runtime did not install."
    }
}

# --- 2. virtual camera --------------------------------------------------

Write-Step "OBS Virtual Camera"

if (Test-Path "C:\Program Files\obs-studio") {
    Write-Host "OBS Studio already present, skipping." -ForegroundColor Green
} else {
    Write-Host "Installing OBS Studio with winget..."
    # winget writes progress and warnings to stderr: in Windows PowerShell that
    # alone fails the command under ErrorActionPreference = Stop, so judge it
    # only by the exit code.
    $previous = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try {
        winget install --id $ObsWingetId --silent --accept-package-agreements --accept-source-agreements
        $wingetExit = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $previous
    }
    if ($wingetExit -ne 0) {
        Write-Host "winget returned $wingetExit." -ForegroundColor Yellow
        Write-Host "Install OBS Studio manually from https://obsproject.com and rerun this script." -ForegroundColor Yellow
    }
}

$clsid = "HKLM:\SOFTWARE\Classes\CLSID\{A3FCE0F5-3493-419F-958A-ABA1250EC20B}"
if (-not (Test-Path $clsid)) {
    Write-Host ""
    Write-Host "The virtual camera is not registered yet." -ForegroundColor Yellow
    Write-Host "Open OBS Studio once and close it: the first launch registers the driver." -ForegroundColor Yellow
}

# --- 3. KinectCam -------------------------------------------------------

Write-Step "KinectCam"

$source = Join-Path $PSScriptRoot "dist\KinectCam.exe"
if (-not (Test-Path $source)) {
    throw "KinectCam.exe not found. Run build.ps1 first."
}

$target = Join-Path $env:LOCALAPPDATA "Programs\KinectCam"
New-Item -ItemType Directory -Force $target | Out-Null
Copy-Item $source (Join-Path $target "KinectCam.exe") -Force
Write-Host "Copied to $target" -ForegroundColor Green

$startMenu = Join-Path $env:APPDATA "Microsoft\Windows\Start Menu\Programs\KinectCam.lnk"
$shell = New-Object -ComObject WScript.Shell
$shortcut = $shell.CreateShortcut($startMenu)
$shortcut.TargetPath = Join-Path $target "KinectCam.exe"
$shortcut.WorkingDirectory = $target
$shortcut.Description = "Use the Kinect v2 as a webcam"
$shortcut.Save()
Write-Host "Start menu shortcut created." -ForegroundColor Green

# --- wrap up ------------------------------------------------------------

Write-Host ""
Write-Host "=== Done ===" -ForegroundColor Green
Write-Host ""
Write-Host "Next:"
Write-Host "  1. Plug the Kinect back in: powered adapter, cable into a direct USB 3.0 port"
Write-Host "  2. Wait for Windows to finish installing the driver (a couple of minutes the first time)"
Write-Host "  3. Launch KinectCam from the Start menu and press 'Start virtual camera'"
Write-Host "  4. In Teams, Zoom, Meet or Discord pick the 'OBS Virtual Camera' device"
Write-Host ""
if ($LogPath) { try { Stop-Transcript | Out-Null } catch { } }
if (-not $Yes) { Read-Host "Press Enter to close" }
