# Builds KinectCam.exe (single file, no console window).
# Requires Python 3.10+ on PATH. Administrator rights are not needed.

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

# Python and pip write warnings and progress to stderr. In Windows PowerShell
# that alone fails the command under ErrorActionPreference = Stop, so native
# commands go through here and are judged by their exit code instead.
function Invoke-Native {
    param([string]$Exe, [string[]]$Arguments, [string]$What)

    $previous = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try {
        & $Exe @Arguments
        $code = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $previous
    }
    if ($code -ne 0) { throw "$What failed (exit code $code)." }
}

$venv = Join-Path $PSScriptRoot ".venv"
$python = Join-Path $venv "Scripts\python.exe"

if (-not (Test-Path $python)) {
    Write-Host "Creating the virtual environment..." -ForegroundColor Cyan
    Invoke-Native "py" @("-3", "-m", "venv", $venv) "Creating the virtual environment"
}

Write-Host "Installing dependencies..." -ForegroundColor Cyan
Invoke-Native $python @("-m", "pip", "install", "--quiet", "--upgrade", "pip") "Upgrading pip"
Invoke-Native $python @("-m", "pip", "install", "--quiet", "-r", "requirements.txt", "pyinstaller") "Installing dependencies"

Write-Host "Running the test suite..." -ForegroundColor Cyan
Invoke-Native $python @("tests\test_offline.py") "Test suite"
Invoke-Native $python @("tests\test_scanning.py") "Scanning test suite"
Invoke-Native $python @("tests\test_gui.py") "Window test suite"

Write-Host "Building the executable..." -ForegroundColor Cyan
Invoke-Native $python @(
    "-m", "PyInstaller",
    "--noconfirm",
    "--onefile",
    "--windowed",
    "--name", "KinectCam",
    "--collect-all", "pyvirtualcam",
    "--exclude-module", "matplotlib",
    "--exclude-module", "pytest",
    "KinectCam.py"
) "PyInstaller build"

$exe = Join-Path $PSScriptRoot "dist\KinectCam.exe"
if (-not (Test-Path $exe)) { throw "Build failed: KinectCam.exe was not produced." }

$sizeMb = [math]::Round((Get-Item $exe).Length / 1MB, 1)
Write-Host ""
Write-Host "Done: $exe ($sizeMb MB)" -ForegroundColor Green

