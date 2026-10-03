$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$VenvDir = Join-Path $ScriptDir ".venv"

if (-not (Test-Path $VenvDir)) {
    $PlatformIoPython = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\python.exe"
    if (Get-Command py -ErrorAction SilentlyContinue) {
        & py -3 -m venv $VenvDir
    }
    elseif (Get-Command python -ErrorAction SilentlyContinue) {
        & python -m venv $VenvDir
    }
    elseif (Test-Path $PlatformIoPython) {
        & $PlatformIoPython -m venv $VenvDir
    }
    else {
        throw "Python 3 was not found. Install Python 3.11 or newer, then rerun this script."
    }
}

$Python = Join-Path $VenvDir "Scripts\python.exe"
& $Python -m pip install --upgrade pip
& $Python -m pip install -r (Join-Path $ScriptDir "requirements.txt")

Write-Host "Setup complete. Run .\run_gui.ps1 to start the debug console."
