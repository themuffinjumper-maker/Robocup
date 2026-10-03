$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$Python = Join-Path $ScriptDir ".venv\Scripts\python.exe"

if (-not (Test-Path $Python)) {
    throw "GUI environment is missing. Run .\setup.ps1 first."
}

& $Python (Join-Path $ScriptDir "DebugGUI.py")
