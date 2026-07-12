# FOC Device Bridge — PyInstaller Packaging
# Builds a standalone .exe for the FOC Device Bridge (system tray app).
#
# Prerequisites: pip install pyinstaller
# Usage: .\build_bridge.ps1

$ErrorActionPreference = "Stop"

$SCRIPT_DIR = Split-Path -Parent $MyInvocation.MyCommand.Path
$PROJECT_DIR = Split-Path -Parent $SCRIPT_DIR
$HOSTCOMPUTER_DIR = Join-Path $PROJECT_DIR "HostComputer"

Write-Host "=== Building FOC Device Bridge ===" -ForegroundColor Cyan

# Verify HostComputer package is available
if (!(Test-Path (Join-Path $HOSTCOMPUTER_DIR "__init__.py"))) {
    Write-Host "ERROR: HostComputer package not found at $HOSTCOMPUTER_DIR" -ForegroundColor Red
    exit 1
}

pyinstaller `
    --onefile `
    --windowed `
    --name FOC_Device_Bridge `
    --specpath "$(Join-Path $PROJECT_DIR build)" `
    --add-data "${HOSTCOMPUTER_DIR};HostComputer" `
    --hidden-import PySide6.QtNetwork `
    --hidden-import serial `
    --hidden-import serial.tools.list_ports `
    "$(Join-Path $SCRIPT_DIR bridge_launcher.py)"

Copy-Item -LiteralPath (Join-Path $PROJECT_DIR "LICENSE") -Destination (Join-Path $PROJECT_DIR "dist\LICENSE.txt") -Force
Copy-Item -LiteralPath (Join-Path $PROJECT_DIR "THIRD_PARTY_NOTICES.md") -Destination (Join-Path $PROJECT_DIR "dist\THIRD_PARTY_NOTICES.md") -Force

Write-Host "`n=== Build Complete ===" -ForegroundColor Green
Write-Host "Output: dist\FOC_Device_Bridge.exe"
