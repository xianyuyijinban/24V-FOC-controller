# Build the local HostComputer PyQt GUI into a Windows one-folder app.

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$RepoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$PythonExe = "python"
$RequirementsPath = Join-Path $RepoRoot "HostComputer\requirements.txt"
$SpecPath = Join-Path $RepoRoot "HostComputer\host_gui_app.spec"
$PyInstallerWorkDir = Join-Path $RepoRoot "build\pyinstaller"
$DistPath = Join-Path $RepoRoot "dist"
$AppName = "24V_FOC_Host"
$AppDir = Join-Path $DistPath $AppName
$ExePath = Join-Path $AppDir "$AppName.exe"

function Invoke-AndCheck([string]$exe, [string[]]$toolArgs)
{
    $previousErrorActionPreference = $ErrorActionPreference
    $nativeErrorPreferenceVar = Get-Variable -Name PSNativeCommandUseErrorActionPreference -ErrorAction SilentlyContinue
    $hasNativeErrorPreference = ($null -ne $nativeErrorPreferenceVar)
    if ($hasNativeErrorPreference) {
        $previousNativeErrorPreference = $nativeErrorPreferenceVar.Value
    }

    try {
        $ErrorActionPreference = "Continue"
        if ($hasNativeErrorPreference) {
            $PSNativeCommandUseErrorActionPreference = $false
        }
        $output = & $exe @toolArgs 2>&1
        $ok = ($LASTEXITCODE -eq 0)
    } finally {
        $ErrorActionPreference = $previousErrorActionPreference
        if ($hasNativeErrorPreference) {
            $PSNativeCommandUseErrorActionPreference = $previousNativeErrorPreference
        }
    }

    return @{
        Ok = $ok
        Output = $output
    }
}

function Remove-WorkspaceTree([string]$targetPath)
{
    if (!(Test-Path $targetPath)) {
        return
    }

    $resolvedRepo = [System.IO.Path]::GetFullPath($RepoRoot)
    $resolvedTarget = [System.IO.Path]::GetFullPath($targetPath)
    if (!$resolvedTarget.StartsWith($resolvedRepo, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to remove path outside workspace: $resolvedTarget"
    }

    Remove-Item -LiteralPath $resolvedTarget -Recurse -Force
}

Push-Location $RepoRoot
try {
    Write-Host "=== Host GUI Packaging ===" -ForegroundColor Cyan
    Write-Host "Python: $( & $PythonExe --version )"

    if (!(Test-Path $RequirementsPath)) {
        throw "Missing requirements file: $RequirementsPath"
    }
    if (!(Test-Path $SpecPath)) {
        throw "Missing PyInstaller spec: $SpecPath"
    }

    Write-Host "`n=== Installing packaging dependencies ===" -ForegroundColor Cyan
    $installResult = Invoke-AndCheck $PythonExe @(
        "-m", "pip", "install",
        "-r", "HostComputer/requirements.txt",
        "PyInstaller"
    )
    if (!$installResult.Ok) {
        Write-Host ($installResult.Output | Out-String) -ForegroundColor Red
        throw "Failed to install HostComputer runtime/build dependencies."
    }
    Write-Host ($installResult.Output | Out-String)

    Write-Host "`n=== Cleaning previous packaging output ===" -ForegroundColor Cyan
    Remove-WorkspaceTree $PyInstallerWorkDir
    Remove-WorkspaceTree $AppDir
    New-Item -ItemType Directory -Force -Path $DistPath | Out-Null

    Write-Host "`n=== Building one-folder app with PyInstaller ===" -ForegroundColor Cyan
    $buildResult = Invoke-AndCheck $PythonExe @(
        "-m", "PyInstaller",
        "--noconfirm",
        "--clean",
        "--distpath", "dist",
        "--workpath", "build/pyinstaller",
        "HostComputer/host_gui_app.spec"
    )
    if (!$buildResult.Ok) {
        Write-Host ($buildResult.Output | Out-String) -ForegroundColor Red
        throw "PyInstaller build failed."
    }
    Write-Host ($buildResult.Output | Out-String)

    if (!(Test-Path $ExePath)) {
        throw "Build completed without expected executable: $ExePath"
    }

    Write-Host "`n=== Packaging Summary ===" -ForegroundColor Green
    Write-Host "App directory: $AppDir"
    Write-Host "Executable:   $ExePath"
    Write-Host "Packaging completed successfully." -ForegroundColor Green
} finally {
    Pop-Location
}
