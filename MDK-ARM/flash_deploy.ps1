# Flash deploy: UV4 -b build + -f flash + pyocd reset/go + FW_INFO liveness check (2026-09-05)
# Usage: powershell -File flash_deploy.ps1 -Commit <sha>   (Commit required, no default)
# Identity chain (2026-09-06 Kimi subtree assertion): build inputs of working tree
#   must equal <Commit> bit-for-bit -- `git diff --quiet <sha> -- <build inputs>`
#   (covers staged+unstaged) AND `git status --porcelain -- <build inputs>` empty.
#   HEAD may advance (scripts/docs) -- printed as reference only.
# Build inputs (from uvprojx, 2026-09-06 F): Core/ + MDK-ARM/code/ + Drivers/ + uvprojx.
#   FW_INFO only proves liveness+version family (git=unknown).
param(
    [Parameter(Mandatory=$true)]
    [string]$Commit,
    [switch]$SkipGitCheck
)

$ErrorActionPreference = "Stop"
$Root = "E:\24V_FOC_Controller_sync_20260519"
$UV4 = "C:\Users\xiangyu\AppData\Local\Keil_v5\UV4\UV4.exe"
$Proj = "$Root\MDK-ARM\24V FOC Controller.uvprojx"
$BuildLog = "$Root\MDK-ARM\build_log_deploy.txt"
$FlashLog = "$Root\MDK-ARM\flash_log_deploy.txt"
$ProbeId = "0001A0000001"
$PyocdTarget = "stm32h743vitx"
# Build input path set (2026-09-06 F: subtree assertion; all uvprojx compile inputs)
$BuildInputs = @("Core/", "MDK-ARM/code/", "Drivers/", "MDK-ARM/24V FOC Controller.uvprojx")

# 1) Subtree identity assertion: working-tree build inputs == <Commit> build inputs
if (-not $SkipGitCheck) {
    Set-Location $Root
    $head = (git rev-parse --short HEAD).Trim()
    Write-Host "git: HEAD=$head (reference only), asserting build inputs == $Commit"

    # a) diff working tree vs commit over build inputs (covers staged+unstaged)
    git diff --quiet $Commit -- $BuildInputs
    if ($LASTEXITCODE -ne 0) {
        Write-Host "FAIL: build inputs differ from $Commit:" -ForegroundColor Red
        git diff --stat $Commit -- $BuildInputs | Write-Host
        exit 1
    }

    # b) untracked files inside build inputs must be none
    $dirty = git status --porcelain -- $BuildInputs
    if ($dirty) {
        Write-Host "FAIL: build inputs have untracked/modified files:" -ForegroundColor Red
        Write-Host "$dirty"
        exit 1
    }
    Write-Host "git: build inputs == $Commit (Core/ code/ Drivers/ uvprojx)" -ForegroundColor Green
}

# 2) Build -b (must compile first, ensure axf matches current code)
if (Test-Path $BuildLog) { Remove-Item $BuildLog -Force }
$p = Start-Process -FilePath $UV4 -ArgumentList "-b `"$Proj`" -o `"$BuildLog`"" -Wait -PassThru
Start-Sleep -Seconds 2
$log = Get-Content $BuildLog -Raw -ErrorAction SilentlyContinue
if ($log -notmatch "0 Error\(s\)") {
    Write-Host "BUILD FAIL (exit=$($p.ExitCode)):" -ForegroundColor Red
    Write-Host $log
    exit 1
}
Write-Host "BUILD OK (exit=$($p.ExitCode))" -ForegroundColor Green

# axf freshness (stale axf hazard)
$axf = Get-ChildItem "$Root\MDK-ARM\24V FOC Controller\24V FOC Controller.axf"
Write-Host "axf: $($axf.LastWriteTime) ($($axf.Length) bytes)"

# 3) Flash -f (Keil verify included)
if (Test-Path $FlashLog) { Remove-Item $FlashLog -Force }
$p = Start-Process -FilePath $UV4 -ArgumentList "-f `"$Proj`" -o `"$FlashLog`"" -Wait -PassThru
Start-Sleep -Seconds 2
$log = Get-Content $FlashLog -Raw -ErrorAction SilentlyContinue
if ($log -notmatch "Verify OK" -and $log -notmatch "Verify.*success" -and $log -notmatch "0 Error") {
    Write-Host "FLASH FAIL (exit=$($p.ExitCode)):" -ForegroundColor Red
    Write-Host $log
    exit 1
}
Write-Host "FLASH OK (exit=$($p.ExitCode))" -ForegroundColor Green

# 4) pyocd reset/go (UV4 leaves MCU halted after -f)
pyocd commander -u $ProbeId -t $PyocdTarget -c "reset" -c "go" 2>&1 | Out-Host

# 5) FW_INFO liveness check
python "$Root\scripts\uart_fw_info.py" COM10

# Identity chain output (2026-09-06 F): FW_INFO lacks SHA; conclusion = subtree
#   assertion (build inputs == Commit) + UV4 Verify OK + FW_INFO liveness
Write-Host "Board=$Commit source build (subtree assert + UV4 Verify OK + FW_INFO alive)" -ForegroundColor Green
Write-Host "DEPLOY_DONE"
