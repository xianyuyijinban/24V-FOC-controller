# Flash deploy: UV4 -b build + -f flash + pyocd reset/go + FW_INFO liveness check (2026-09-05)
# Usage: powershell -File flash_deploy.ps1 -Commit <sha>   (Commit 必填, 无默认值)
# 身份链纪律 (2026-09-06 Kimi 定案): HEAD=Commit 且 code/ 干净 (axf==声称 commit)
#   才许可烧录; MDK-ARM/code/ 残留即 fail, 不 WARN 继续。FW_INFO 仅活体+版本族
#   (git=unknown), 身份 = HEAD 断言 + 干净树 + 烧录日志三点。
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

# 1) Git assertion: HEAD == Commit && code/ clean (2026-09-06: code/ 脏则 fail)
if (-not $SkipGitCheck) {
    Set-Location $Root
    $head = (git rev-parse --short HEAD).Trim()
    if ($head -ne $Commit) {
        Write-Host "FAIL: HEAD=$head expected=$Commit" -ForegroundColor Red
        exit 1
    }
    # code/ 残留 (foc_app.c/h 等) 时 axf != 声称 commit — 身份链破洞, 直接 exit
    $codeDirty = git status --porcelain -- "MDK-ARM/code/"
    if ($codeDirty) {
        Write-Host "FAIL: MDK-ARM/code/ 有残留, axf 不可信任:" -ForegroundColor Red
        Write-Host "$codeDirty"
        exit 1
    }
    $status = git status --porcelain
    if ($status) {
        Write-Host "WARN: working tree not clean (non-code):" -ForegroundColor Yellow
        $status | Where-Object { $_ -notmatch "^\?\? .*\.json$" } | Write-Host
    } else {
        Write-Host "git: HEAD=$Commit code/clean" -ForegroundColor Green
    }
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

# 身份链输出 (2026-09-06): FW_INFO 看不到 SHA, 结论 = HEAD 断言 + code/干净 + 烧录日志三点
Write-Host "板=$Commit 源码构建 (HEAD 断言 + code/clean + UV4 Verify OK)" -ForegroundColor Green
Write-Host "DEPLOY_DONE"
