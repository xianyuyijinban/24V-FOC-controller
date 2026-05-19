$ErrorActionPreference = "Stop"

$PROJECT_ROOT = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$BUILD_DIR = Join-Path $PSScriptRoot "build"
$TARGET = "uart_smoke_test"
# Output artifact: uart_smoke_test.elf
$ELF_PATH = Join-Path $BUILD_DIR "$TARGET.elf"
$HEX_PATH = Join-Path $BUILD_DIR "$TARGET.hex"
$BIN_PATH = Join-Path $BUILD_DIR "$TARGET.bin"

$GCC_PATH = "C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin"
$CC = Join-Path $GCC_PATH "arm-none-eabi-gcc.exe"
$OBJCOPY = Join-Path $GCC_PATH "arm-none-eabi-objcopy.exe"
$SIZE = Join-Path $GCC_PATH "arm-none-eabi-size.exe"

function Invoke-AndCheck([string]$exe, [string[]]$toolArgs)
{
    $previousErrorActionPreference = $ErrorActionPreference
    $previousNativeErrorPreference = $PSNativeCommandUseErrorActionPreference

    try {
        $ErrorActionPreference = "Continue"
        $PSNativeCommandUseErrorActionPreference = $false
        $output = & $exe @toolArgs 2>&1
        $ok = ($LASTEXITCODE -eq 0)
    } finally {
        $ErrorActionPreference = $previousErrorActionPreference
        $PSNativeCommandUseErrorActionPreference = $previousNativeErrorPreference
    }

    return @{
        Ok = $ok
        Output = $output
    }
}

function Get-ObjectPath([string]$src)
{
    $name = ($src -replace "[:\\/\s]", "_") -replace "\.(c|s)$", ""
    return Join-Path $BUILD_DIR "$name.o"
}

$INCLUDES = @(
    "-I$PSScriptRoot",
    "-I$PROJECT_ROOT\Drivers\STM32H7xx_HAL_Driver\Inc",
    "-I$PROJECT_ROOT\Drivers\STM32H7xx_HAL_Driver\Inc\Legacy",
    "-I$PROJECT_ROOT\Drivers\CMSIS\Device\ST\STM32H7xx\Include",
    "-I$PROJECT_ROOT\Drivers\CMSIS\Include"
)

$DEFINES = @(
    "-DUSE_HAL_DRIVER",
    "-DSTM32H743xx"
)

$MCU_FLAGS = @("-mcpu=cortex-m7", "-mfpu=fpv5-d16", "-mfloat-abi=hard", "-mthumb")

$CFLAGS = $MCU_FLAGS + $DEFINES + $INCLUDES + @(
    "-O2",
    "-ffunction-sections",
    "-fdata-sections",
    "-Wall",
    "-Wextra",
    "-Wno-unused-parameter",
    "-std=c11",
    "-c"
)

$ASFLAGS = $MCU_FLAGS + $DEFINES + $INCLUDES + @(
    "-x",
    "assembler-with-cpp",
    "-c"
)

$LFLAGS = $MCU_FLAGS + @(
    "-specs=nano.specs",
    "-specs=nosys.specs",
    "-T$PROJECT_ROOT\STM32H743VITX_FLASH.ld",
    "-Wl,-Map=$BUILD_DIR\$TARGET.map,--cref",
    "-Wl,--gc-sections",
    "-lm",
    "-lc"
)

$C_SOURCES = @(
    "$PSScriptRoot\main.c",
    "$PROJECT_ROOT\Core\Src\system_stm32h7xx.c",
    "$PROJECT_ROOT\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_hal.c",
    "$PROJECT_ROOT\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_hal_cortex.c",
    "$PROJECT_ROOT\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_hal_dma.c",
    "$PROJECT_ROOT\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_hal_dma_ex.c",
    "$PROJECT_ROOT\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_hal_flash.c",
    "$PROJECT_ROOT\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_hal_flash_ex.c",
    "$PROJECT_ROOT\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_hal_gpio.c",
    "$PROJECT_ROOT\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_hal_pwr.c",
    "$PROJECT_ROOT\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_hal_pwr_ex.c",
    "$PROJECT_ROOT\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_hal_rcc.c",
    "$PROJECT_ROOT\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_hal_rcc_ex.c",
    "$PROJECT_ROOT\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_hal_uart.c",
    "$PROJECT_ROOT\Drivers\STM32H7xx_HAL_Driver\Src\stm32h7xx_hal_uart_ex.c"
)

$ASM_SOURCES = @(
    "$PROJECT_ROOT\Drivers\CMSIS\Device\ST\STM32H7xx\Source\Templates\gcc\startup_stm32h743xx.s"
)

foreach ($tool in @($CC, $OBJCOPY, $SIZE)) {
    if (!(Test-Path $tool)) {
        Write-Host "Missing tool: $tool" -ForegroundColor Red
        exit 1
    }
}

New-Item -ItemType Directory -Force -Path $BUILD_DIR | Out-Null

$objectFiles = New-Object System.Collections.Generic.List[string]
$compileErrors = New-Object System.Collections.Generic.List[string]

function Compile-SourceGroup([string]$title, [string[]]$sources, [string]$tag, [string[]]$flags)
{
    Write-Host "`n=== $title ===" -ForegroundColor Cyan
    foreach ($src in $sources) {
        $obj = Get-ObjectPath $src
        $script:objectFiles.Add($obj)

        Write-Host "$tag $src" -NoNewline
        $result = Invoke-AndCheck $CC ($flags + @($src, "-o", $obj))
        if ($result.Ok) {
            Write-Host " [OK]" -ForegroundColor Green
        } else {
            Write-Host " [FAIL]" -ForegroundColor Red
            $script:compileErrors.Add("$src`n$($result.Output | Out-String)")
        }
    }
}

Compile-SourceGroup "Compiling Standalone UART Smoke Test C Sources" $C_SOURCES "CC" $CFLAGS
Compile-SourceGroup "Compiling Standalone UART Smoke Test ASM Sources" $ASM_SOURCES "AS" $ASFLAGS

if ($compileErrors.Count -gt 0) {
    Write-Host "`n=== COMPILATION ERRORS ===" -ForegroundColor Red
    foreach ($err in $compileErrors) {
        Write-Host $err -ForegroundColor Red
    }
    exit 1
}

Write-Host "`n=== Linking ===" -ForegroundColor Cyan
$linkResult = Invoke-AndCheck $CC ($objectFiles + $LFLAGS + @("-o", $ELF_PATH))
if ((!$linkResult.Ok) -or !(Test-Path $ELF_PATH)) {
    Write-Host ($linkResult.Output | Out-String) -ForegroundColor Red
    exit 1
}
Write-Host "LD $TARGET.elf [OK]" -ForegroundColor Green

& $OBJCOPY -O ihex $ELF_PATH $HEX_PATH
& $OBJCOPY -O binary $ELF_PATH $BIN_PATH
& $SIZE $ELF_PATH

Write-Host "`nCreated:"
Write-Host "  $ELF_PATH"
Write-Host "  $HEX_PATH"
Write-Host "  $BIN_PATH"
