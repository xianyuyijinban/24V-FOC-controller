# Quick build test for FOC code (without startup file)
# Tests compilation of C sources only

$ErrorActionPreference = "Stop"

# Toolchain
$GCC_PATH = "C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin"
$CC = "$GCC_PATH\arm-none-eabi-gcc.exe"

# Build directory
$BUILD_DIR = "build/gcc"

# Include paths
$INCLUDES = @(
    "-ICore/Inc",
    "-IDrivers/STM32H7xx_HAL_Driver/Inc",
    "-IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy",
    "-IDrivers/CMSIS/Device/ST/STM32H7xx/Include",
    "-IDrivers/CMSIS/Include",
    "-I.cmsis/include",
    "-IMDK-ARM/code",
    "-IMDK-ARM/RTE/_24V_FOC_Controller"
)

# Defines
$DEFINES = @(
    "-DUSE_HAL_DRIVER",
    "-DSTM32H743xx",
    "-DARM_MATH_CM7"
)

# MCU flags
$MCU_FLAGS = @("-mcpu=cortex-m7", "-mfpu=fpv5-d16", "-mfloat-abi=hard", "-mthumb")

# Compiler flags
$CFLAGS = $MCU_FLAGS + $DEFINES + $INCLUDES + @(
    "-O2",
    "-ffunction-sections",
    "-fdata-sections",
    "-Wall",
    "-Wextra",
    "-Wno-unused-parameter",
    "-fomit-frame-pointer",
    "-std=c11",
    "-c"
)

# Source files to test
$SOURCES = @(
    "MDK-ARM/code/foc_core.c",
    "MDK-ARM/code/foc_app.c",
    "MDK-ARM/code/motor_identify.c",
    "MDK-ARM/code/param_storage.c",
    "MDK-ARM/code/adc_sampling.c",
    "MDK-ARM/code/tle5012.c",
    "MDK-ARM/code/drv8350s.c",
    "MDK-ARM/code/uart_upload.c"
)

# Create build directory
New-Item -ItemType Directory -Force -Path $BUILD_DIR | Out-Null

$success = 0
$failed = 0

Write-Host "`n=== Testing FOC Code Compilation ===" -ForegroundColor Cyan
Write-Host "Compiler: $( & $CC --version | Select-Object -First 1 )`n"

foreach ($src in $SOURCES) {
    $objName = [System.IO.Path]::GetFileNameWithoutExtension($src)
    Write-Host "Compiling $objName.c ... " -NoNewline
    
    try {
        $output = & $CC $CFLAGS $src -o "$BUILD_DIR/$objName.o" 2>&1
        if ($LASTEXITCODE -eq 0) {
            Write-Host "OK" -ForegroundColor Green
            $success++
        } else {
            Write-Host "FAILED" -ForegroundColor Red
            Write-Host "  Error: $output" -ForegroundColor DarkRed
            $failed++
        }
    } catch {
        Write-Host "FAILED" -ForegroundColor Red
        Write-Host "  Error: $_" -ForegroundColor DarkRed
        $failed++
    }
}

Write-Host "`n=== Summary ===" -ForegroundColor Cyan
Write-Host "Successful: $success" -ForegroundColor Green
Write-Host "Failed: $failed" -ForegroundColor Red

if ($failed -eq 0) {
    Write-Host "`n✓ All FOC code compiled successfully!" -ForegroundColor Green
    exit 0
} else {
    Write-Host "`n✗ Some files failed to compile" -ForegroundColor Red
    exit 1
}
