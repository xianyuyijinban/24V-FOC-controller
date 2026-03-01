# 24V FOC Controller Build Script for ARM GCC
# Target: STM32H743VIT6

$ErrorActionPreference = "Stop"

# Toolchain
$GCC_PATH = "C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin"
$CC = "$GCC_PATH\arm-none-eabi-gcc.exe"
$LD = "$GCC_PATH\arm-none-eabi-gcc.exe"
$OBJCOPY = "$GCC_PATH\arm-none-eabi-objcopy.exe"
$SIZE = "$GCC_PATH\arm-none-eabi-size.exe"

# Target
$TARGET = "24V_FOC_Controller"
$BUILD_DIR = "build/gcc"
$ELF_PATH = "$BUILD_DIR/$TARGET.elf"
$HEX_PATH = "$BUILD_DIR/$TARGET.hex"
$BIN_PATH = "$BUILD_DIR/$TARGET.bin"

function Get-ObjectPath([string]$src)
{
    $name = ($src -replace "[:\\/\s]", "_") -replace "\.(c|s)$", ""
    return "$BUILD_DIR/$name.o"
}

function Invoke-AndCheck([string]$exe, [string[]]$toolArgs)
{
    $output = & $exe @toolArgs 2>&1
    $ok = ($LASTEXITCODE -eq 0)
    return @{
        Ok = $ok
        Output = $output
    }
}

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

# Assembler flags (use GCC driver + GNU startup .s)
$ASFLAGS = $MCU_FLAGS + $DEFINES + $INCLUDES + @(
    "-x",
    "assembler-with-cpp",
    "-c"
)

# Linker flags
$LFLAGS = $MCU_FLAGS + @(
    "-specs=nano.specs",
    "-specs=nosys.specs",
    "-TSTM32H743VITX_FLASH.ld",
    "-Wl,-Map=$BUILD_DIR/$TARGET.map,--cref",
    "-Wl,--gc-sections",
    "-lm",
    "-lc"
)

# Source files
$HAL_SOURCES = @(
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c",
    "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c"
)

$CORE_SOURCES = @(
    "Core/Src/main.c",
    "Core/Src/gpio.c",
    "Core/Src/adc.c",
    "Core/Src/dma.c",
    "Core/Src/fdcan.c",
    "Core/Src/i2c.c",
    "Core/Src/memorymap.c",
    "Core/Src/spi.c",
    "Core/Src/tim.c",
    "Core/Src/usart.c",
    "Core/Src/stm32h7xx_it.c",
    "Core/Src/stm32h7xx_hal_msp.c",
    "Core/Src/system_stm32h7xx.c"
)

$FOC_SOURCES = @(
    "MDK-ARM/code/adc_sampling.c",
    "MDK-ARM/code/drv8350s.c",
    "MDK-ARM/code/foc_app.c",
    "MDK-ARM/code/foc_core.c",
    "MDK-ARM/code/motor_identify.c",
    "MDK-ARM/code/param_storage.c",
    "MDK-ARM/code/tle5012.c",
    "MDK-ARM/code/uart_upload.c"
)

$ASM_SOURCES = @(
    "Drivers/CMSIS/Device/ST/STM32H7xx/Source/Templates/gcc/startup_stm32h743xx.s"
)

# Pre-check toolchain
foreach ($tool in @($CC, $OBJCOPY, $SIZE)) {
    if (!(Test-Path $tool)) {
        Write-Host "Missing tool: $tool" -ForegroundColor Red
        exit 1
    }
}

# Create build directory
New-Item -ItemType Directory -Force -Path $BUILD_DIR | Out-Null

$objectFiles = New-Object System.Collections.Generic.List[string]
$compileErrors = New-Object System.Collections.Generic.List[string]

function Compile-SourceGroup([string]$title, [string[]]$sources, [string]$tag, [string[]]$flags)
{
    Write-Host "`n=== $title ===" -ForegroundColor Cyan
    foreach ($src in $sources) {
        if (!(Test-Path $src)) {
            Write-Host "$tag $src [SKIP]" -ForegroundColor Yellow
            continue
        }

        $obj = Get-ObjectPath $src
        $script:objectFiles.Add($obj)
        Write-Host "$tag $src" -NoNewline

        $result = Invoke-AndCheck $CC ($flags + @($src, "-o", $obj))
        if ($result.Ok) {
            Write-Host " [OK]" -ForegroundColor Green
        } else {
            Write-Host " [FAIL]" -ForegroundColor Red
            $message = "$src`n$($result.Output | Out-String)"
            $script:compileErrors.Add($message)
        }
    }
}

Compile-SourceGroup "Compiling HAL Library" $HAL_SOURCES "CC" $CFLAGS
Compile-SourceGroup "Compiling Core Sources" $CORE_SOURCES "CC" $CFLAGS
Compile-SourceGroup "Compiling FOC Sources" $FOC_SOURCES "CC" $CFLAGS
Compile-SourceGroup "Compiling ASM Sources" $ASM_SOURCES "AS" $ASFLAGS

# Check for compile errors
if ($compileErrors.Count -gt 0) {
    Write-Host "`n=== COMPILATION ERRORS ===" -ForegroundColor Red
    foreach ($err in $compileErrors) {
        Write-Host $err -ForegroundColor Red
    }
    exit 1
}

# Link
Write-Host "`n=== Linking ===" -ForegroundColor Cyan
Write-Host "LD $TARGET.elf" -NoNewline
$linkResult = Invoke-AndCheck $LD ($objectFiles + $LFLAGS + @("-o", $ELF_PATH))
if ((!$linkResult.Ok) -or !(Test-Path $ELF_PATH)) {
    Write-Host " [FAIL]" -ForegroundColor Red
    Write-Host ($linkResult.Output | Out-String) -ForegroundColor Red
    exit 1
}
Write-Host " [OK]" -ForegroundColor Green

# Generate hex and bin
Write-Host "`n=== Generating Output Files ===" -ForegroundColor Cyan
$hexResult = Invoke-AndCheck $OBJCOPY @("-O", "ihex", $ELF_PATH, $HEX_PATH)
if (!$hexResult.Ok) {
    Write-Host "Failed to generate HEX" -ForegroundColor Red
    Write-Host ($hexResult.Output | Out-String) -ForegroundColor Red
    exit 1
}
Write-Host "Created $TARGET.hex"

$binResult = Invoke-AndCheck $OBJCOPY @("-O", "binary", $ELF_PATH, $BIN_PATH)
if (!$binResult.Ok) {
    Write-Host "Failed to generate BIN" -ForegroundColor Red
    Write-Host ($binResult.Output | Out-String) -ForegroundColor Red
    exit 1
}
Write-Host "Created $TARGET.bin"

# Show size
Write-Host "`n=== Build Summary ===" -ForegroundColor Green
$sizeResult = Invoke-AndCheck $SIZE @($ELF_PATH)
if (!$sizeResult.Ok) {
    Write-Host "Failed to read ELF size" -ForegroundColor Red
    Write-Host ($sizeResult.Output | Out-String) -ForegroundColor Red
    exit 1
}
Write-Host ($sizeResult.Output | Out-String)

Write-Host "Build completed successfully!" -ForegroundColor Green
