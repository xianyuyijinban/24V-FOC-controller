# 24V FOC Controller - Makefile for ARM GCC
# Target: STM32H743VIT6

# Toolchain
PREFIX = arm-none-eabi
CC = $(PREFIX)-gcc
CXX = $(PREFIX)-g++
AS = $(PREFIX)-gcc
LD = $(PREFIX)-gcc
OBJCOPY = $(PREFIX)-objcopy
OBJDUMP = $(PREFIX)-objdump
SIZE = $(PREFIX)-size
AR = $(PREFIX)-ar

# Target name
TARGET = 24V_FOC_Controller

# Build directory
BUILD_DIR = build/gcc

# Source directories
CORE_DIR = Core
DRIVERS_DIR = Drivers
CODE_DIR = MDK-ARM/code
CMSIS_DIR = .cmsis

# Include paths
INCLUDES = \
-I$(CORE_DIR)/Inc \
-I$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Inc \
-I$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Inc/Legacy \
-I$(DRIVERS_DIR)/CMSIS/Device/ST/STM32H7xx/Include \
-I$(DRIVERS_DIR)/CMSIS/Include \
-I$(CMSIS_DIR)/include \
-I$(CODE_DIR) \
-IMDK-ARM/RTE/_24V_FOC_Controller

# Defines
DEFINES = \
-DUSE_HAL_DRIVER \
-DSTM32H743xx \
-DARM_MATH_CM7

# MCU flags
MCU_FLAGS = -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb

# Compiler flags
CFLAGS = $(MCU_FLAGS) $(DEFINES) $(INCLUDES) \
-O2 \
-ffunction-sections \
-fdata-sections \
-Wall \
-Wextra \
-Wno-unused-parameter \
-fomit-frame-pointer \
-ffast-math \
-std=c11 \
-MMD -MP

# C++ flags
CXXFLAGS = $(CFLAGS) -std=c++11 -fno-rtti -fno-exceptions

# Assembler flags
ASFLAGS = $(MCU_FLAGS) -Wall -fdata-sections -ffunction-sections

# Linker flags
LDFLAGS = $(MCU_FLAGS) -specs=nano.specs -specs=nosys.specs \
-TSTM32H743VITX_FLASH.ld \
-Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
-Wl,--gc-sections \
-lm -lc

# Source files
HAL_SOURCES = \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c \
$(DRIVERS_DIR)/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c

C_SOURCES = \
$(wildcard $(CORE_DIR)/Src/*.c) \
$(CODE_DIR)/adc_sampling.c \
$(CODE_DIR)/drv8350s.c \
$(CODE_DIR)/foc_app.c \
$(CODE_DIR)/foc_core.c \
$(CODE_DIR)/motor_identify.c \
$(CODE_DIR)/param_storage.c \
$(CODE_DIR)/tle5012.c \
$(CODE_DIR)/uart_upload.c \
$(HAL_SOURCES)

# ASM sources (GNU startup file)
ASM_SOURCES = \
Drivers/CMSIS/Device/ST/STM32H7xx/Source/Templates/gcc/startup_stm32h743xx.s

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# Default target
.PHONY: all clean info

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	@echo "Build complete!"
	$(SIZE) $(BUILD_DIR)/$(TARGET).elf

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile C files
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@echo "CC $<"
	@$(CC) $(CFLAGS) -c $< -o $@

# Compile ASM files
$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	@echo "AS $<"
	@$(AS) $(MCU_FLAGS) $(DEFINES) $(INCLUDES) -x assembler-with-cpp -c $< -o $@

# Link
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	@echo "LD $@"
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@

# Create hex file
$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

# Create bin file
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

# Clean
clean:
	rm -rf $(BUILD_DIR)

# Show info
info:
	@echo "C sources: $(C_SOURCES)"
	@echo "ASM sources: $(ASM_SOURCES)"
	@echo "Objects: $(OBJECTS)"

# Dependencies
-include $(wildcard $(BUILD_DIR)/*.d)
