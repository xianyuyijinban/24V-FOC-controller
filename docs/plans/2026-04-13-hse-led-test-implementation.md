# HSE LED Test Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build a standalone STM32H743 HSE LED bench-test firmware that blinks `PB8` or `PB9` to report HSE startup success or failure.

**Architecture:** Keep the FOC firmware untouched. Add a separate minimal bench project that reuses the repo's HAL/CMSIS baseline, initializes `PB8/PB9` before the clock switch, tries the board's `25 MHz HSE + PLL1` configuration, and enters one of two explicit LED loops.

**Tech Stack:** STM32H7 HAL, CMSIS startup/linker files, PowerShell build wrapper, pyOCD flashing.

---

### Task 1: Add source-contract coverage

**Files:**
- Modify: `test_build_system.py`

**Step 1: Write the failing test**

- Add a new test that asserts:
  - `BenchTests/HSE_LED_Test/main.c` exists;
  - `BenchTests/HSE_LED_Test/build.ps1` exists;
  - `main.c` contains `GPIO_PIN_8|GPIO_PIN_9`, `RCC_OSCILLATORTYPE_HSE`, `RCC_HSE_ON`, `RCC_PLLSOURCE_HSE`, and explicit success/failure LED loops.

**Step 2: Run test to verify it fails**

Run: `python -m pytest .\test_build_system.py -q -k hse_led_test`

Expected: FAIL because the standalone project files do not exist yet.

**Step 3: Commit**

- Skip commit until implementation is complete.

### Task 2: Create the standalone HSE LED test project

**Files:**
- Create: `BenchTests/HSE_LED_Test/main.c`
- Create: `BenchTests/HSE_LED_Test/main.h`
- Create: `BenchTests/HSE_LED_Test/stm32h7xx_hal_conf.h`

**Step 1: Write minimal implementation**

- `main.c`
  - call `HAL_Init()`;
  - enable `GPIOB`;
  - configure `PB8/PB9` as push-pull outputs;
  - call `SystemClock_Config()` that returns status;
  - on success, blink `PB8` slowly and keep `PB9` low;
  - on failure, blink `PB9` quickly and keep `PB8` low with a busy-wait loop.
- `main.h`
  - include HAL and declare helpers.
- `stm32h7xx_hal_conf.h`
  - keep the minimum modules required for RCC/GPIO/Cortex/PWR.

**Step 2: Reuse existing startup/linker baseline**

- Reference the repo's existing startup assembly and linker script from the build script rather than duplicating generated artifacts.

### Task 3: Add standalone build path

**Files:**
- Create: `BenchTests/HSE_LED_Test/build.ps1`

**Step 1: Write minimal build script**

- Compile:
  - standalone `main.c`
  - required HAL driver sources
  - existing `system_stm32h7xx.c`
  - existing startup assembly
- Link with the repository's existing linker script.
- Emit:
  - `BenchTests/HSE_LED_Test/build/hse_led_test.elf`
  - `.hex`
  - `.bin`

**Step 2: Keep the script explicit**

- Hard-code the exact source list needed for this test project.
- Avoid pulling in unrelated FOC modules.

### Task 4: Green the source-contract test

**Files:**
- Modify: `test_build_system.py`
- Create/modify: `BenchTests/HSE_LED_Test/*`

**Step 1: Run the targeted test**

Run: `python -m pytest .\test_build_system.py -q -k hse_led_test`

Expected: PASS

**Step 2: Run full source-contract suite**

Run: `python -m pytest .\test_build_system.py -q`

Expected: PASS

### Task 5: Build and flash the standalone firmware

**Files:**
- Create/modify: `BenchTests/HSE_LED_Test/build.ps1`

**Step 1: Build**

Run: `powershell -NoProfile -ExecutionPolicy Bypass -File .\BenchTests\HSE_LED_Test\build.ps1`

Expected: `hse_led_test.elf` created successfully.

**Step 2: Flash**

Run: `python -m pyocd load -u 0001A0000001 -t stm32h743xx -f 1000000 -M attach .\BenchTests\HSE_LED_Test\build\hse_led_test.elf`

Expected: load succeeds without the flaky disconnect behavior seen on `pyocd flash`.

**Step 3: Report bench meaning**

- Document:
  - `PB8` slow blink = HSE path reached main loop.
  - `PB9` fast blink = HSE setup failed.

