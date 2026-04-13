# HSE LED Test Design

**Goal:** Build a standalone STM32H743 bench-test firmware that uses the board's 25 MHz HSE path and drives `PB8` / `PB9` LEDs to distinguish successful clock startup from early boot failure.

**Context**

- The main FOC firmware already proved that the board can run under `HSI`, but repeated `HSE` bring-up attempts stalled inside `SystemClock_Config()`.
- The user wants an isolated hardware check rather than another pass through the full motor-control stack.
- The indicator must remain meaningful even if `HAL_RCC_OscConfig()` fails.

**Chosen Approach**

- Create a new minimal project under a dedicated directory, reusing the current repository's STM32H743 HAL, CMSIS startup file, and linker script.
- Initialize `GPIOB` for `PB8` / `PB9` immediately after `HAL_Init()` and before the HSE clock switch attempt.
- Attempt the same `25 MHz HSE + PLL1 -> 480 MHz SYSCLK` configuration used by the main project.
- Replace the normal hard-stop error path with a dedicated failure blink loop that does not depend on `SysTick`.

**LED Semantics**

- Success: `PB8` slow blink, `PB9` off.
- Failure: `PB9` fast blink, `PB8` off.

**Clock / Boot Rules**

- The standalone test uses `HSE` only; it does not silently fall back to `HSI`.
- `SystemClock_Config()` returns `HAL_OK` / `HAL_ERROR` instead of trapping in place.
- The main loop only runs when the full HSE + PLL clock tree is accepted by HAL.

**Implementation Shape**

- New directory: `BenchTests/HSE_LED_Test/`
- Core files:
  - `main.c`
  - `main.h`
  - `stm32h7xx_hal_conf.h`
  - build script
- Reused shared files:
  - `Drivers/STM32H7xx_HAL_Driver/*`
  - `Drivers/CMSIS/*`
  - existing startup assembly
  - existing linker script

**Failure Loop**

- No `HAL_Delay()` in the failure loop.
- Use a simple busy-wait delay helper so the error indication still works if the clock switch failed before the final tick configuration is stable.

**Verification**

- Add a source-contract test that checks:
  - the standalone project files exist;
  - `PB8` / `PB9` are configured as outputs;
  - the test firmware uses `RCC_OSCILLATORTYPE_HSE`, `RCC_HSE_ON`, and `RCC_PLLSOURCE_HSE`;
  - success and failure blink loops are both present.
- Build the standalone project into a dedicated output artifact.
- Flash it with `pyocd load` rather than `pyocd flash`, because the current CMSIS-DAP session is stable with `load` and flaky on `flash` disconnect.

