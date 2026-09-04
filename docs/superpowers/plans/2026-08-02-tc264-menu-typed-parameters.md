# TC264 Menu Typed Parameters Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent TC264 menu display and editing from reading or writing beyond `uint8` and `int16` parameter objects.

**Architecture:** Add a header-only typed integer access boundary that converts `uint8_t`, `int16_t`, or `int32_t` values to an editing `int32_t` and stores them through the matching pointer type. Keep the existing float path and menu behavior, while binding each integer menu item to a correctly typed callback.

**Tech Stack:** C99, `<stdint.h>`, GCC host tests, TASKING AURIX target project

## Global Constraints

- Preserve existing menu controls, step sizes, Flash writes, and float behavior.
- Do not change parameter ranges in this fix.
- Do not modify unrelated Xcar or TC264 files.

---

### Task 1: Typed Menu Parameter Access

**Files:**
- Create: `/home/orangepi/Desktop/TC264/code/menu_param_access.h`
- Modify: `/home/orangepi/Desktop/TC264/host_tests/test_tc264_reliability.c`
- Modify: `/home/orangepi/Desktop/TC264/code/Menu.c`

**Interfaces:**
- Produces: `MenuIntegerType`, `menu_integer_read(const void *, MenuIntegerType)`, and `menu_integer_write(void *, MenuIntegerType, int32_t)`.
- Produces menu callbacks: `menu_tuning_uint8`, `menu_tuning_int16`, and existing `menu_tuning_int` for 32-bit values.

- [ ] **Step 1: Write the failing sentinel tests**

Add tests that include `menu_param_access.h`, read all three integer widths, and write `uint8_t` and `int16_t` fields surrounded by sentinel bytes. Assert the edited value changes while every sentinel remains unchanged.

- [ ] **Step 2: Run the host tests and verify RED**

Run: `bash /home/orangepi/Desktop/TC264/host_tests/run_tests.sh`

Expected: compilation fails because `menu_param_access.h` and its interfaces do not exist.

- [ ] **Step 3: Implement the minimal typed access header**

Define `MENU_INTEGER_UINT8`, `MENU_INTEGER_INT16`, and `MENU_INTEGER_INT32`. Implement C99 `static inline` read/write switches using only the pointer type selected by the enum. Invalid types return `0` and do not write.

- [ ] **Step 4: Run the host tests and verify GREEN**

Run: `bash /home/orangepi/Desktop/TC264/host_tests/run_tests.sh`

Expected: exit status `0` with no warnings under `-Wall -Wextra -Werror`.

- [ ] **Step 5: Route Menu.c through typed access**

Include `menu_param_access.h`. Change integer editing to accept a `MenuIntegerType`, add `uint8` and `int16` wrappers, and keep `menu_tuning_int` mapped to `MENU_INTEGER_INT32`. Rebind mode flags to the `uint8` wrapper and PWM values to the `int16` wrapper. Update list rendering to select the same type from the callback identity.

- [ ] **Step 6: Audit and verify**

Run the host test command again. Use `rg` to confirm no `*(int *)param` or `*(int *)menu[i].param` remains, all four mode flags use `menu_tuning_uint8`, all PWM entries use `menu_tuning_int16`, and `runtime`, `buzzer`, and `wireless` remain on `menu_tuning_int`.

- [ ] **Step 7: Attempt target build and record environment result**

Run: `make -C /home/orangepi/Desktop/TC264/Debug`

Expected on the Orange Pi: the generated Windows/TASKING project cannot complete because the TASKING compiler and original Windows paths are unavailable. Record the exact first failure without changing generated build files.
