# Xcar Terminal Output Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the running Xcar program print only model initialization, raw OCR text, actual LLM input, final LLM results, and deduplicated fatal errors.

**Architecture:** Add one thread-safe `terminal_output` component that owns the five permitted line formats and fatal-key deduplication. Migrate the OCR/LLM/model initialization call sites to that component, delete all other direct runtime output, and enforce the boundary with a source-policy test that ignores file-writing `fprintf` calls and test executables.

**Tech Stack:** C++17, CMake, `std::mutex`, `std::unordered_set`, `std::ostream`, Python 3 source-policy test, existing Xcar test build.

## Global Constraints

- Preserve the current uncommitted prompt changes in `src/io/llm_decision.cpp`; edit only its terminal-output statements.
- Do not change OCR results, aggregation, LLM request payloads, LLM parsing, state machines, UART behavior, model inference, or configuration values.
- Keep these line categories: `[MODEL INIT]`, `[OCR RAW]`, `[LLM INPUT]`, `[LLM RESULT]`, and `[FATAL]`.
- Empty OCR strings do not print.
- `[LLM INPUT]` prints only after an LLM request is successfully accepted; local-only decisions do not print it.
- The same fatal key prints at most once per process.
- Model inference failures that can repeat per frame remain silent; model initialization success/failure may print.
- Test executables may continue printing their own pass/fail diagnostics.
- File serialization through `fprintf(fp, ...)` is not terminal output and must remain unchanged.

## File Structure

- Create `include/io/terminal_output.h`: public five-category output API plus test sink/reset hooks.
- Create `src/io/terminal_output.cpp`: formatting, synchronization, output sink, and fatal-key deduplication.
- Create `test/test_terminal_output.cpp`: unit tests for formats, ordering, empty OCR handling, and fatal deduplication.
- Create `test/check_terminal_output_policy.py`: strips comments and rejects direct terminal writes in the production files listed in the script.
- Modify `CMakeLists.txt`: compile `src/io/terminal_output.cpp` into `main`.
- Modify `test/CMakeLists.txt`: add `test_terminal_output` and link the shared output component where production sources need it.
- Modify `src/app/Pipeline.cpp`: emit model status, LLM input/result, and fatal startup errors through the new API; delete process logs.
- Modify `AI/PPOCR-1/PPOCR-System/cpp/main.cc`: emit raw OCR text through the new API and remove OCR lifecycle output.
- Modify `src/io/llm_decision.cpp`: remove HTTP response, fallback, and credential chatter without touching the prompt.
- Modify runtime-log owners under `src/control/`, `src/io/`, `src/perception/`, `Uart/`, and `common/include/`: remove process logs and retain only deduplicated fatal errors.
- Modify compiled AI backend sources under `AI/base/` and `AI/PPOCR-1/PPOCR-System/cpp/rknpu2/`: remove tensor/debug/per-frame output and retain concise initialization/fatal output.

---

### Task 1: Thread-safe terminal output contract

**Files:**
- Create: `include/io/terminal_output.h`
- Create: `src/io/terminal_output.cpp`
- Create: `test/test_terminal_output.cpp`
- Modify: `CMakeLists.txt`
- Modify: `test/CMakeLists.txt`

**Interfaces:**
- Produces: `terminal_output::modelInit(const std::string&, const std::string&)`
- Produces: `terminal_output::ocrRaw(const std::string&)`
- Produces: `terminal_output::llmInput(const std::vector<std::string>&)`
- Produces: `terminal_output::llmResult(const std::string&, int, float, const std::string&)`
- Produces: `terminal_output::fatalOnce(const std::string&, const std::string&)`
- Produces for tests: `terminal_output::testing::setSink(std::ostream*)` and `terminal_output::testing::reset()`

- [ ] **Step 1: Write the failing unit test**

Create `test/test_terminal_output.cpp`:

```cpp
#include "io/terminal_output.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

int main() {
    std::ostringstream out;
    terminal_output::testing::reset();
    terminal_output::testing::setSink(&out);

    terminal_output::modelInit("YOLO", "ready");
    terminal_output::ocrRaw("");
    terminal_output::ocrRaw("原始路牌");
    terminal_output::llmInput({"第一行", "第二行"});
    terminal_output::llmResult("turn_right", 1, 0.91f, "llm");
    terminal_output::fatalOnce("uart-open", "UART unavailable");
    terminal_output::fatalOnce("uart-open", "UART unavailable again");
    terminal_output::fatalOnce("config-open", "config unavailable");

    const std::string expected =
        "[MODEL INIT] YOLO: ready\n"
        "[OCR RAW] 原始路牌\n"
        "[LLM INPUT] 第一行\n"
        "[LLM INPUT] 第二行\n"
        "[LLM RESULT] action=turn_right flag=1 confidence=0.91 source=llm\n"
        "[FATAL] UART unavailable\n"
        "[FATAL] config unavailable\n";
    if (out.str() != expected) {
        std::cerr << "terminal output mismatch\nEXPECTED:\n"
                  << expected << "ACTUAL:\n" << out.str();
        return 1;
    }

    terminal_output::testing::reset();
    std::cout << "terminal output tests passed\n";
    return 0;
}
```

Add the target to `test/CMakeLists.txt` before implementing the component:

```cmake
add_executable(test_terminal_output
    ${CMAKE_CURRENT_LIST_DIR}/test_terminal_output.cpp
    ${ROOT_DIR}/src/io/terminal_output.cpp
)
set_target_properties(test_terminal_output PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
```

- [ ] **Step 2: Run the test build and verify RED**

Run:

```bash
cmake -S test -B test/build
cmake --build test/build --target test_terminal_output -j$(nproc)
```

Expected: compilation fails because `io/terminal_output.h` and the implementation do not exist.

- [ ] **Step 3: Add the minimal public interface**

Create `include/io/terminal_output.h`:

```cpp
#pragma once

#include <iosfwd>
#include <string>
#include <vector>

namespace terminal_output {
void modelInit(const std::string& component, const std::string& message);
void ocrRaw(const std::string& text);
void llmInput(const std::vector<std::string>& texts);
void llmResult(const std::string& action, int flag, float confidence,
               const std::string& source);
void fatalOnce(const std::string& key, const std::string& message);

namespace testing {
void setSink(std::ostream* sink);
void reset();
}
}  // namespace terminal_output
```

Create `src/io/terminal_output.cpp`:

```cpp
#include "io/terminal_output.h"

#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <unordered_set>

namespace terminal_output {
namespace {
std::mutex output_mutex;
std::unordered_set<std::string> fatal_keys;
std::ostream* test_sink = nullptr;

void emitLocked(const std::string& line) {
    std::ostream& out = test_sink ? *test_sink : std::cout;
    out << line << '\n';
    out.flush();
}

void emit(const std::string& line) {
    std::lock_guard<std::mutex> lock(output_mutex);
    emitLocked(line);
}
}  // namespace

void modelInit(const std::string& component, const std::string& message) {
    emit("[MODEL INIT] " + component + ": " + message);
}

void ocrRaw(const std::string& text) {
    if (!text.empty()) emit("[OCR RAW] " + text);
}

void llmInput(const std::vector<std::string>& texts) {
    for (const auto& text : texts) {
        if (!text.empty()) emit("[LLM INPUT] " + text);
    }
}

void llmResult(const std::string& action, int flag, float confidence,
               const std::string& source) {
    std::ostringstream line;
    line << "[LLM RESULT] action=" << action
         << " flag=" << flag
         << " confidence=" << std::fixed << std::setprecision(2) << confidence
         << " source=" << source;
    emit(line.str());
}

void fatalOnce(const std::string& key, const std::string& message) {
    std::lock_guard<std::mutex> lock(output_mutex);
    if (fatal_keys.insert(key).second) emitLocked("[FATAL] " + message);
}

namespace testing {
void setSink(std::ostream* sink) {
    std::lock_guard<std::mutex> lock(output_mutex);
    test_sink = sink;
}

void reset() {
    std::lock_guard<std::mutex> lock(output_mutex);
    fatal_keys.clear();
    test_sink = nullptr;
}
}  // namespace testing
}  // namespace terminal_output
```

Add `src/io/terminal_output.cpp` to the root `SOURCES` list immediately before `src/io/llm_decision.cpp`.

- [ ] **Step 4: Run the focused test and verify GREEN**

Run:

```bash
cmake --build test/build --target test_terminal_output -j$(nproc)
./test/build/bin/test_terminal_output
```

Expected: `terminal output tests passed`.

- [ ] **Step 5: Commit the output contract**

```bash
git add include/io/terminal_output.h src/io/terminal_output.cpp \
    test/test_terminal_output.cpp CMakeLists.txt test/CMakeLists.txt
git commit -m "feat: add terminal output contract"
```

---

### Task 2: OCR, LLM, and Pipeline integration

**Files:**
- Create: `test/check_terminal_output_policy.py`
- Modify: `src/app/Pipeline.cpp`
- Modify: `AI/PPOCR-1/PPOCR-System/cpp/main.cc`
- Modify: `src/io/llm_decision.cpp`

**Interfaces:**
- Consumes: all five `terminal_output` functions from Task 1.
- Produces: raw OCR output at recognition time, LLM input only after accepted submission, final LLM result output, and concise model initialization output.

- [ ] **Step 1: Add the first source-policy check**

Create `test/check_terminal_output_policy.py`:

```python
#!/usr/bin/env python3
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CHECKED_FILES = [
    "src/app/Pipeline.cpp",
    "src/io/llm_decision.cpp",
    "AI/PPOCR-1/PPOCR-System/cpp/main.cc",
]
DIRECT_OUTPUT = re.compile(
    r"\bstd::(?:cout|cerr|clog)\b|(?<![A-Za-z0-9_])printf\s*\("
    r"|fprintf\s*\(\s*(?:stdout|stderr)\b"
)

def without_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//.*", "", text)

violations = []
for relative in CHECKED_FILES:
    path = ROOT / relative
    for line_no, line in enumerate(without_comments(path.read_text()).splitlines(), 1):
        if DIRECT_OUTPUT.search(line):
            violations.append(f"{relative}:{line_no}: {line.strip()}")

if violations:
    print("Unmanaged terminal output:")
    print("\n".join(violations))
    sys.exit(1)

print("terminal output policy passed")
```

- [ ] **Step 2: Run the policy check and verify RED**

Run:

```bash
python3 test/check_terminal_output_policy.py
```

Expected: FAIL listing existing direct output in `Pipeline.cpp`, `llm_decision.cpp`, and OCR `main.cc`.

- [ ] **Step 3: Route the permitted Pipeline output**

Add:

```cpp
#include "io/terminal_output.h"
```

Use these exact calls at the existing lifecycle points:

```cpp
terminal_output::modelInit("YOLO", "ready");
terminal_output::modelInit("PPSeg", "ready");
terminal_output::modelInit("PPSeg", "failed; track mask unavailable");
terminal_output::modelInit("OCR", "ready");
terminal_output::modelInit("OCR", "failed");
```

Replace the valid LLM-result `cout` block with:

```cpp
terminal_output::llmResult(cmd.action, cmd.flag, cmd.confidence, cmd.source);
```

After `g_llm_requests.submit(...)` returns true, replace the request-summary and line loop with:

```cpp
terminal_output::llmInput(sign_texts);
```

Do not call `llmInput` on the local-rule branch or on rejected submissions. Delete Pipeline output for stale requests, OCR processor/session state, OCR result metadata, keyboard/FPS/UART/control lifecycle, release, and exit. Replace unknown runtime mode and race-mode hardware failure with:

```cpp
terminal_output::fatalOnce(
    "runtime-mode", "Unknown runtimeMode: " + APP.runtimeMode);
terminal_output::fatalOnce(
    "race-hardware", "race mode requires an available hardware proxy");
```

- [ ] **Step 4: Route raw OCR and silence LLM transport chatter**

In `AI/PPOCR-1/PPOCR-System/cpp/main.cc`, include the terminal header and replace:

```cpp
printf("[Frame %d] 识别结果: %s | 置信度: %.2f\n",
       task.frame_index, rec_res.str, rec_res.score);
```

with:

```cpp
terminal_output::ocrRaw(rec_res.str);
```

Delete the OCR lifecycle `printf` statements in this file.

In `src/io/llm_decision.cpp`, delete only the existing HTTP GET/POST response, bearer response, fallback, raw error, and missing-credential terminal statements. Leave the prompt template and all command behavior byte-for-byte unchanged.

- [ ] **Step 5: Run the policy and focused regressions**

Run:

```bash
python3 test/check_terminal_output_policy.py
cmake --build test/build --target test_terminal_output test_sign_llm_requests \
    test_sign_session_isolation test_llm_valid_decision -j$(nproc)
./test/build/bin/test_terminal_output
./test/build/bin/test_sign_llm_requests
./test/build/bin/test_sign_session_isolation
./test/build/bin/test_llm_valid_decision
```

Expected: policy PASS and all four executables exit 0.

- [ ] **Step 6: Commit the OCR/LLM integration**

```bash
git add test/check_terminal_output_policy.py src/app/Pipeline.cpp \
    AI/PPOCR-1/PPOCR-System/cpp/main.cc
git add -p src/io/llm_decision.cpp
git commit -m "feat: reduce OCR and LLM terminal output"
```

In the interactive add, stage only log-removal hunks and reject every prompt-template hunk. Before committing, inspect `git diff --cached -- src/io/llm_decision.cpp` and confirm that the user's prompt edits are absent.

---

### Task 3: Silence control, I/O, and transport process logs

**Files:**
- Modify: `test/check_terminal_output_policy.py`
- Modify: `src/control/drive_control.cpp`
- Modify: `src/control/UartCommander.cpp`
- Modify: `src/perception/imgprocess.cpp`
- Modify: `src/perception/ppseg_infer.cpp`
- Modify: `src/io/config.cpp`
- Modify: `src/io/videocapture.cpp`
- Modify: `src/io/uart.cpp`
- Modify: `Uart/HardwareProxy.hpp`
- Modify: `common/include/UdsIpc.hpp`
- Modify: `test/CMakeLists.txt`

**Interfaces:**
- Consumes: `terminal_output::fatalOnce`.
- Produces: no normal terminal output from control, I/O, perception, UART, or UDS.

- [ ] **Step 1: Expand the policy test**

Append these paths to `CHECKED_FILES`:

```python
    "src/control/drive_control.cpp",
    "src/control/UartCommander.cpp",
    "src/perception/imgprocess.cpp",
    "src/perception/ppseg_infer.cpp",
    "src/io/config.cpp",
    "src/io/videocapture.cpp",
    "src/io/uart.cpp",
    "Uart/HardwareProxy.hpp",
    "common/include/UdsIpc.hpp",
```

- [ ] **Step 2: Run the expanded policy and verify RED**

Run:

```bash
python3 test/check_terminal_output_policy.py
```

Expected: FAIL listing unmanaged control, UART, config, UDS, and perception output.

- [ ] **Step 3: Remove normal process logs**

Delete all direct `printf`, `std::cout`, and `std::cerr` statements in the listed files, including statements currently guarded by `verboseLogs`. Preserve `snprintf` HUD formatting and every `fprintf(fp, ...)` configuration write.

Keep only critical startup errors by including `io/terminal_output.h` and using stable keys:

```cpp
terminal_output::fatalOnce("config-open:" + path,
                           "Cannot open config: " + path);
terminal_output::fatalOnce("uart-open", "UART unavailable");
terminal_output::fatalOnce("uds-socket", "UDS server socket creation failed");
terminal_output::fatalOnce("uds-bind", "UDS server bind failed");
terminal_output::fatalOnce("uds-listen", "UDS server listen failed");
```

Do not report repeated UART write-incomplete, receive-buffer-overflow, inference-frame, state timeout, or sign/gold recording events.

- [ ] **Step 4: Link the output component into tests that compile config**

Add this to `TEST_COMMON_SOURCES`:

```cmake
    ${ROOT_DIR}/src/io/terminal_output.cpp
```

Also add the same source to standalone config targets (`test_gold_band_visual_config`, `test_gold_outside_record_config`, `test_config_cleanup`, `test_ped_relative_config`, `test_lost_track_steer`, `test_det_sync`, `test_ai_inference_mode_config`, `test_sign_ocr_config`, and `test_sign_strategy_config`) if they do not consume `TEST_COMMON_SOURCES`.

- [ ] **Step 5: Run the policy and control/config regressions**

Run:

```bash
python3 test/check_terminal_output_policy.py
cmake --build test/build --target test_sign_strategy_control \
    test_ped_source_driven_control test_vehicle_gold_source_driven_control \
    test_gold_slow_band test_sign_ocr_config test_ai_inference_mode_config \
    -j$(nproc)
./test/build/bin/test_sign_strategy_control
./test/build/bin/test_ped_source_driven_control
./test/build/bin/test_vehicle_gold_source_driven_control
./test/build/bin/test_gold_slow_band
./test/build/bin/test_sign_ocr_config
./test/build/bin/test_ai_inference_mode_config
```

Expected: policy PASS and every executable exits 0.

- [ ] **Step 6: Commit the runtime cleanup**

```bash
git add test/check_terminal_output_policy.py test/CMakeLists.txt \
    src/control/drive_control.cpp src/control/UartCommander.cpp \
    src/perception/imgprocess.cpp src/perception/ppseg_infer.cpp \
    src/io/config.cpp src/io/videocapture.cpp src/io/uart.cpp \
    Uart/HardwareProxy.hpp common/include/UdsIpc.hpp
git commit -m "refactor: silence runtime process logs"
```

---

### Task 4: Silence compiled AI backend diagnostics

**Files:**
- Modify: `test/check_terminal_output_policy.py`
- Modify: `AI/base/func.cpp`
- Modify: `AI/base/rknnpool.cpp`
- Modify: `AI/base/postprocess.cc`
- Modify: `AI/base/ppyoloe.cc`
- Modify: `AI/PPOCR-1/PPOCR-System/cpp/rknpu2/ppocr_det.cc`
- Modify: `AI/PPOCR-1/PPOCR-System/cpp/rknpu2/ppocr_rec.cc`
- Modify: `AI/PPOCR-1/PPOCR-System/cpp/rknpu2/ppocr_system.cc`
- Modify: `test/CMakeLists.txt`

**Interfaces:**
- Consumes: `terminal_output::fatalOnce`.
- Produces: concise initialization/fatal reporting without tensor dumps, debug images, or per-frame inference output.

- [ ] **Step 1: Expand the policy to the compiled AI backend**

Append:

```python
    "AI/base/func.cpp",
    "AI/base/rknnpool.cpp",
    "AI/base/postprocess.cc",
    "AI/base/ppyoloe.cc",
    "AI/PPOCR-1/PPOCR-System/cpp/rknpu2/ppocr_det.cc",
    "AI/PPOCR-1/PPOCR-System/cpp/rknpu2/ppocr_rec.cc",
    "AI/PPOCR-1/PPOCR-System/cpp/rknpu2/ppocr_system.cc",
```

- [ ] **Step 2: Run the expanded policy and verify RED**

Run:

```bash
python3 test/check_terminal_output_policy.py
```

Expected: FAIL listing tensor metadata, model details, debug output, and inference errors.

- [ ] **Step 3: Remove backend diagnostics and retain initialization failures**

Delete tensor dump helpers and all success/debug/per-frame `printf` calls. Replace only initialization failures with stable fatal keys:

```cpp
terminal_output::fatalOnce("yolo-model-read", "YOLO model file could not be read");
terminal_output::fatalOnce("yolo-rknn-init", "YOLO RKNN initialization failed");
terminal_output::fatalOnce("ocr-det-init", "OCR detection model initialization failed");
terminal_output::fatalOnce("ocr-rec-init", "OCR recognition model initialization failed");
terminal_output::fatalOnce("label-load", "YOLO label file could not be loaded");
```

Return the same error codes as before. Do not add terminal output to repeated `rknn_run`, image conversion, output retrieval, post-processing, or debug-image paths.

- [ ] **Step 4: Link the output component into the isolated RKNN pool test**

Update `test_rknnpool_fake`:

```cmake
add_executable(test_rknnpool_fake
    ${CMAKE_CURRENT_LIST_DIR}/test_rknnpool_fake.cpp
    ${ROOT_DIR}/AI/base/rknnpool.cpp
    ${ROOT_DIR}/src/io/terminal_output.cpp
)
```

- [ ] **Step 5: Run the backend policy, focused test, and main build**

Run:

```bash
python3 test/check_terminal_output_policy.py
cmake --build test/build --target test_rknnpool_fake -j$(nproc)
./test/build/bin/test_rknnpool_fake
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

Expected: policy PASS, `test_rknnpool_fake` exits 0, and `build/bin/main` links successfully.

- [ ] **Step 6: Commit the AI cleanup**

```bash
git add test/check_terminal_output_policy.py test/CMakeLists.txt \
    AI/base/func.cpp AI/base/rknnpool.cpp AI/base/postprocess.cc \
    AI/base/ppyoloe.cc \
    AI/PPOCR-1/PPOCR-System/cpp/rknpu2/ppocr_det.cc \
    AI/PPOCR-1/PPOCR-System/cpp/rknpu2/ppocr_rec.cc \
    AI/PPOCR-1/PPOCR-System/cpp/rknpu2/ppocr_system.cc
git commit -m "refactor: silence AI backend diagnostics"
```

---

### Task 5: Final verification and documentation update

**Files:**
- Modify: `Xcar2.md`

**Interfaces:**
- Consumes: completed output policy and all existing OCR/LLM regression targets.
- Produces: documented terminal contract and evidence that the main binary builds.

- [ ] **Step 1: Document the terminal contract**

Add under `app/Pipeline.cpp` responsibilities in `Xcar2.md`:

```markdown
- 终端运行日志统一为 `[MODEL INIT]`、`[OCR RAW]`、`[LLM INPUT]`、
  `[LLM RESULT]` 和去重后的 `[FATAL]`；业务代码不得直接新增
  `printf/std::cout/std::cerr`。`[LLM INPUT]` 只表示实际成功提交给
  LLM 的文本，本地规则直接决策时不会出现。
```

- [ ] **Step 2: Run final static and automated verification**

Run:

```bash
python3 test/check_terminal_output_policy.py
cmake --build test/build -j$(nproc)
./test/build/bin/test_terminal_output
./test/build/bin/test_sign_ocr_aggregator
./test/build/bin/test_sign_llm_requests
./test/build/bin/test_sign_failsafe
./test/build/bin/test_sign_session_isolation
./test/build/bin/test_llm_valid_decision
./test/build/bin/test_rknnpool_fake
cmake --build build -j$(nproc)
git diff --check
```

Expected: policy PASS, all named tests exit 0, main build succeeds, and `git diff --check` produces no output.

- [ ] **Step 3: Audit the final diff**

Run:

```bash
git diff --stat
git diff -- src/io/llm_decision.cpp
git status --short
```

Expected: no prompt/config value is reverted; only intended log changes touch `src/io/llm_decision.cpp`; the user's pre-existing config modifications remain present.

- [ ] **Step 4: Commit documentation**

```bash
git add Xcar2.md
git commit -m "docs: document terminal output contract"
```
