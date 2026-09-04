#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CHECKED_FILES = [
    "src/app/Pipeline.cpp",
    "src/control/drive_control.cpp",
    "src/control/UartCommander.cpp",
    "src/perception/imgprocess.cpp",
    "src/perception/ppseg_infer.cpp",
    "src/io/config.cpp",
    "src/io/videocapture.cpp",
    "src/io/uart.cpp",
    "src/io/llm_decision.cpp",
    "Uart/HardwareProxy.hpp",
    "common/include/UdsIpc.hpp",
    "AI/base/func.cpp",
    "AI/base/rknnpool.cpp",
    "AI/base/postprocess.cc",
    "AI/base/ppyoloe.cc",
    "AI/base/utils/image_utils.c",
    "AI/base/utils/file_utils.c",
    "AI/base/utils/image_drawing.c",
    "AI/PPOCR-1/PPOCR-System/cpp/main.cc",
    "AI/PPOCR-1/PPOCR-System/cpp/rknpu2/ppocr_det.cc",
    "AI/PPOCR-1/PPOCR-System/cpp/rknpu2/ppocr_rec.cc",
    "AI/PPOCR-1/PPOCR-System/cpp/rknpu2/ppocr_system.cc",
]
DIRECT_OUTPUT = re.compile(
    r"\b(?:std::)?(?:cout|cerr|clog)\b|(?<![A-Za-z0-9_])printf\s*\("
    r"|fprintf\s*\(\s*(?:stdout|stderr)\b"
)
ALLOWED_DIRECT_OUTPUT = {
    "src/perception/ppseg_infer.cpp": (
        '"[PPSeg] cannot open model:',
        '"[PPSeg] failed to read model:',
        '"[PPSeg] rknn_init failed:',
        '"[PPSeg] rknn_query IN_OUT_NUM failed:',
        '"[PPSeg] rknn_query INPUT_ATTR[',
        '"[PPSeg] rknn_query OUTPUT_ATTR[',
        '"[PPSeg] track seg OK model=',
    ),
}


def without_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//.*", "", text)


violations = []
for relative in CHECKED_FILES:
    path = ROOT / relative
    for line_no, line in enumerate(
            without_comments(path.read_text()).splitlines(), 1):
        if DIRECT_OUTPUT.search(line):
            allowed = ALLOWED_DIRECT_OUTPUT.get(relative, ())
            if any(prefix in line for prefix in allowed):
                continue
            violations.append(f"{relative}:{line_no}: {line.strip()}")

if violations:
    print("Unmanaged terminal output:")
    print("\n".join(violations))
    sys.exit(1)

print("terminal output policy passed")
