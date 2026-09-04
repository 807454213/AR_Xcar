#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CHECKED_FILES = [
    "Uart/uart.hpp",
    "src/io/uart.cpp",
    "include/control/uart_commander.h",
    "src/control/UartCommander.cpp",
    "Position/slam_workspace/slam_all/uart_send.hpp",
    "Position/slam_workspace/slam_all/uart_send.cpp",
    "test/Position_test/imu_gettest/uart_send.hpp",
    "test/Position_test/imu_gettest/uart_send.cpp",
]
FORBIDDEN = re.compile(r"\b0x0A\b|\bdata_10\b|\bstatus_val\b|\bstatus_val_recv\b")


violations = []
for relative in CHECKED_FILES:
    path = ROOT / relative
    for line_no, line in enumerate(path.read_text().splitlines(), 1):
        if FORBIDDEN.search(line):
            violations.append(f"{relative}:{line_no}: {line.strip()}")

if violations:
    print("Removed UART cmd 0x0A must not remain in main UART protocol files:")
    print("\n".join(violations))
    sys.exit(1)

print("uart protocol policy passed")
