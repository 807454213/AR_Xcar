#!/usr/bin/env python3
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

pipeline = (ROOT / "src/app/Pipeline.cpp").read_text(encoding="utf-8")
postprocess = (ROOT / "AI/base/postprocess.cc").read_text(encoding="utf-8")
names = (ROOT / "AI/base/model/coco.names").read_text(encoding="utf-8")

violations = []

if "xcarAiClassAllowedForControl" not in pipeline:
    violations.append("src/app/Pipeline.cpp: missing xcarAiClassAllowedForControl whitelist")

if "detection.class_id != SIGN" in pipeline and "xcarAiClassAllowedForControl(detection.class_id)" not in pipeline:
    violations.append("src/app/Pipeline.cpp: predictable AI detections are not class-whitelisted")

if "(char*)\"speed\"" in postprocess:
    violations.append("AI/base/postprocess.cc: speed label remains in display label table")

if any(line.strip() == "speed" for line in names.splitlines()):
    violations.append("AI/base/model/coco.names: speed label remains in model label file")

if violations:
    print("Deleted AI classes must be blocked before the control/main path:")
    print("\n".join(violations))
    sys.exit(1)

print("deleted AI class policy passed")
