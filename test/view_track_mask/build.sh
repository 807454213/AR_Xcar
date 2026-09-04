#!/usr/bin/env bash
# 独立编译 view_track_mask（不依赖主工程 build/）
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
BUILD="${ROOT}/build"
mkdir -p "${BUILD}"
cd "${BUILD}"
cmake ..
make -j"$(nproc 2>/dev/null || echo 4)"
echo ""
echo "OK: ${BUILD}/bin/view_track_mask"
echo "Run from repo:  ${BUILD}/bin/view_track_mask --config ${ROOT}/../../configs/config.json"
