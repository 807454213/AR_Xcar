#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
BUILD="${ROOT}/build"
mkdir -p "${BUILD}"
cd "${BUILD}"
cmake ..
make -j"$(nproc 2>/dev/null || echo 4)"
echo ""
echo "OK: ${BUILD}/bin/shm_test"
echo "Run: ${BUILD}/bin/shm_test"
echo "     ${BUILD}/bin/shm_test --compare"
