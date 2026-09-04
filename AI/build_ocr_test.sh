#!/bin/bash
# =============================================================================
#   单文件命令行编译 ocr_test.cc
#   用法:  bash build_ocr_test.sh
#   产物:  ./ocr_test
# =============================================================================
set -e

cd "$(dirname "$0")"

XCAR="$(cd .. && pwd)"
PPOCR=$XCAR/AI/PPOCR-1

CFLAGS_COMMON="-O3 -ffast-math -DNDEBUG"
CFLAGS_INC=" \
  -I$PPOCR/PPOCR-System/cpp \
  -I$PPOCR/PPOCR-System/cpp/rknpu2 \
  -I$PPOCR/PPOCR-Det/utils \
  -I$PPOCR/PPOCR-Det/3rdparty/rknpu2/include \
  -I$PPOCR/PPOCR-Det/3rdparty/stb_image \
  -I$XCAR/include \
  -I$XCAR/src \
  -I/usr/include/rga "

OPENCV_FLAGS="$(pkg-config --cflags --libs opencv4 2>/dev/null || pkg-config --cflags --libs opencv)"

OBJDIR="./.ocr_build"
mkdir -p "$OBJDIR"

# ---------- 1. C 源 (utils) ----------
for c in \
  "$PPOCR/PPOCR-Det/utils/file_utils.c" \
  "$PPOCR/PPOCR-Det/utils/image_utils.c" \
  "$PPOCR/PPOCR-Det/utils/image_drawing.c"; do
  obj="$OBJDIR/$(basename "${c%.c}").o"
  echo "[CC]  $c"
  gcc $CFLAGS_COMMON $CFLAGS_INC -c "$c" -o "$obj"
done

# ---------- 2. C++ 源 ----------
CXX_SRCS=(
  "ocr_test.cc"
  "$PPOCR/PPOCR-System/cpp/main.cc"
  "$PPOCR/PPOCR-System/cpp/postprocess.cc"
  "$PPOCR/PPOCR-System/cpp/clipper.cc"
  "$PPOCR/PPOCR-System/cpp/rknpu2/ppocr_system.cc"
  "$PPOCR/PPOCR-System/cpp/rknpu2/ppocr_det.cc"
  "$PPOCR/PPOCR-System/cpp/rknpu2/ppocr_rec.cc"
  "$XCAR/src/io/videocapture.cpp"
  "$XCAR/src/io/llm_decision.cpp"
  "$XCAR/src/io/config.cpp"
  "$XCAR/src/io/terminal_output.cpp"
  "$XCAR/src/perception/camera_model.cpp"
)

for s in "${CXX_SRCS[@]}"; do
  obj="$OBJDIR/$(basename "${s%.*}").o"
  echo "[CXX] $s"
  g++ -std=c++17 $CFLAGS_COMMON -fopenmp $CFLAGS_INC \
      $(pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv) \
      -c "$s" -o "$obj"
done

# ---------- 3. 链接 ----------
echo "[LD]  ocr_test"
g++ -std=c++17 -fopenmp \
    "$OBJDIR"/*.o \
    $OPENCV_FLAGS \
    -lrknnrt -lrga -lturbojpeg -lcurl -lssl -lcrypto -lpthread -lrt -lm \
    -o ocr_test

echo "Build OK -> ./ocr_test"
