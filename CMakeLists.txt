cmake_minimum_required(VERSION 3.10)
project(Xcar2)

add_compile_definitions(XCAR_PROJECT_ROOT="${CMAKE_SOURCE_DIR}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 OpenMP（跨平台兼容）
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message(STATUS "OpenMP enabled: ${OpenMP_CXX_FLAGS}")
endif()

# ARM 架构优化：NEON SIMD + 浮点优化（适用于 RK3588 / Cortex-A76/A55）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -DNDEBUG")

# OpenCV
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    message(STATUS "OpenCV version: ${OpenCV_VERSION}")
endif()

# 头文件目录（include/ 下含 control/ app/ perception/ 子目录）
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/Uart)
include_directories(${CMAKE_SOURCE_DIR}/common)
include_directories(${CMAKE_SOURCE_DIR}/common/include)
include_directories(${CMAKE_SOURCE_DIR}/AI/base)
include_directories(${CMAKE_SOURCE_DIR}/AI/base/utils)
include_directories(/usr/include/rga)
include_directories(${CMAKE_SOURCE_DIR}/AI/base/third_party/stb)
include_directories(${CMAKE_SOURCE_DIR}/AI/PPOCR-1/PPOCR-System/cpp)
include_directories(${CMAKE_SOURCE_DIR}/AI/PPOCR-1/PPOCR-System/cpp/rknpu2)
include_directories(${CMAKE_SOURCE_DIR}/AI/PPOCR-1/PPOCR-Det/3rdparty/rknpu2/include)

#=============================================================================
# 源文件（分层：app / control / perception / io + 第三方 AI 内核）
#=============================================================================
set(SOURCES
    # app —— 主循环与 HUD
    src/main.cpp
    src/app/Pipeline.cpp
    src/app/sign_llm_requests.cpp
    src/app/Hud.cpp
    src/app/lost_track_steer.cpp
    src/app/keyboard_input.cpp
    src/app/perf_hud.cpp
    src/app/ppseg_async_policy.cpp
    src/app/uart_fail_fast.cpp
    # control —— 顶层状态机、元素子状态机、引导线、统一 UART 出口
    src/control/drive_control.cpp
    src/control/ped_relative_away.cpp
    src/control/UartCommander.cpp
    src/control/sign_strategy.cpp
    # perception —— 感知（逐字保留已标定算法）
    src/perception/imgprocess.cpp
    src/perception/ppseg_infer.cpp
    src/perception/camera_model.cpp
    src/perception/ai_frame_fusion.cpp
    src/perception/ai_control_evidence.cpp
    src/perception/ocr_feed_sample.cpp
    src/perception/sign_ocr_aggregator.cpp
    # io —— 串口/里程/录像/配置/LLM
    src/io/config.cpp
    src/io/videocapture.cpp
    src/io/function.cpp
    src/io/uart.cpp
    src/io/terminal_output.cpp
    src/io/llm_decision.cpp
    # 第三方 AI 内核（RKNN / PPYOLOe / PPOCR）
    AI/base/func.cpp
    AI/base/rknnpool.cpp
    AI/base/postprocess.cc
    AI/base/ppyoloe.cc
    AI/base/utils/image_utils.c
    AI/base/utils/file_utils.c
    AI/base/utils/image_drawing.c
    AI/PPOCR-1/PPOCR-System/cpp/rknpu2/ppocr_system.cc
    AI/PPOCR-1/PPOCR-System/cpp/rknpu2/ppocr_det.cc
    AI/PPOCR-1/PPOCR-System/cpp/rknpu2/ppocr_rec.cc
    AI/PPOCR-1/PPOCR-System/cpp/postprocess.cc
    AI/PPOCR-1/PPOCR-System/cpp/clipper.cc
    AI/PPOCR-1/PPOCR-System/cpp/main.cc
)

add_executable(main ${SOURCES})

target_link_libraries(main
    ${OpenCV_LIBS}
    rknnrt
    pthread
    rt
    rga
    m
    turbojpeg
    curl
    ssl
    crypto
)

set_target_properties(main PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

message(STATUS "=================================")
message(STATUS "  Xcar2 Main Build (layered)")
message(STATUS "=================================")
message(STATUS "OpenCV:        ${OpenCV_VERSION}")
message(STATUS "Build type:    ${CMAKE_BUILD_TYPE}")
message(STATUS "Output:        ${CMAKE_BINARY_DIR}/bin/main")
message(STATUS "CXX flags:     ${CMAKE_CXX_FLAGS}")
