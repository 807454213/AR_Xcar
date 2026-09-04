cmake_minimum_required(VERSION 3.10)

project(rknn_ppocr_det_demo)

# 1. 设置 C++ 标准（多线程需要至少 C++11）
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (ENABLE_ASAN)
    message(STATUS "BUILD WITH ADDRESS SANITIZER")
    set (CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
    set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
    set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
endif ()

# 2. 引入项目依赖的工具库
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/ 3rdparty.out)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../utils/ utils.out)

# 3. OpenCV 配置 (针对 Orange Pi 5 本地编译进行了优化)
# 优先寻找系统安装的 OpenCV，这样对视频格式（FFmpeg）的支持最好
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_DIR=${OpenCV_DIR}")
message(STATUS "OpenCV_LIBS=${OpenCV_LIBS}")

set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")

# 4. 根据硬件选择 NPU 推理实现
if (TARGET_SOC STREQUAL "rk1808" OR TARGET_SOC STREQUAL "rv1109" OR TARGET_SOC STREQUAL "rv1126")
    set(ppocr_det_file rknpu1/ppocr_det.cc)
else()
    # RK3588 (Orange Pi 5) 走这里
    set(ppocr_det_file rknpu2/ppocr_det.cc)
endif()

# 5. 定义可执行程序及其源码
# 确保 main.cc 是你刚才修改过的那个支持视频和多线程的版本
add_executable(${PROJECT_NAME}
    main.cc
    postprocess.cc
    clipper.cc
    ${ppocr_det_file}
)

# 6. 链接库 (添加了 Threads 确保多线程正常)
find_package(Threads REQUIRED)

target_link_libraries(${PROJECT_NAME}
    fileutils
    imageutils
    imagedrawing
    ${OpenCV_LIBS}
    ${LIBRKNNRT}
    Threads::Threads
)

if (CMAKE_SYSTEM_NAME STREQUAL "Android")
    target_link_libraries(${PROJECT_NAME} log )
endif()

# 7. 头文件路径
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${LIBRKNNRT_INCLUDES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/rknpu2/include
)

# 8. 安装配置 (执行 make install 时会将程序和模型打包到 install 文件夹)
install(TARGETS ${PROJECT_NAME} DESTINATION .)
file(GLOB RKNN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/../model/*.rknn")
install(FILES ${RKNN_FILES} DESTINATION model)
# 如果有测试视频，也可以一并安装
# install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../model/test_video.mp4 DESTINATION model)