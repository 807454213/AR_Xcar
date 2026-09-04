cmake_minimum_required(VERSION 3.10)

project(rknn_ppocr_system_demo)

# 1. 设置 C++ 标准 (多线程需要至少 C++11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (ENABLE_ASAN)
    message(STATUS "BUILD WITH ADDRESS SANITIZER")
    set (CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
    set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
    set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
endif ()

# 2. 引入项目依赖的工具库
# 注意：请根据你实际的目录层级确认这些路径是否正确
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../PPOCR-Det/3rdparty/ 3rdparty.out)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../PPOCR-Det/utils/ utils.out)

# 3. OpenCV 配置
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_DIR=${OpenCV_DIR}")
message(STATUS "OpenCV_LIBS=${OpenCV_LIBS}")

set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")

# 4. 根据硬件选择 NPU 实现路径
if (TARGET_SOC STREQUAL "rk1808" OR TARGET_SOC STREQUAL "rv1109" OR TARGET_SOC STREQUAL "rv1126")
    set(NPU_DIR rknpu1)
else()
    # RK3588 (Orange Pi 5) 走这里
    set(NPU_DIR rknpu2)
endif()
message(STATUS "Selected NPU_DIR: ${NPU_DIR}")

# 5. 定义可执行程序及其依赖源码
# 确保 ppocr_system.cc 也在 NPU_DIR 文件夹下
add_executable(${PROJECT_NAME}
    main.cc
    postprocess.cc
    clipper.cc
    ${NPU_DIR}/ppocr_system.cc
    ${NPU_DIR}/ppocr_det.cc
    ${NPU_DIR}/ppocr_rec.cc
)

# 6. 多线程库支持 (必须)
find_package(Threads REQUIRED)

# 7. 链接所有的库
target_link_libraries(${PROJECT_NAME}
    fileutils
    imageutils
    imagedrawing
    ${OpenCV_LIBS}
    ${LIBRKNNRT}
    Threads::Threads  # 支持 std::thread
)

if (CMAKE_SYSTEM_NAME STREQUAL "Android")
    target_link_libraries(${PROJECT_NAME} log )
endif()

# 8. 头文件包含路径 (关键修改点)
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/${NPU_DIR}   # 这一行让 ppocr_system.cc 能找到同目录下的 ppocr_det.h
    ${LIBRKNNRT_INCLUDES}
    # 显式包含 3rdparty 中的 rknn 接口头文件
    ${CMAKE_CURRENT_SOURCE_DIR}/../../PPOCR-Det/3rdparty/rknpu2/include
)

# 9. 安装配置
install(TARGETS ${PROJECT_NAME} DESTINATION .)

# 安装测试图片
if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../model/test.jpg)
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../model/test.jpg DESTINATION model)
endif()

# 安装检测模型 (从 PPOCR-Det 目录获取)
set(det_model_path ${CMAKE_CURRENT_SOURCE_DIR}/../../PPOCR-Det/model/ppocrv4_det.rknn)
if (EXISTS ${det_model_path})
    install(FILES ${det_model_path} DESTINATION model)
    message(STATUS "Install Det Model: ${det_model_path}")
endif()

# 安装识别模型 (从 PPOCR-Rec 目录获取)
set(rec_model_path ${CMAKE_CURRENT_SOURCE_DIR}/../../PPOCR-Rec/model/ppocrv4_rec.rknn)
if (EXISTS ${rec_model_path})
    install(FILES ${rec_model_path} DESTINATION model)
    message(STATUS "Install Rec Model: ${rec_model_path}")
endif()

# 自动拷贝本地 model 文件夹下的所有 rknn 文件
file(GLOB RKNN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/../model/*.rknn")
if(RKNN_FILES)
    install(FILES ${RKNN_FILES} DESTINATION model)
endif()