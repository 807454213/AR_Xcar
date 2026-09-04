cmake_minimum_required(VERSION 3.10)
project(SmartCar_AR_Inference)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

# 确保包含了所有头文件路径
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
    /usr/include/rga  
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/stb  
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

add_executable(smart_car_app 
    main.cpp 
    func.cpp 
    infer_wrap.cpp 
    rknnpool.cpp
    postprocess.cc
    utils/image_utils.c
    utils/file_utils.c
    utils/image_drawing.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/videocapture.cpp
)

# 链接库，包含数学库 m
target_link_libraries(smart_car_app 
    ${OpenCV_LIBS} 
    rknnrt 
    pthread 
    rt
    rga
    m
    turbojpeg
)