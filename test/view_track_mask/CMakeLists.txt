cmake_minimum_required(VERSION 3.10)
project(view_track_mask LANGUAGES CXX)

set(XCAR_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -DNDEBUG")

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(OpenCV REQUIRED)

include_directories(
    ${XCAR_ROOT}/include
    ${XCAR_ROOT}/Uart
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(view_track_mask
    ${XCAR_ROOT}/test/view_track_mask.cpp
    ${XCAR_ROOT}/src/io/videocapture.cpp
    ${XCAR_ROOT}/src/perception/imgprocess.cpp
    ${XCAR_ROOT}/src/io/config.cpp
    ${XCAR_ROOT}/src/perception/camera_model.cpp
)

target_link_libraries(view_track_mask
    ${OpenCV_LIBS}
    pthread
    rt
)

set_target_properties(view_track_mask PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

message(STATUS "view_track_mask -> ${CMAKE_BINARY_DIR}/bin/view_track_mask")
