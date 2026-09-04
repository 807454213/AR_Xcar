cmake_minimum_required(VERSION 3.10)
project(shm_test LANGUAGES CXX)

set(XCAR_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -DNDEBUG")

find_package(OpenCV REQUIRED)

include_directories(
    ${XCAR_ROOT}/include
    ${XCAR_ROOT}/AI/PPOCR-1/PPOCR-Det/3rdparty/rknpu2/include
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(shm_test
    ${XCAR_ROOT}/test/shm_test.cpp
    ${XCAR_ROOT}/src/io/config.cpp
    ${XCAR_ROOT}/src/perception/camera_model.cpp
    ${XCAR_ROOT}/src/perception/ppseg_infer.cpp
)

target_compile_definitions(shm_test PRIVATE
    XCAR_PROJECT_ROOT="${XCAR_ROOT}"
)

target_link_libraries(shm_test
    ${OpenCV_LIBS}
    rknnrt
    pthread
    rt
)

set_target_properties(shm_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

message(STATUS "shm_test -> ${CMAKE_BINARY_DIR}/bin/shm_test")
