cmake_minimum_required(VERSION 3.10)
project(PPSeg LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall")

set(XCAR_AI_BASE "${CMAKE_SOURCE_DIR}/../base" CACHE PATH "Xcar AI base (rknn)")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(RKNN_LIB_ARCH "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    set(RKNN_LIB_ARCH "armhf")
else()
    set(RKNN_LIB_ARCH "aarch64")
endif()

set(RKNN_SOC "RK3588" CACHE STRING "RK3588 | RK356X | RV1106")
set(RKNN_INCLUDE_DIR "${XCAR_AI_BASE}")
set(RKNN_INCLUDE_DIR_ALT
    "${XCAR_AI_BASE}/rknpu2/runtime/${RKNN_SOC}/Linux/librknn_api/include")
set(RKNN_RUNTIME_DIR
    "${XCAR_AI_BASE}/rknpu2/runtime/${RKNN_SOC}/Linux/librknn_api/${RKNN_LIB_ARCH}")
set(RKNNRT_LIB "${RKNN_RUNTIME_DIR}/librknnrt.so")

if(NOT EXISTS "${RKNNRT_LIB}")
    message(FATAL_ERROR "librknnrt.so not found: ${RKNNRT_LIB}")
endif()

add_library(rknnrt SHARED IMPORTED GLOBAL)
set_target_properties(rknnrt PROPERTIES
    IMPORTED_LOCATION "${RKNNRT_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${RKNN_INCLUDE_DIR};${RKNN_INCLUDE_DIR_ALT}"
)

find_package(OpenCV REQUIRED)

set(PPSEG_CORE_SRCS
    src/videocapture.cpp
    src/seg/SegThread.cpp
)

add_executable(ppseg src/main.cpp ${PPSEG_CORE_SRCS})

add_executable(view_ppseg src/view_ppseg.cpp ${PPSEG_CORE_SRCS})

foreach(tgt ppseg view_ppseg)
    target_include_directories(${tgt} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/include/common
        ${CMAKE_SOURCE_DIR}/include/seg
        ${RKNN_INCLUDE_DIR}
        ${RKNN_INCLUDE_DIR_ALT}
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(${tgt} PRIVATE
        ${OpenCV_LIBS}
        rknnrt
        pthread
        rt
    )
    set_target_properties(${tgt} PROPERTIES
        BUILD_RPATH "${RKNN_RUNTIME_DIR}"
        INSTALL_RPATH "${RKNN_RUNTIME_DIR}"
    )
endforeach()

file(GLOB PPSEG_MODELS "${CMAKE_SOURCE_DIR}/models/*.rknn")
foreach(tgt ppseg view_ppseg)
    add_custom_command(TARGET ${tgt} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:${tgt}>/models
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${RKNNRT_LIB} $<TARGET_FILE_DIR:${tgt}>/
        COMMENT "Deploy librknnrt.so and models for ${tgt}"
    )
    foreach(m ${PPSEG_MODELS})
        get_filename_component(mname ${m} NAME)
        add_custom_command(TARGET ${tgt} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${m} $<TARGET_FILE_DIR:${tgt}>/models/${mname}
        )
    endforeach()
endforeach()

install(TARGETS ppseg view_ppseg DESTINATION bin)
install(DIRECTORY models/ DESTINATION bin/models FILES_MATCHING PATTERN "*.rknn")
