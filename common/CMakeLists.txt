# common/CMakeLists.txt
add_library(robot_common INTERFACE)

# 暴露 include 文件夹的路径给外部
target_include_directories(robot_common INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)