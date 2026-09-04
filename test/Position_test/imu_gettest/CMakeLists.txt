cmake_minimum_required(VERSION 3.10)

# 项目名称和版本
project(imu_uwb VERSION 1.0 LANGUAGES CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 查找必要的库
find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
# 添加源文件
set(SOURCES
    main.cpp
    ImuPredict.cpp
    UwbCorrect.cpp
    initializer.cpp
    NhcCorrect.cpp
    uart_send.cpp
)

# 创建可执行文件
add_executable(imu_uwb ${SOURCES})

# 设置包含目录（当前目录）
target_include_directories(imu_uwb PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(imu_uwb PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/include 
)
# 链接依赖库
target_link_libraries(imu_uwb PRIVATE 
    nlohmann_json::nlohmann_json 
    Sophus::Sophus 
    Threads::Threads 
    m
)

# 添加Eigen3包含目录
target_include_directories(imu_uwb PRIVATE ${EIGEN3_INCLUDE_DIR})




# 编译选项
target_compile_options(imu_uwb PRIVATE
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/uwb_config.json 
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
# 打印构建信息
message(STATUS "Building ${PROJECT_NAME} version ${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")