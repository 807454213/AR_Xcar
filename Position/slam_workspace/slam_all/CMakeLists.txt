cmake_minimum_required(VERSION 3.10)
project(LioEngine LANGUAGES CXX)

# --- 1. 编译标准与模式设定 ---
# 强烈建议使用 C++17，较新的 Sophus 和 Ceres 对 C++17 支持更好，且能开启更多编译器优化
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 确保默认以 Release 模式编译（最高性能）
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# --- 2. 极限性能压榨 (针对 RK3588 / ARM 架构) ---
# -O3: 开启最高级别优化
# -ffast-math: 允许浮点数激进优化（牺牲极微小精度换取数倍数学计算速度，对 LIO 盲推极度友好）
# -pthread: 明确启用多线程支持
add_compile_options(-O3 -ffast-math -pthread)

# --- 3. 寻找依赖库 ---
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED) # 必须显式寻找 Sophus
find_package(OpenMP REQUIRED)

# --- 4. 头文件包含 ---
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    # 现代 CMake 推荐：Ceres 和 Sophus 的头文件会通过 target_link_libraries 自动继承
)

# --- 5. 编译目标 ---
# 因为你将核心逻辑全部封装成了高效率的 Header-Only (.hpp) 模式
# 所以只需要编译 main.cpp 即可
add_executable(robot_core main.cpp)

# --- 6. 链接库 ---
target_link_libraries(robot_core 
    PRIVATE 
    ${CERES_LIBRARIES} 
    Eigen3::Eigen
    Sophus::Sophus         # 链接 Sophus 李群库
    OpenMP::OpenMP_CXX     # 链接 OpenMP 多线程加速 (用于全局重定位加速)
    pthread                # 链接 Linux 原生线程库
)