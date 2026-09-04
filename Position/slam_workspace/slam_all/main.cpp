#include <iostream>
#include <thread>
#include <chrono>

#include "ring_buffer.hpp"
#include "IMU_GET.hpp"
#include "n10p_driver.hpp"
#include "AlgorithmClient.hpp"
#include "FusionEngine.hpp"

// 实例化全局无锁队列 (内存隔离，防爆栈)
LockFreeRingBuffer<ImuData, 1024> g_imu_queue;
LockFreeRingBuffer<EncoderPacket, 256> g_encoder_queue;

static std::array<LidarScan, 32> g_lidar_memory_pool;
LockFreeRingBuffer<LidarScan*, 32> g_lidar_free_queue;
LockFreeRingBuffer<LidarScan*, 32> g_lidar_data_queue;

int main() {
    // std::cout << ">>> 启动底层外设驱动..." << std::endl;
    std::string target_ip = "10.249.33.99";
    std::string target_ip_2 = "10.249.17.215";
    int target_port =  9005;
    int coordinate_port = 9010;
    for (int i = 0; i < 32; ++i) {
        g_lidar_memory_pool[i].valid_points_count = 0;
        g_lidar_free_queue.push(&g_lidar_memory_pool[i]);
    }

    // 1. 启动 IMU (向 g_imu_queue 压数据)
    ImuH30ThreadSafe imu_sensor("/dev/my_imu", &g_imu_queue); // 注意核对你的端口号
    if (!imu_sensor.start()) {
        std::cerr << "IMU 启动失败！" << std::endl;
        return -1;
    }

    // 2. 启动底层主控通信 (获取里程计，向 g_encoder_queue 压数据)
    AlgorithmClient uds_client(&g_encoder_queue);
    if (!uds_client.start("/tmp/robot_hw.sock")) { // 核对你的 uds 路径
        std::cerr << "UDS 客户端连接失败！" << std::endl;
        return -1;
    }
    PoseUdpSender udp_monitor;
    if (!udp_monitor.init(target_ip, target_port, "RK3588_Prod", 0.16, 200.0)) {
        std::cerr << "UDP Monitor 初始化失败！" << std::endl;
     }

    PoseUdpSender udp_coordinate;
    if (!udp_coordinate.init(target_ip_2, coordinate_port, "RK3588_Prod", 0.16, 100.0)) {
        std::cerr << "UDP 9010 坐标发送初始化失败！" << std::endl;
    }

    // 3. 启动 N10P 雷达 (向 g_lidar_queue 压数据)
    N10PLidar lidar_sensor;
    lidar_sensor.attachQueues(&g_lidar_free_queue, &g_lidar_data_queue);
    // 这里假设你的 N10P 有一个 start 接口。如果没有可以自己在此调用
    lidar_sensor.start("/dev/my_lidar");

    // ==========================================
    // 4. 点火！启动 LIO 融合大脑
    // ==========================================
    FusionEngine engine(&g_imu_queue, &g_encoder_queue, &g_lidar_free_queue, &g_lidar_data_queue, &udp_monitor, &udp_coordinate);
    //engine.setManualAnchor(6.08992,0.440017,-94.6181);
  //  engine.injectPriorPoseAndRelocalize(1.5,0.5,0.1); // 这里的局部先验是相对于锚点的增量，单位是米和度
    try {
        engine.start();
    } catch (const std::exception& e) {
        std::cerr << "致命错误: " << e.what() << std::endl;
        return -1;
    }

    // 主线程可以在这里挂起，或者执行 ROS node 逻辑
    // std::cout << "主程序进入驻留状态。按 Ctrl+C 退出。" << std::endl;
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 退出资源清理
    engine.stop();
    imu_sensor.stop();
    uds_client.stopSpinning();

    return 0;
}
