#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>
#include <unistd.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iomanip>
// 引入我们的 16 维核心算法组件
#include "ring_buffer.hpp"
#include "state.hpp"
#include "EskfManager.hpp"
#include "initializer.hpp"
#include "UdsIpc.hpp"
#include "ipc_messages.hpp"
#include "udp_sender.hpp"
// 引入硬件驱动 (根据你的实际代码路径替换)
#include "IMU_GET.hpp"
#include "uwb_re.hpp"  
#include "uart_send.hpp"
// ===================================================================
// 1. 全局缓冲区：容量 1024，完美覆盖 200Hz 下的 5 秒历史
// ===================================================================
LockFreeESKFBuffer<1024> g_imu_buffer;

using namespace RobotIPC;
Eigen::Vector3d solveRoughUwbPosition(const std::vector<UwbMeasurement, Eigen::aligned_allocator<UwbMeasurement>>& meas_list, 
                                      double anchor_z, double tag_z) 
{
    if (meas_list.size() < 3) return Eigen::Vector3d::Zero();

    int n = meas_list.size() - 1;
    Eigen::MatrixXd A(n, 2);
    Eigen::VectorXd b(n);

    // 计算高度差的平方
    double dz_sq = std::pow(anchor_z - tag_z, 2);

    double x0 = meas_list[0].anchor.x();
    double y0 = meas_list[0].anchor.y();
    // 【核心修正】：3D 距离投影为 2D 水平距离
    // 注意防范异常测距导致根号下为负数
    double d0_sq = std::max(0.0, std::pow(meas_list[0].distance, 2) - dz_sq);

    for (int i = 0; i < n; ++i) {
        double xi = meas_list[i + 1].anchor.x();
        double yi = meas_list[i + 1].anchor.y();
        double di_sq = std::max(0.0, std::pow(meas_list[i + 1].distance, 2) - dz_sq);

        A(i, 0) = 2 * (xi - x0);
        A(i, 1) = 2 * (yi - y0);
        b(i) = (xi * xi - x0 * x0) + (yi * yi - y0 * y0) + (d0_sq - di_sq);
    }

    Eigen::Vector2d xy = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    return Eigen::Vector3d(xy(0), xy(1), tag_z);
}
constexpr double PI = 3.14159265358979323846;
constexpr double MCU_DT = 0.005; // 假设 TC264 每 5ms 发送一次
constexpr double WHEEL_DIAMETER = 0.062955;            // 轮子直径平均
constexpr double ENCODER_RESOLUTION = 4096;
constexpr double GEAR_RATIO = (68.0/30.0);
constexpr double k = 1.06678;
constexpr double METERS_PER_TICK   = (PI * WHEEL_DIAMETER * k) / (ENCODER_RESOLUTION * GEAR_RATIO );

// ===================================================================
// 4. 主程序：流水线发令枪
// ===================================================================
int main() 
{
    std::cout << "===================================================" << std::endl;
    std::cout << "  16 维全自动 ESKF 紧耦合定位系统 (含动态找北)  " << std::endl;
    std::cout << "===================================================" << std::endl;
    std::string target_ip = "192.168.3.191";
    int target_port =  9005; 
    // ----------------------------------------------------
    // 第一步：全局配置解析（唯一一次读取硬盘操作）
    // ----------------------------------------------------
    ConfigManager config;
    if (!config.loadConfig("uwb_config.json")) {
        std::cerr << "[致命错误] 配置文件 uwb_config.json 加载失败，程序退出！" << std::endl;
        return -1;
    }
    
    // 获取全局配置的只读引用
    const Parameters& global_params = config.getParams();

    // --- A. 初始化硬件 ---
    ImuH30ThreadSafe imu_sensor("/dev/my_imu", &g_imu_buffer);
    if (!imu_sensor.start()) {
        std::cerr << "[致命错误] IMU 初始化失败！检查串口或权限。" << std::endl;
        return -1;
    }

    UwbReceiverESKF uwb_sensor;
    if (!uwb_sensor.init(global_params.params_uwb, "/dev/my_uwb", 460800)) {
        std::cerr << "[致命错误] UWB 初始化失败！" << std::endl;
        return -1;
    }
    PoseUdpSender udp_sender;
    udp_sender.init(target_ip, target_port, "RK3588_Prod", 0.16);

    // if(!UartSender::instance().open("/dev/ttyUSB1"))
    // {
    //     std::cerr << "[致命错误] UART 初始化失败！" << std::endl;
    //     return -1;
    // }

    UdsClient uds_client;


    // --- B. 实例化算法大脑与安检门 ---
    // EskfManager 参数：加计噪声, 陀螺仪噪声, 加计零偏游走, 陀螺零偏游走, UWB测距方差
    EskfManager eskf_manager(global_params.params_imu, global_params.params_uwb, global_params.params_nhc);
    EskfInitializer initializer;
    
    // UWB 标签相对于 IMU 中心的安装外参 (例如装在 IMU 正上方 10cm 处)
    Eigen::Vector3d tag_offset(0.0, 0.0, 0.12);
    
    // 状态机标志位
    bool is_eskf_running = false;
    bool send_once = false;
    // --- C. 启动后台独立线程 ---
    // std::thread imu_thread(ImuProducerThread, &imu_sensor);
    std::cout << "[系统] 核心融合调度器启动..." << std::endl;

    // --- D. 核心主循环 (消费者) ---
    while (true) 
    {
        if (!uds_client.isConnected()) {
            uds_client.connectToServer("/tmp/robot_hw.sock");
        }
        // ----------------------------------------------------
        // 环节 1：高频处理 IMU 队列
        // ----------------------------------------------------
        InitStatus current_state = initializer.getStatus();
        size_t head, tail;
        if (g_imu_buffer.getHistoryWindow(head, tail)) 
        {
            for (size_t i = tail; i != head; i = (i + 1) & 1023) 
            {
                const ImuData& imu_data = g_imu_buffer.getNode(i);
                
                // 安检第一关：静止对齐
                /*  到时候可以下位机发送开始校准，
                *** 然后静态校准，静态完成后发送开始动态校准，
                *** 这个也完成后发送成功，最后让下位机给启动进程开始运行图像处理，
                *** 然后启动
                */
               switch (current_state)
               {
               case InitStatus::UNINITIALIZED:
                // 静态校准
                    {
                        if (!send_once)
                        {
                            // UartSender::instance().mode_flag = 1;
                            // UartSender::instance().send(0x05,1);            //给下位机发送开始校准距离
                            send_once = true;
                        }
                        
                        initializer.processStaticImu(imu_data);         
  
                        break;
                    }
                
               case InitStatus::STATIC_INITIALIZED: 
                    {
                        // 静态校准完成，等待动态找北。
                        // 此时可以维持 ESKF 的基础姿态预测（仅利用陀螺仪），但不融合位置信息
                        if (!is_eskf_running) 
                        {
                            eskf_manager.setInitialState(initializer.getInitialState());
                            is_eskf_running = true;
                        
                            // 发送起步信号 (注意：真实工程中 receive() 应设为非阻塞)
                            std::cout << "[提示] 请驾驶小车起步，速度 > 0.5m/s 以激活航向锁定！" << std::endl;
                            // UartSender::instance().mode_flag = 2;
                            // UartSender::instance().send(0x05,1);
                        } 
                        // 维持高频姿态推算
                        eskf_manager.processImu(imu_data);
                        break;
                    }
                    case InitStatus::FULLY_INITIALIZED: 
                        {
                            eskf_manager.processImu(imu_data);
                        }

               default:
                    break;
               }
            }
            g_imu_buffer.advanceTail(head); // 释放缓冲区空间
        }
        // ----------------------------------------------------
        // 🌟 环节 1.5：高频处理底层编码器与 NHC 约束
        // ----------------------------------------------------
        EncoderPacket enc_data;
        bool has_new_odom = false;

        // 瞬间榨干操作系统缓冲区，只留最新的一帧
        while (uds_client.receiveData(enc_data)) 
        {
            has_new_odom = true;
        }


        if (is_eskf_running && has_new_odom) 
        {
            double all_enc = (enc_data.v_left + enc_data.v_right)/2.0;
            double v_speed = (all_enc * METERS_PER_TICK)/MCU_DT;
            eskf_manager.processOdom(v_speed, enc_data.timestamp);
            // std::cout << std::fixed << std::setprecision(6) // 固定小数点显示 6 位
            // << "left: " << v_left_ms 
            // << " | right: " << v_right_ms 
            // << std::endl;
        }
        // ----------------------------------------------------
        // 环节 2：低频处理 UWB 修正与找北
        // ----------------------------------------------------
        std::vector<UwbMeasurement, Eigen::aligned_allocator<UwbMeasurement>> uwb_meas_list;
        if (uwb_sensor.fetchLatestMeasurements(uwb_meas_list) && is_eskf_running) 
        {
            
            // 取这一批 UWB 数据的公共时间戳 (或者取最后一个)
            double rx_timestamp = uwb_meas_list.empty() ? 0.0 : uwb_meas_list.back().timestamp;
            switch (current_state)
            {
            case InitStatus::UNINITIALIZED:
                {
                break;
                }
            case InitStatus::STATIC_INITIALIZED: 
                {
                    Eigen::Vector3d rough_pos = solveRoughUwbPosition(uwb_meas_list,
                                                                    global_params.params_uwb.anchor_height,
                                                                    global_params.params_uwb.tag_height);
                    State current_state_ref = eskf_manager.getState();
                    bool yaw_locked = initializer.processDynamicUwb(rough_pos, rx_timestamp, &current_state_ref);
                    if (yaw_locked) 
                    {
                        // 强行把锁定好车头的状态写回 ESKF
                        eskf_manager.setInitialState(current_state_ref);
                        
                        std::cout << "[初始化成功] ESKF 原点已对齐至 UWB 坐标: [" 
                                  << rough_pos.transpose() << "]" << std::endl;
                        // UartSender::instance().mode_flag = 3;
                        // UartSender::instance().send(0x05,1);        // 发送所有就绪信号
                    }
                    break;
                }
            case InitStatus::FULLY_INITIALIZED: 
                {
                    // 1. 利用刚写到的状态，利用 UWB 测量修正状态
                    for (const auto& meas : uwb_meas_list) 
                    {
                        eskf_manager.processUwb(meas.distance, meas.anchor, meas.timestamp, tag_offset);
                    }
                    break;
                }
            default:
                break;
            }
            
        }

        // ----------------------------------------------------
        // 环节 3：系统状态监控与输出
        // ----------------------------------------------------
        if (initializer.getStatus() == InitStatus::FULLY_INITIALIZED) 
        {
            State final_state = eskf_manager.getState();


            Eigen::Matrix3d R = final_state.pose_.block<3,3>(0,0);


            double yaw_rad = std::atan2(R(1,0), R(0,0));


            double yaw_deg = yaw_rad * 180.0 / M_PI - 90;



            udp_sender.sendPose( final_state.pose_(0,3), final_state.pose_(1,3), yaw_deg);
            static int print_counter = 0;
            if (print_counter++ % 100 == 0) 
            { // 降频打印
                State final_state = eskf_manager.getState();
                std::cout << "[定位输出] X: " << final_state.pose_(0,3) 
                          << "m, Y: " << final_state.pose_(1,3) 
                          << "m | 在线估计延迟: " << final_state.t_delay_ * 1000.0 << " ms" 
                          << std::endl;
            }
        }

        // 让出主线程切片，防止占用 100% CPU
        usleep(1000); 
    }

    // 清理资源
    uwb_sensor.stop();
    return 0;
}