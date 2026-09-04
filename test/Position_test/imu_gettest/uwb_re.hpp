#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/time.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <nlohmann/json.hpp>

// ==========================================
// 1. 观测数据传输对象 (DTO)
// ==========================================
struct UwbMeasurement 
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;         // 绝对系统单调时间 (秒)
    Eigen::Vector3d anchor;   // 该测距对应的基站绝对 3D 世界坐标
    double distance;          // 真实测距值 (米)
};

// ================================================================================
// 定义分层结构体
// ================================================================================

//=================================================================================
//@brief uwb初始化的参数
//@param origin_id 原点id  x_axis_id x轴id  y_axis_id y轴id diag_id 对轴id
//@param   field_width   场地宽度 (米)  field_length  场地长度 (米)
//@param   anchor_height  基站高度 (米) 标签高度 (米)
//@param uwb_var uwb测距的方差 (米)
//==================================================================================
struct UwbInitParams 
{
    int origin_id = 0, x_axis_id = 1, y_axis_id = 3, diag_id = 2;
    double field_width = 0.0, field_length = 0.0;
    double anchor_height = 0.0, tag_height = 0.0;
    double uwb_var;
};

//==================================================================================
//@brief 陀螺仪的基本参数
//@param acc_n 加速度计的测量噪声 (米/秒^2)
//@param gryr_n 陀螺仪的测量噪声 (弧度/秒)
//@param acc_w 加速度计的测量权重 (米/秒^2)
//@param gyr_w 陀螺仪的测量权重 (弧度/秒)
//==================================================================================
struct imu_params
{
    double acc_n=0;
    double gyr_n=0;
    double acc_w=0;
    double gyr_w=0;
};
//==================================================================================
//@brief          初始化软约束的“柔软度” (噪声方差)
//@param noise_vy 侧滑速度允许的方差 (建议 0.05 m/s)
//@param noise_vz 垂向跳动允许的方差 (建议 0.02 m/s)
//@param noise_pz 绝对高度允许的漂移方差 (建议 0.01 m)
//@param fixed_z  小车标签的绝对物理高度 (米)
//==================================================================================
struct nhc_params
{
  
    double noise_vy = 0.05;

    double noise_vz = 0.02;
    double noise_vx = 0.05;

    double noise_pz = 0.01;

    double fixed_z = 0.0;
};

//==================================================================================
//@brief
//@param
//@param
//===================================================================================
struct Parameters 
{
    UwbInitParams params_uwb;
    imu_params params_imu;
    nhc_params params_nhc;
};

class ConfigManager 
{
private:
    Parameters params;
    bool is_loaded = false;

public:
    bool loadConfig(const std::string& filepath) 
    {
        try {
            std::ifstream file(filepath);
            if (!file.is_open()) return false;

            nlohmann::json j;
            file >> j;

            auto& anchors = j.at("uwb_setup").at("anchors");
            params.params_uwb.origin_id = anchors.at("origin_id");
            params.params_uwb.x_axis_id = anchors.at("x_axis_id");
            params.params_uwb.y_axis_id = anchors.at("y_axis_id");
            params.params_uwb.diag_id   = anchors.at("diag_id");

            auto& field = j.at("uwb_setup").at("field");
            params.params_uwb.field_width  = field.at("width_m");
            params.params_uwb.field_length = field.at("length_m");

            auto& hardware = j.at("uwb_setup").at("hardware");
            params.params_uwb.anchor_height = hardware.at("anchor_height_m");
            params.params_uwb.tag_height    = hardware.at("tag_height_m");
            params.params_uwb.uwb_var       = hardware.at("uwb_var");

            auto& imu = j.at("imu_setup");
            params.params_imu.acc_n = imu.at("acc_n");
            params.params_imu.gyr_n = imu.at("gyr_n");
            params.params_imu.acc_w = imu.at("acc_w");
            params.params_imu.gyr_w = imu.at("gyr_w");

            auto& nhc = j.at("nhc_setup");
            params.params_nhc.noise_vy = nhc.at("noise_vy");
            params.params_nhc.noise_vz = nhc.at("noise_vz");
            params.params_nhc.noise_pz = nhc.at("noise_pz");
            params.params_nhc.fixed_z  = nhc.at("fixed_z");
            params.params_nhc.noise_vx = nhc.at("noise_vx");



            is_loaded = true;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[UWB Config] JSON 致命解析错误: " << e.what() << std::endl;
            return false; 
        }
    }
    bool isLoaded() const { return is_loaded; }
    const Parameters& getParams() const { return params; }
};

// ==========================================
// 3. 极速串口通信类 (防御双重释放 & 自适应波特率)
// ==========================================
class LinuxSerialPort 
{
private:
    int fd = -1;
    std::string rx_buffer;

public:
    LinuxSerialPort() = default;
    ~LinuxSerialPort() { if (fd != -1) { close(fd); fd = -1; } }
    
    // 禁用拷贝，强制使用移动语义转移句柄控制权
    LinuxSerialPort(const LinuxSerialPort&) = delete;
    LinuxSerialPort& operator=(const LinuxSerialPort&) = delete;

    LinuxSerialPort(LinuxSerialPort&& other) noexcept 
    {
        this->fd = other.fd;
        this->rx_buffer = std::move(other.rx_buffer);
        other.fd = -1; 
    }
    LinuxSerialPort& operator=(LinuxSerialPort&& other) noexcept
    {
        if (this != &other) {
            if (this->fd != -1) close(this->fd);
            this->fd = other.fd;
            this->rx_buffer = std::move(other.rx_buffer);
            other.fd = -1;
        }
        return *this;
    }

    bool openPort(const std::string& port_name, int baud_rate = 460800) 
    {
        // O_NDELAY: 开启非阻塞模式，防止底层的 read() 卡死后台线程
        fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1) return false;

        fcntl(fd, F_SETFL, 0); 
        struct termios options;
        tcgetattr(fd, &options);

        // 动态适配 Mini5 的 TTL 直连 (115200) 或 USB 直连 (460800)
        speed_t speed = B115200; 
        if (baud_rate == 460800) speed = B460800;
        else if (baud_rate == 921600) speed = B921600;

        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);

        options.c_cflag &= ~PARENB;   
        options.c_cflag &= ~CSTOPB;   
        options.c_cflag &= ~CSIZE;    
        options.c_cflag |= CS8;       
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
        options.c_oflag &= ~OPOST; 
        options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR); 
        options.c_cc[VTIME] = 0; 
        options.c_cc[VMIN]  = 0; 

        tcflush(fd, TCIOFLUSH);
        return tcsetattr(fd, TCSANOW, &options) == 0;
    }

    bool getNextFrame(std::string& frame) 
    {
        char buf[256];
        int bytes_read = read(fd, buf, sizeof(buf) - 1);
        if (bytes_read > 0) 
        {
            buf[bytes_read] = '\0';
            rx_buffer += buf;
        }
        
        // 熔断机制：拦截脏数据导致的内存泄漏
        if (rx_buffer.size() > 1024)
        {
            std::cerr << "[UART] 接收溢出，执行对齐清除！" << std::endl;
            size_t last_crlf = rx_buffer.rfind("\r\n");
        
            if (last_crlf != std::string::npos) 
            {
                // 巧妙操作：删除最后一个 \r\n 之前的所有数据
                // 剩下的可能是最新一帧的头部，或者是空的
                rx_buffer.erase(0, last_crlf + 2);
            } 
            else 
            {
                // 如果连 \r\n 都找不到，说明全是垃圾数据，彻底清空
                rx_buffer.clear();
            }
            return false;
        }
        
        size_t pos = rx_buffer.find("\r\n");
        if (pos != std::string::npos) 
        {
            frame = rx_buffer.substr(0, pos);
            rx_buffer.erase(0, pos + 2); 
            return true;
        }
        return false;
    }
};

// ==========================================
// 4. ESKF 专用多线程邮箱接收器
// ==========================================
class UwbReceiverESKF 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    ConfigManager configMgr;
    LinuxSerialPort  serialPort;
    Eigen::Vector3d anchors_pos_[4];

    // --- 多线程核心控制组件 ---
    std::thread worker_thread_;
    std::atomic<bool> is_running_{false};

    // 创建锁
    std::mutex mailbox_mtx_;
    std::vector<UwbMeasurement, Eigen::aligned_allocator<UwbMeasurement>> mailbox_data_;      //数据池
    bool has_new_mail_ = false;         //是否有新数据进入数据池
    // 抓取极速单调时间
    double getSystemTime() 
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return ts.tv_sec + ts.tv_nsec * 1e-9;
    }

    // Thread 2: 后台死磕解析线程
    void receiveLoop() 
    {
        std::string frame;
        while (is_running_.load(std::memory_order_relaxed)) 
        {
            if (serialPort.getNextFrame(frame)) 
            {
                double current_time = getSystemTime();
                unsigned int mask = 0;
                unsigned int dist_hex[4] = {0};
                
                // 极速匹配研创 Mini5 的 ASCII 协议
                int parsed = sscanf(frame.c_str(), "mc %x %x %x %x %x", 
                                    &mask, &dist_hex[0], &dist_hex[1], &dist_hex[2], &dist_hex[3]);
                
                if (parsed >= 2) 
                {
                    std::vector<UwbMeasurement, Eigen::aligned_allocator<UwbMeasurement>> temp_meas;
                    for (int i = 0; i < 4; ++i) 
                    {
                        if (mask & (1 << i))            // 通信协议里的验证数据是否可用
                        { 
                            if (parsed > i + 1) 
                            { 
                                double raw_dist_mm = dist_hex[i];
                                // YCHIOT 线性校准补偿
                                double calibrated_dist_mm = raw_dist_mm ;       //自己手动测，用excl拟合
                                double final_d_m = calibrated_dist_mm / 1000.0;
                                
                                // 拦截多径效应导致的极端飞点
                                if (final_d_m > 0.1 && final_d_m < 100.0) 
                                {
                                    UwbMeasurement meas;
                                    meas.timestamp = current_time;
                                    meas.anchor    = anchors_pos_[i]; 
                                    meas.distance  = final_d_m;
                                    temp_meas.push_back(meas);
                                }
                            }
                        }
                    }

                    // 瞬间加锁投递：保证 ESKF 主线程永远拿到最新的物理切片
                    if (!temp_meas.empty()) 
                    {
                        std::lock_guard<std::mutex> lock(mailbox_mtx_);
                        mailbox_data_ = std::move(temp_meas); 
                        has_new_mail_ = true;
                    }
                }
            } 
            else 
            {
                // 致命防线：非阻塞读取若无数据，必须让出 CPU，防止 RK3588 单核 100% 满载
                usleep(2000); // 10Hz 设备休眠 2 毫秒完全不影响实时性
            }
        }
    }

public:
    UwbReceiverESKF() = default;
    ~UwbReceiverESKF() { stop(); }

    bool init(const UwbInitParams& p, const std::string& serial_dev, int baud_rate = 460800) 
    {


        // 自动将场地 2D 坐标映射到 3D 绝对空间
        anchors_pos_[p.origin_id] = Eigen::Vector3d(0.0, 0.0, p.anchor_height);
        anchors_pos_[p.x_axis_id] = Eigen::Vector3d(p.field_width, 0.0, p.anchor_height);
        anchors_pos_[p.y_axis_id] = Eigen::Vector3d(0.0, p.field_length, p.anchor_height);
        anchors_pos_[p.diag_id]   = Eigen::Vector3d(p.field_width, p.field_length, p.anchor_height);

        if (!serialPort.openPort(serial_dev, baud_rate)) return false;

        // 点火启动独立线程
        is_running_.store(true, std::memory_order_relaxed);
        worker_thread_ = std::thread(&UwbReceiverESKF::receiveLoop, this);

        std::cout << "[UWB RE] 异步数据接入层就绪。基站 Z 轴锚定: " << p.anchor_height 
                  << "m, 波特率: " << baud_rate << std::endl;
        return true;
    }

    void stop() 
    {
        is_running_.store(false, std::memory_order_relaxed);
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

    // =========================================================
    // 算法层暴露接口 (供主循环调用)
    // =========================================================
    // 非阻塞查收：若有新信件返回 true 并通过引用传出；若为空立刻返回 false，零阻力
    bool fetchLatestMeasurements(std::vector<UwbMeasurement, Eigen::aligned_allocator<UwbMeasurement>>& out_meas) 
    {
        std::lock_guard<std::mutex> lock(mailbox_mtx_);
        if (has_new_mail_) 
        {
            out_meas = std::move(mailbox_data_); 
            has_new_mail_ = false;    // 阅后即焚
            return true;
        }
        return false;
    }
};