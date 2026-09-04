#ifndef UART_HPP
#define UART_HPP

#include <cstdint>
#include <cerrno>
#include <array>
#include <atomic>

// 根据本进程已发送的串口指令推断的下位机运动相关状态（用于画面 OSD）
struct CarMotionHudState {
    uint8_t cmd02_mode = 0;   // 0x02 末次值：0 正常 1 停车 2 快速 3 减速 4 金币赛道外减速 5 兜底返回赛道减速 6 金币边界带减速 7 快速回到赛道
    bool    start_car_sent = false; // 曾发送 0x05,1 发车（锁存）
    bool    speed_limit_on = false; // 0x08：末次值 <=12 为限速模式，>12 解除
    float   speed_limit_value = 0.f;
    uint8_t cmd03_protect = 0;      // 0x03 末次值（保护停车等）
};

struct UartSendStats {
    uint64_t attempts = 0;
    uint64_t failures = 0;
    uint64_t suppressed = 0;
    std::array<uint64_t, 256> failures_by_cmd{};
};

union Param1 {
    float data_1;     // cmd 0x01: error值
    uint8_t data_2;   // cmd 0x02: 速度控制标志
    uint8_t data_3;   // cmd 0x03: 保护标志
    uint8_t data_4;   // cmd 0x04: 蜂鸣器
    uint8_t data_5;     // cmd 0x05: 发车指令
    uint8_t data_6;     // cmd 0x06: 遥控控制
    uint8_t data_7;     // cmd 0x07: 转弯标志
    float data_8;     // cmd 0x08: 最大速度
    uint8_t data_9;   // cmd 0x09: 顶层 DriveState 标志，有效值 1..9
    uint8_t data_11;  // cmd 0x0B: sign方向决策 0 none 1 left 2 right
    uint8_t bytes[4];
};

union Param2 {
    int16_t data_1;
    int16_t data_2;
    float data_3;
    uint8_t data_4;
    float   data_5;
    float   data_6;
    uint8_t data_7;
    float   data_8;
    uint8_t data_9;
    uint8_t bytes[4];
};

class Uart {
public:
    static Uart& instance();

    bool open(const char* device = "/dev/my_tc264");
    bool send(uint8_t cmd, uint8_t length, uint8_t value);
    bool send(uint8_t cmd, uint8_t length, int value);
    bool send(uint8_t cmd, uint8_t length, float value);
    void setTransmitEnabled(bool enabled);
    bool transmitEnabled() const;
    UartSendStats sendStatsSnapshot() const;
    void receive();

    // 运动状态 HUD（由 send() 更新，供 main 绘制）
    // 反映当前待发/已组帧的指令变量（与 write 成败无关，供画面 OSD）
    CarMotionHudState motionHudSnapshot() const;
    bool wait_and_receive(int timeout_ms = 10);
    uint16_t crc16(uint8_t* data, uint8_t length);

    // -------- 发送数据变量（可配置） --------
    float uart_error = 0.0f;   // cmd 0x01: error
    uint8_t flag_stop = 0;     // cmd 0x02: 暂时停车标志（默认 NORMAL）
    uint8_t flag_protect = 0;  // cmd 0x03: 保护标志
    uint8_t flag_buzzer = 1;   // cmd 0x04: 蜂鸣器
    uint8_t flag_start_car = 1; // cmd 0x05: 发车指令
    uint8_t key_control = 1;    // cmd 0x06: 键盘控制
    uint8_t turn_flag = 0; // cmd 0x07: 转弯标志
    float max_speed = 10.0f;    // cmd 0x08: 最大速度
    uint8_t state_flag = 1;   // cmd 0x09: 顶层 DriveState 标志，1..9
    uint8_t sign_direction = 0; // cmd 0x0B: sign方向决策 0 none 1 left 2 right

    int fd = -1;
    int param1_longest_byte = 4;
    int param1_send_all_byte = 9;

    // -------- 接收数据变量 --------
    int16_t encoder_left = 0; //左编码器
    int16_t encoder_right = 0; //右编码器
    uint32_t encoder_sample_seq = 0; // 收到成对 0x01+0x02 后递增，供 ODOM 去重
    int16_t left_speed_recv = 0;  // 左轮速度接收
    int16_t right_speed_recv = 0; // 右轮速度接收
    float yaw = 0;  // yaw
    uint8_t flag_right_speed = 0; // 右轮速度标志

    float error = 10.0f;
    uint8_t flag_stop_recv = 0;
    float speed_val_recv = 0.0f;
    float angle_val_recv = 0.0f;
    float distance_val_recv = 0.0f;
    uint8_t turn_flag_recv = 0;
    float max_speed_recv = 0.0f;
    uint8_t state_flag_recv = 0;

private:
    Uart() = default;
    ~Uart();
    Uart(const Uart&) = delete;
    Uart& operator=(const Uart&) = delete;

    void process_received_data(const uint8_t* data, uint8_t length);
    bool send_raw(uint8_t cmd, uint8_t length);
    void recordSendFailure(uint8_t cmd);
    void updateMotionHudFromSend(uint8_t cmd, uint8_t uval, float fval, bool is_float_param);

    CarMotionHudState motion_hud_{};
    std::atomic<bool> tx_enabled_{true};
    std::atomic<uint64_t> send_attempts_{0};
    std::atomic<uint64_t> send_failures_{0};
    std::atomic<uint64_t> send_suppressed_{0};
    std::array<std::atomic<uint64_t>, 256> send_failures_by_cmd_{};

    static constexpr uint8_t RX_BUFFER_SIZE = 64;
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    uint16_t rx_index = 0;
};

#endif
