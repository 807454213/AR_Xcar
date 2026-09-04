#pragma once

#include <cstdint>

union Param1 {
    float data_1;     // cmd 0x01: 错误值
    uint8_t data_2;   // cmd 0x02: 停止标志
    uint8_t data_3;   // cmd 0x03: 保护标志
    uint8_t data_4;   // cmd 0x04: 模式选择
    uint8_t data_5;     // cmd 0x05: 速度值
    float data_6;     // cmd 0x06: 角度值
    float data_7;     // cmd 0x07: 距离值
    float data_8;     // cmd 0x08: 功率值
    uint8_t data_9;   // cmd 0x09: 使能标志
    uint8_t bytes[4];
};

union Param2 {
    float data_1;
    uint8_t data_2;
    uint8_t data_3;
    uint8_t data_4;
    float data_5;
    float data_6;
    float data_7;
    float data_8;
    uint8_t data_9;
    uint8_t bytes[4];
};

class UartSender {
public:
    static UartSender& instance();

    bool open(const char* device = "/dev/ttyUSB1");
    void send(uint8_t cmd, uint8_t length);
    void receive();
    uint16_t crc16(uint8_t* data, uint8_t length);

    // -------- 发送数据变量（可配置） --------
    float uart_error = 10.0f;   // cmd 0x01: 错误值
    uint8_t flag_stop = 0;     // cmd 0x02: 停止标志
    uint8_t flag_protect = 0;  // cmd 0x03: 保护标志
    uint8_t flag_buzzer = 1;   // cmd 0x04: 模式选择  校验模式 退出发数据ok
    uint8_t mode_flag = 0;     // cmd 0x05: 模式选择
    float angle_val = 0.0f;    // cmd 0x06: 角度值
    float distance_val = 0.0f; // cmd 0x07: 距离值
    float power_val = 0.0f;    // cmd 0x08: 功率值
    uint8_t enable_flag = 0;   // cmd 0x09: 使能标志

    int fd = -1;
    int param1_longest_byte = 4;
    int param1_send_all_byte = 9;

    // -------- 接收数据变量 --------
    float error = 10.0f;
    uint8_t flag_stop_recv = 0;
    int16_t flag_left_speed = 0;
    int16_t flag_right_speed = 0;
    int16_t speed_val_recv = 0;
    float angle_val_recv = 0.0f;
    float distance_val_recv = 0.0f;
    float power_val_recv = 0.0f;
    uint8_t enable_flag_recv = 0;

private:
    UartSender() = default;
    ~UartSender();
    UartSender(const UartSender&) = delete;
    UartSender& operator=(const UartSender&) = delete;

    void process_received_data(const uint8_t* data, uint8_t length);

    static constexpr uint8_t RX_BUFFER_SIZE = 64;
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    uint16_t rx_index = 0;
};


