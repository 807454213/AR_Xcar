#include "uart.hpp"
#include "../include/odom_hw.h"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <cerrno>
#include <poll.h>

namespace {

bool writeAllNonblock(int fd, const uint8_t* buf, size_t len,
                      bool wait_for_drain)
{
    size_t off = 0;
    int stall_polls = 0;
    constexpr int kMaxStallPolls = 50;  // 50 × 10ms ≈ 500ms

    while (off < len) {
        ssize_t n = ::write(fd, buf + off, len - off);
        if (n > 0) {
            off += static_cast<size_t>(n);
            stall_polls = 0;
            continue;
        }
        if (n == 0)
            return false;

        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            if (!wait_for_drain)
                return false;
            pollfd pfd{};
            pfd.fd = fd;
            pfd.events = POLLOUT;
            const int pr = ::poll(&pfd, 1, 10);
            if (pr < 0)
                return false;
            if (pr == 0) {
                if (++stall_polls > kMaxStallPolls)
                    return false;
                continue;
            }
            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
                return false;
            continue;
        }
        return false;
    }
    return true;
}

}  // namespace

Uart& Uart::instance() {
    static Uart inst;
    return inst;
}

Uart::~Uart() {
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }
}

bool Uart::open(const char* device) {
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }

    fd = ::open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      //  std::cerr << "Fail to open " << device << std::endl;
        return false;
    }

    termios opt{};
    tcflush(fd, TCIOFLUSH);
    tcgetattr(fd, &opt);

    cfsetospeed(&opt, B460800);
    cfsetispeed(&opt, B460800);

    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;
    opt.c_cflag &= ~PARENB;
    opt.c_iflag &= ~INPCK;
    opt.c_cflag &= ~CSTOPB;

    opt.c_oflag &= ~OPOST;
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    tcsetattr(fd, TCSANOW, &opt);
    //std::cout << "UART opened: " << device << std::endl;
    return true;
}

uint16_t Uart::crc16(uint8_t* data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

CarMotionHudState Uart::motionHudSnapshot() const
{
    CarMotionHudState s = motion_hud_;
    s.cmd02_mode = flag_stop;
    s.cmd03_protect = flag_protect;
    if (max_speed > 12.0f) {
        s.speed_limit_on = false;
    } else if (max_speed > 0.0f) {
        s.speed_limit_on = true;
        s.speed_limit_value = max_speed;
    }
    return s;
}

void Uart::setTransmitEnabled(bool enabled)
{
    tx_enabled_.store(enabled, std::memory_order_relaxed);
}

bool Uart::transmitEnabled() const
{
    return tx_enabled_.load(std::memory_order_relaxed);
}

UartSendStats Uart::sendStatsSnapshot() const
{
    UartSendStats s;
    s.attempts = send_attempts_.load(std::memory_order_relaxed);
    s.failures = send_failures_.load(std::memory_order_relaxed);
    s.suppressed = send_suppressed_.load(std::memory_order_relaxed);
    for (size_t i = 0; i < s.failures_by_cmd.size(); ++i) {
        s.failures_by_cmd[i] = send_failures_by_cmd_[i].load(std::memory_order_relaxed);
    }
    return s;
}

void Uart::recordSendFailure(uint8_t cmd)
{
    send_failures_.fetch_add(1, std::memory_order_relaxed);
    send_failures_by_cmd_[cmd].fetch_add(1, std::memory_order_relaxed);
}

void Uart::updateMotionHudFromSend(uint8_t cmd, uint8_t uval, float fval, bool is_float_param)
{
    // 0x01 仅 error，不参与运动状态
    if (cmd == 0x02) {
        motion_hud_.cmd02_mode = uval;
    } else if (cmd == 0x03) {
        motion_hud_.cmd03_protect = uval;
    } else if (cmd == 0x05) {
        if (uval == 1)
            motion_hud_.start_car_sent = true;
    } else if (cmd == 0x08 && is_float_param) {
        if (fval > 12.0f) {
            motion_hud_.speed_limit_on = false;
        } else {
            motion_hud_.speed_limit_on = true;
            motion_hud_.speed_limit_value = fval;
        }
    }
}

bool Uart::send_raw(uint8_t cmd, uint8_t length) {
    send_attempts_.fetch_add(1, std::memory_order_relaxed);
    if (!tx_enabled_.load(std::memory_order_relaxed)) {
        send_suppressed_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    if (fd < 0) {
        if (!open()) {
            recordSendFailure(cmd);
            return false;
        }
    }

    Param1 all_data{};
    all_data.data_1 = 0;
    uint8_t datasend[9];

    datasend[0] = 0x55;
    datasend[1] = cmd;
    datasend[2] = static_cast<uint8_t>(param1_longest_byte);

    if (cmd == 0x01) {
        all_data.data_1 = uart_error;//error
    } else if (cmd == 0x02) {
        all_data.data_2 = flag_stop; //暂时停车
    } else if (cmd == 0x03) {
        all_data.data_3 = flag_protect; //保护，停车
    } else if (cmd == 0x04) {
        all_data.data_4 = flag_buzzer;//蜂鸣器
    } else if (cmd == 0x05) {
        all_data.data_5 = flag_start_car; //发车指令
    } else if (cmd == 0x06) {
        all_data.data_6 = key_control;
    } else if (cmd == 0x07) {
        all_data.data_7 = turn_flag;
    } else if (cmd == 0x08) {
        all_data.data_8 = max_speed;
    } else if (cmd == 0x09) {
        all_data.data_9 = state_flag;
    } else if (cmd == 0x0B) {
        all_data.data_11 = sign_direction;
    }

    for (uint8_t i = 0; i < static_cast<uint8_t>(param1_longest_byte); i++) {
        datasend[i + 3] = all_data.bytes[i];
    }

    uint16_t crc = crc16(datasend, static_cast<uint8_t>(param1_longest_byte + 3));
    datasend[param1_send_all_byte - 2] = (crc >> 8) & 0xFF;
    datasend[param1_send_all_byte - 1] = crc & 0xFF;

    const size_t frame_len = static_cast<size_t>(param1_send_all_byte);
    const bool wait_for_drain = cmd != 0x01;
    if (!writeAllNonblock(fd, datasend, frame_len, wait_for_drain)) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            ::close(fd);
            fd = -1;
        }
        recordSendFailure(cmd);
        return false;
    }
    return true;
}

bool Uart::send(uint8_t cmd, uint8_t length, uint8_t value) {
    switch (cmd) {
        case 0x02: flag_stop = value; break;
        case 0x03: flag_protect = value; break;
        case 0x04: flag_buzzer = value; break;
        case 0x05: flag_start_car = value; break;
        case 0x06: key_control = value; break;
        case 0x07: turn_flag = value; break;
        case 0x09: state_flag = value; break;
        case 0x0B: sign_direction = value; break;
        default:
            send_attempts_.fetch_add(1, std::memory_order_relaxed);
            recordSendFailure(cmd);
            return false;
    }
    updateMotionHudFromSend(cmd, value, 0.f, false);
    return send_raw(cmd, length);
}

bool Uart::send(uint8_t cmd, uint8_t length, int value) {
    return send(cmd, length, static_cast<uint8_t>(value));
}

bool Uart::send(uint8_t cmd, uint8_t length, float value) {
    switch (cmd) {
        case 0x01: uart_error = value; break;
        case 0x08: max_speed = value; break;
        default:
            send_attempts_.fetch_add(1, std::memory_order_relaxed);
            recordSendFailure(cmd);
            return false;
    }
    if (cmd == 0x08)
        updateMotionHudFromSend(cmd, 0, value, true);
    return send_raw(cmd, length);
}

void Uart::process_received_data(const uint8_t* data, uint8_t length) {
    if (data[2] == 0 || data[2] > static_cast<uint8_t>(param1_longest_byte)) {
        return;
    }

    uint16_t crc_calc = crc16(const_cast<uint8_t*>(data), static_cast<uint8_t>(data[2] + 3));
    uint16_t crc_recv = (data[length - 2] << 8) | data[length - 1];

    if (crc_calc == crc_recv) {
        Param2 all_data{};
        uint8_t cmd = data[1];
        for (int i = 0; i < param1_longest_byte; i++) {
            all_data.bytes[i] = data[i + 3];
        }

        static bool enc_left_fresh = false;

        if (cmd == 0x01) {
            encoder_left = all_data.data_1;
            odomOnUartLeftTicks(all_data.data_1);
            enc_left_fresh = true;
        } else if (cmd == 0x02) {
            encoder_right = all_data.data_2;
            odomOnUartRightTicks(all_data.data_2);
            if (enc_left_fresh) {
                encoder_sample_seq++;
                enc_left_fresh = false;
            }
        } else if (cmd == 0x03) {
            yaw = all_data.data_3;
        } else if (cmd == 0x04) {
            flag_right_speed = all_data.data_4;
        } else if (cmd == 0x05) {
            speed_val_recv = all_data.data_5;
        } else if (cmd == 0x06) {
            angle_val_recv = all_data.data_6;
        } else if (cmd == 0x07) {
            turn_flag_recv = all_data.data_7;
        } else if (cmd == 0x08) {
            max_speed_recv = all_data.data_8;
        } else if (cmd == 0x09) {
            state_flag_recv = all_data.data_9;
        }
    }
}

void Uart::receive() {
    if(fd<0)    return;

    uint8_t chunk[32];
    const int bytes_read = ::read(fd, chunk, sizeof(chunk));
    if (bytes_read <= 0) {
        if(bytes_read<0&&errno!=EAGAIN&&errno !=EWOULDBLOCK){
            ::close(fd);
            fd = -1;
        }
        return;
    }

    for (int n = 0; n < bytes_read; ++n) {
        const uint8_t data = chunk[n];

        if (rx_index == 0 && data != 0x55) continue;

        rx_buffer[rx_index++] = data;

        if (rx_index >= 9) {
            process_received_data(rx_buffer, rx_index);
            rx_index = 0;
        }
    }
}

bool Uart::wait_and_receive(int timeout_ms) {
    if (fd < 0) return false;

    pollfd pfd{};
    pfd.fd = fd;
    pfd.events = POLLIN;

    int ret = ::poll(&pfd, 1, timeout_ms);
    if (ret <= 0) {
        return false;
    }

    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        ::close(fd);
        fd = -1;
        return false;
    }

    if (pfd.revents & POLLIN) {
        // 触发后尽量读空，减少下次事件延迟
        while (fd >= 0) {
            uint16_t last_idx = rx_index;
            receive();
            if (fd < 0 || rx_index == last_idx) {
                break;
            }
        }
        return true;
    }

    return false;
}
