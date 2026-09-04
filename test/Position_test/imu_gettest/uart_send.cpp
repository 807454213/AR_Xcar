#include "uart_send.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <iostream>

UartSender& UartSender::instance() {
    static UartSender inst;
    return inst;
}

UartSender::~UartSender() {
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }
}

bool UartSender::open(const char* device) {
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }

    fd = ::open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "Fail to open " << device << std::endl;
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
    std::cout << "UART opened: " << device << std::endl;
    return true;
}

uint16_t UartSender::crc16(uint8_t* data, uint8_t length) {
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

void UartSender::send(uint8_t cmd, uint8_t length) {
    if (fd < 0) {
        std::cerr << "UART fd invalid, reopening..." << std::endl;
        if (!open()) {
            std::cerr << "ERROR: Cannot open UART" << std::endl;
            return;
        }
    }

    Param1 all_data{};
    all_data.data_1 = 0;
    uint8_t datasend[9];

    datasend[0] = 0x55;
    datasend[1] = cmd;
    datasend[2] = length;

    if (cmd == 0x01) {
        all_data.data_1 = uart_error;
    } else if (cmd == 0x02) {
        all_data.data_2 = flag_stop;
    } else if (cmd == 0x03) {
        all_data.data_3 = flag_protect;
    } else if (cmd == 0x04) {
        all_data.data_4 = flag_buzzer;
    } else if (cmd == 0x05) {
        all_data.data_5 = mode_flag;
    } else if (cmd == 0x06) {
        all_data.data_6 = angle_val;
    } else if (cmd == 0x07) {
        all_data.data_7 = distance_val;
    } else if (cmd == 0x08) {
        all_data.data_8 = power_val;
    } else if (cmd == 0x09) {
        all_data.data_9 = enable_flag;
    }

    for (uint8_t i = 0; i < length; i++) {
        datasend[i + 3] = all_data.bytes[i];
    }

    uint16_t crc = crc16(datasend, length + 3);
    datasend[param1_send_all_byte - 2] = (crc >> 8) & 0xFF;
    datasend[param1_send_all_byte - 1] = crc & 0xFF;

    int bytes_written = ::write(fd, datasend, param1_send_all_byte);
    if (bytes_written < 0) {
        std::cerr << "ERROR: write failed" << std::endl;
        return;
    }
    if (static_cast<size_t>(bytes_written) != sizeof(datasend)) {
        std::cerr << "Warning: only " << bytes_written << " bytes written" << std::endl;
    }
}

void UartSender::process_received_data(const uint8_t* data, uint8_t length) {
    uint16_t crc_calc = crc16(const_cast<uint8_t*>(data), data[2] + 3);
    uint16_t crc_recv = (data[length - 2] << 8) | data[length - 1];

    if (crc_calc == crc_recv) {
        Param2 all_data{};
        uint8_t cmd = data[1];
        for (int i = 0; i < param1_longest_byte; i++) {
            all_data.bytes[i] = data[i + 3];
        }
        if (cmd == 0x01) {
            error = all_data.data_1;
            std::cout << "error:" << error << std::endl;
        } else if (cmd == 0x02) {
            flag_stop_recv = all_data.data_2;
        } else if (cmd == 0x03) {
            flag_left_speed = all_data.data_3;
        } else if (cmd == 0x04) {
            flag_right_speed = all_data.data_4;
        } else if (cmd == 0x05) {
            speed_val_recv = all_data.data_5;
        } else if (cmd == 0x06) {
            angle_val_recv = all_data.data_6;
        } else if (cmd == 0x07) {
            distance_val_recv = all_data.data_7;
        } else if (cmd == 0x08) {
            power_val_recv = all_data.data_8;
        } else if (cmd == 0x09) {
            enable_flag_recv = all_data.data_9;
        }
    }
}

void UartSender::receive() {
    uint8_t data;
    int bytes_read;

    bytes_read = ::read(fd, &data, 1);
    if (bytes_read <= 0) return;

    if (rx_index == 0 && data != 0x55) return;

    rx_buffer[rx_index++] = data;

    if (rx_index >= 9) {
        process_received_data(rx_buffer, rx_index);
        rx_index = 0;
    } else if (rx_index >= 60) {
        std::cerr << "UART receive buffer overflow" << std::endl;
        rx_index = 0;
    }
}
