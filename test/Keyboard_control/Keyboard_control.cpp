#include <iostream>
#include <termios.h>
#include <unistd.h>
#include "uart.hpp"

// 设置终端为无回车、实时按键模式
void setTerminalMode() {
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
}

// 恢复终端正常模式
void resetTerminalMode() {
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
}

int main() {
    setTerminalMode();

    std::cout << "======== 按键控制 ========\n";
    std::cout << "w → 发送 1\n";
    std::cout << "s → 发送 0\n";
    std::cout << "a → 发送 2\n";
    std::cout << "d → 发送 3\n";
    std::cout << "q → 退出\n";
    std::cout << "=========================\n";

    char key;
    while (true) {
        key = getchar();

        switch (key) {
            case 'w':
                std::cout << "发送: 1\n";
                Uart::instance().send(0x06, 1, 1);  // 直走

                break;
            case 's':
                std::cout << "发送: 0\n";
                Uart::instance().send(0x06, 1, 0);  // 停止
                break;
            case 'a':
                std::cout << "发送: 2\n";
                Uart::instance().send(0x06, 1, 2);  // 左转
                break;
            case 'd':
                std::cout << "发送: 3\n";
                Uart::instance().send(0x06, 1, 3);  // 右转
                break;
            case 'q':
                std::cout << "\n退出程序\n";
                goto exit_loop;
            default:
                break;
        }
    }

exit_loop:
    resetTerminalMode();
    return 0;
}