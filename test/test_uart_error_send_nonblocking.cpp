#include "uart.hpp"

#include <chrono>
#include <cerrno>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

void odomOnUartLeftTicks(int16_t) {}
void odomOnUartRightTicks(int16_t) {}

namespace {

bool fillNonblockingPipe(int fd)
{
    char buf[4096];
    for (char& ch : buf)
        ch = 0x5a;
    while (true) {
        const ssize_t n = ::write(fd, buf, sizeof(buf));
        if (n > 0)
            continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            return true;
        return false;
    }
}

}  // namespace

int main()
{
    int pipefd[2] = {-1, -1};
    if (::pipe2(pipefd, O_NONBLOCK) != 0) {
        std::cerr << "pipe2 failed\n";
        return 2;
    }

    if (!fillNonblockingPipe(pipefd[1])) {
        std::cerr << "failed to fill nonblocking pipe\n";
        ::close(pipefd[0]);
        ::close(pipefd[1]);
        return 3;
    }

    Uart& uart = Uart::instance();
    if (uart.fd >= 0)
        ::close(uart.fd);
    uart.fd = pipefd[1];
    uart.setTransmitEnabled(true);

    const auto t0 = std::chrono::steady_clock::now();
    const bool ok = uart.send(0x01, 4, 1.25f);
    const auto t1 = std::chrono::steady_clock::now();
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    uart.fd = -1;
    ::close(pipefd[0]);
    ::close(pipefd[1]);

    if (ok) {
        std::cerr << "send unexpectedly succeeded on a full nonblocking fd\n";
        return 4;
    }
    if (elapsed_ms > 50.0) {
        std::cerr << "error send blocked too long: " << elapsed_ms << "ms\n";
        return 1;
    }

    std::cout << "uart error send returned in " << elapsed_ms << "ms\n";
    return 0;
}
