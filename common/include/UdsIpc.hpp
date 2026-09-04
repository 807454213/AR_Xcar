#pragma once

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>
#include <iostream>
#include <cstring>
#include <cerrno>

/**
 * @brief 极速 UDS 进程间通信库 (基于 SOCK_SEQPACKET)
 * 专为 200Hz+ 高频机器人状态机与传感器数据流设计
 */
namespace RobotIPC {

class UdsServer {
private:
    int server_fd_ = -1;
    int client_fd_ = -1;
    std::string socket_path_;

    // 设置文件描述符为非阻塞模式
    void setNonBlocking(int fd) {
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }

public:
    UdsServer() = default;
    ~UdsServer() { stop(); }

    /**
     * @brief 启动服务端监听
     * @param path 本地 Socket 挂载路径 (例如 "/tmp/robot_hw.sock")
     */
    bool start(const std::string& path) {
        socket_path_ = path;
        
        // 1. 创建 SEQPACKET 类型的 Unix Socket
        server_fd_ = socket(AF_UNIX, SOCK_SEQPACKET, 0);
        if (server_fd_ < 0)
            return false;

        // 2. 清除历史残留文件，防止 Bind 失败
        (void)unlink(socket_path_.c_str());

        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

        // 3. 绑定与监听
        if (bind(server_fd_, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
            close(server_fd_);
            server_fd_ = -1;
            return false;
        }

        if (listen(server_fd_, 5) == -1)
            return false;

        setNonBlocking(server_fd_);
        return true;
    }

    /**
     * @brief 非阻塞接收客户端连接
     */
    bool acceptClient() {
        if (client_fd_ != -1) return true; // 已经连接

        struct sockaddr_un client_addr;
        socklen_t client_len = sizeof(client_addr);
        int new_fd = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);
        
        if (new_fd >= 0) {
            client_fd_ = new_fd;
            setNonBlocking(client_fd_);
            return true;
        }
        return false;
    }

    /**
     * @brief 模板方法：极速发送任意结构体 (零序列化开销)
     */
    template <typename T>
    bool sendData(const T& data) {
        if (client_fd_ == -1) return false;
        
        // MSG_NOSIGNAL 防止对方意外断开时引发 SIGPIPE 导致主进程崩溃
        ssize_t sent = send(client_fd_, &data, sizeof(T), MSG_NOSIGNAL);
        if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            // 对方已断开连接，安全重置
            close(client_fd_);
            client_fd_ = -1;
            return false;
        }
        return sent == sizeof(T);
    }

    /**
     * @brief 模板方法：非阻塞接收任意结构体
     */
    template <typename T>
    bool receiveData(T& data) {
        if (client_fd_ == -1) return false;

        ssize_t bytes_read = recv(client_fd_, &data, sizeof(T), MSG_DONTWAIT);
        if (bytes_read == 0 || (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
            // 返回 0 代表客户端断开连接
            close(client_fd_);
            client_fd_ = -1;
            return false;
        }
        return bytes_read == sizeof(T);
    }

    void stop() {
        if (client_fd_ != -1) close(client_fd_);
        if (server_fd_ != -1) close(server_fd_);
        unlink(socket_path_.c_str());
        client_fd_ = -1; server_fd_ = -1;
    }
};

class UdsClient {
private:
    int socket_fd_ = -1;
    std::string server_path_;

    void setNonBlocking(int fd) {
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }

public:
    UdsClient() = default;
    ~UdsClient() { disconnect(); }

    /**
     * @brief 连接到硬件代理服务端
     */
    bool connectToServer(const std::string& path) {
        if (socket_fd_ != -1) return true;

        socket_fd_ = socket(AF_UNIX, SOCK_SEQPACKET, 0);
        if (socket_fd_ < 0) return false;

        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);

        if (connect(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        setNonBlocking(socket_fd_);
        return true;
    }

    template <typename T>
    bool sendData(const T& data) {
        if (socket_fd_ == -1) return false;
        
        ssize_t sent = send(socket_fd_, &data, sizeof(T), MSG_NOSIGNAL);
        if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            disconnect();
            return false;
        }
        return sent == sizeof(T);
    }

    template <typename T>
    bool receiveData(T& data) {
        if (socket_fd_ == -1) return false;

        ssize_t bytes_read = recv(socket_fd_, &data, sizeof(T), MSG_DONTWAIT);
        if (bytes_read == 0 || (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
            disconnect();
            return false;
        }
        return bytes_read == sizeof(T);
    }

    void disconnect() {
        if (socket_fd_ != -1) {
            close(socket_fd_);
            socket_fd_ = -1;
        }
    }
    
    bool isConnected() const { return socket_fd_ != -1; }
};

} // namespace RobotIPC
