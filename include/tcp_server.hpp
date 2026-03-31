#pragma once

#include <spdlog/spdlog.h>

#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <functional>
#include <mutex>
#include <netinet/in.h>
#include <string>
#include <thread>
#include <unistd.h>

#include "sys/socket.h"

struct tcp_server {
    using ReceiveHandler = std::function<void(const std::string&)>;

    int listen_fd{-1};
    int client_fd{-1};
    sockaddr_in server_addr{};
    std::atomic<bool> running{false};
    std::thread io_thread;
    std::mutex client_mutex;
    ReceiveHandler on_receive;

    explicit tcp_server(int port, const std::string& bind_ip = "0.0.0.0") {
        listen_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd < 0) {
            spdlog::error("socket creation failed: {}", strerror(errno));
            exit(EXIT_FAILURE);
        }

        int reuse = 1;
        if (setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
            spdlog::error("failed to set SO_REUSEADDR: {}", strerror(errno));
            close(listen_fd);
            listen_fd = -1;
            exit(EXIT_FAILURE);
        }

        std::memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        if (bind_ip.empty() || bind_ip == "0.0.0.0") {
            server_addr.sin_addr.s_addr = INADDR_ANY;
        } else if (inet_pton(AF_INET, bind_ip.c_str(), &server_addr.sin_addr) != 1) {
            spdlog::error("invalid bind ip: {}", bind_ip);
            close(listen_fd);
            listen_fd = -1;
            exit(EXIT_FAILURE);
        }
        server_addr.sin_port = htons(port);

        if (bind(listen_fd, reinterpret_cast<const sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
            spdlog::error("bind failed: {}", strerror(errno));
            close(listen_fd);
            listen_fd = -1;
            exit(EXIT_FAILURE);
        }

        if (listen(listen_fd, 1) < 0) {
            spdlog::error("listen failed: {}", strerror(errno));
            close(listen_fd);
            listen_fd = -1;
            exit(EXIT_FAILURE);
        }

        spdlog::info("TCP server initialized on {}:{}", bind_ip.empty() ? "0.0.0.0" : bind_ip, port);
    }

    ~tcp_server() {
        stop();
        close_listen_fd();
    }

    void start() {
        if (running.load()) {
            return;
        }
        running.store(true);
        io_thread = std::thread(&tcp_server::io_loop, this);
        spdlog::info("TCP IO thread started");
    }

    void stop() {
        if (!running.load()) {
            return;
        }
        running.store(false);

        close_client_fd();
        close_listen_fd();

        if (io_thread.joinable()) {
            io_thread.join();
        }
        spdlog::info("TCP IO thread stopped");
    }

    bool send(const std::string& data) {
        std::lock_guard<std::mutex> lock(client_mutex);
        if (client_fd < 0) {
            return false;
        }

        size_t total_sent = 0;
        while (total_sent < data.size()) {
            const ssize_t sent = ::send(client_fd, data.data() + total_sent, data.size() - total_sent, 0);
            if (sent <= 0) {
                if (errno == EINTR) {
                    continue;
                }
                spdlog::warn("send failed: {}", strerror(errno));
                close_client_fd_locked();
                return false;
            }
            total_sent += static_cast<size_t>(sent);
        }
        return true;
    }

    bool send(const void* data, size_t size) {
        return send(std::string(reinterpret_cast<const char*>(data), size));
    }

private:
    void io_loop() {
        while (running.load()) {
            int local_client = -1;
            {
                std::lock_guard<std::mutex> lock(client_mutex);
                local_client = client_fd;
            }

            if (local_client < 0) {
                wait_for_client();
            } else {
                read_from_client(local_client);
            }
        }
    }

    void wait_for_client() {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(listen_fd, &readfds);

        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 100000;

        const int ready = select(listen_fd + 1, &readfds, nullptr, nullptr, &tv);
        if (ready <= 0) {
            return;
        }

        if (FD_ISSET(listen_fd, &readfds)) {
            sockaddr_in client_addr{};
            socklen_t client_len = sizeof(client_addr);
            int accepted_fd = accept(listen_fd, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
            if (accepted_fd < 0) {
                if (errno != EINTR) {
                    spdlog::warn("accept failed: {}", strerror(errno));
                }
                return;
            }

            char ip[INET_ADDRSTRLEN] = {0};
            inet_ntop(AF_INET, &client_addr.sin_addr, ip, sizeof(ip));
            spdlog::info("Client connected: {}:{}", ip, ntohs(client_addr.sin_port));

            std::lock_guard<std::mutex> lock(client_mutex);
            if (client_fd >= 0) {
                close_client_fd_locked();
            }
            client_fd = accepted_fd;
        }
    }

    void read_from_client(int fd) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);

        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 100000;

        const int ready = select(fd + 1, &readfds, nullptr, nullptr, &tv);
        if (ready <= 0) {
            return;
        }

        if (FD_ISSET(fd, &readfds)) {
            char buffer[2048];
            const ssize_t len = recv(fd, buffer, sizeof(buffer), 0);
            if (len > 0) {
                if (on_receive) {
                    on_receive(std::string(buffer, static_cast<size_t>(len)));
                }
                return;
            }

            if (len == 0) {
                spdlog::info("Client disconnected");
            } else if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
                spdlog::warn("recv failed: {}", strerror(errno));
            }

            std::lock_guard<std::mutex> lock(client_mutex);
            close_client_fd_locked();
        }
    }

    void close_client_fd() {
        std::lock_guard<std::mutex> lock(client_mutex);
        close_client_fd_locked();
    }

    void close_client_fd_locked() {
        if (client_fd >= 0) {
            shutdown(client_fd, SHUT_RDWR);
            close(client_fd);
            client_fd = -1;
        }
    }

    void close_listen_fd() {
        if (listen_fd >= 0) {
            shutdown(listen_fd, SHUT_RDWR);
            close(listen_fd);
            listen_fd = -1;
        }
    }
};
