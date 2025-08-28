/**
 * Copyright (c) 2025 Robert Bosch GmbH.
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#include "../include/udp_communication.hpp"

#include <sys/select.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

UdpCommunication::UdpCommunication(const std::string& address, uint16_t port, char* receiveBuffer,
                                   std::size_t receiveBufferSize, char* sendBuffer, std::size_t sendBufferSize) {
    addSocket(address, port, receiveBuffer, receiveBufferSize, sendBuffer, sendBufferSize);
}

UdpCommunication::~UdpCommunication() {
    for (auto& socketInfo : sockets_) {
        if (close(socketInfo.sockfd) < 0) {
            std::cerr << "Failed to close socket" << std::endl;
        }
    }
}

void UdpCommunication::addSocket(const std::string& address, uint16_t port, char* receiveBuffer,
                                 std::size_t receiveBufferSize, char* sendBuffer, std::size_t sendBufferSize) {
    SocketInfo socketInfo;
    socketInfo.sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketInfo.sockfd < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        throw std::runtime_error("Socket creation failed");
    }

    memset(&socketInfo.serverAddress, 0, sizeof(socketInfo.serverAddress));
    socketInfo.serverAddress.sin_family = AF_INET;
    if (inet_pton(AF_INET, address.c_str(), &socketInfo.serverAddress.sin_addr) <= 0) {
        std::cerr << "Invalid address or address not supported" << std::endl;
        close(socketInfo.sockfd);
        throw std::runtime_error("Invalid address or address not supported");
    }
    socketInfo.serverAddress.sin_port = htons(port);

    socketInfo.receiveBuffer = receiveBuffer;
    socketInfo.sendBuffer = sendBuffer;
    socketInfo.receiveBufferSize = receiveBufferSize;
    socketInfo.sendBufferSize = sendBufferSize;

    sockets_.push_back(socketInfo);

    std::cout << "Socket added, address & port set:\n"
              << inet_ntoa(socketInfo.serverAddress.sin_addr) << ":" << ntohs(socketInfo.serverAddress.sin_port)
              << std::endl;
}

bool UdpCommunication::send() {
    for (const auto& socketInfo : sockets_) {
        if (sendto(socketInfo.sockfd, socketInfo.sendBuffer, socketInfo.sendBufferSize, 0,
                   (struct sockaddr*)&socketInfo.serverAddress, sizeof(socketInfo.serverAddress)) == -1) {
            std::cerr << "Failed to send message" << std::endl;
            return false;
        }
    }
    return true;
}

bool UdpCommunication::sendToSocket(int socketIndex) {
    if (socketIndex < 0 || socketIndex >= static_cast<int>(sockets_.size())) {
        std::cerr << "Invalid socket index" << std::endl;
        return false;
    }

    const auto& socketInfo = sockets_[socketIndex];
    if (sendto(socketInfo.sockfd, socketInfo.sendBuffer, socketInfo.sendBufferSize, 0,
               (struct sockaddr*)&socketInfo.serverAddress, sizeof(socketInfo.serverAddress)) == -1) {
        std::cerr << "Failed to send message to socket " << socketIndex << std::endl;
        return false;
    }

    return true;
}

void UdpCommunication::registerReceiveCallback(int socketIndex, ReceiveCallback callback) {
    if (socketIndex < 0 || socketIndex >= static_cast<int>(sockets_.size())) {
        std::cerr << "Invalid socket index" << std::endl;
        return;
    }

    // Check if the callback is valid
    if (!callback) {
        std::cerr << "Invalid callback function" << std::endl;
        return;
    }

    sockets_[socketIndex].callback = callback;
}

void UdpCommunication::startReceiveCallbacks() {
    fd_set readfds;

    while (true) {
        FD_ZERO(&readfds);
        int max_sd = 0;

        for (const auto& socketInfo : sockets_) {
            FD_SET(socketInfo.sockfd, &readfds);
            if (socketInfo.sockfd > max_sd) {
                max_sd = socketInfo.sockfd;
            }
        }

        int activity = select(max_sd + 1, &readfds, nullptr, nullptr, nullptr);

        if (activity < 0 && errno != EINTR) {
            std::cerr << "Select error" << std::endl;
            return;
        }

        for (auto& socketInfo : sockets_) {
            if (FD_ISSET(socketInfo.sockfd, &readfds)) {
                socklen_t addrLen = sizeof(socketInfo.serverAddress);
                // Receive the data directly into the fixed receiveBuffer
                ssize_t bytesRead = recvfrom(socketInfo.sockfd, socketInfo.receiveBuffer, socketInfo.receiveBufferSize,
                                             0, (struct sockaddr*)&socketInfo.serverAddress, &addrLen);
                if (bytesRead < 0) {
                    std::cerr << "Failed to receive message" << std::endl;
                } else if (socketInfo.callback) {
                    socketInfo.callback();
                }
            }
        }
    }
}
