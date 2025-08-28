/**
 * Copyright (c) 2025 Robert Bosch GmbH.
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <arpa/inet.h>

#include <functional>
#include <string>
#include <vector>

class UdpCommunication {
    using ReceiveCallback = std::function<void()>;

   private:
    struct SocketInfo {
        int sockfd;
        struct sockaddr_in serverAddress;
        ReceiveCallback callback;
        char* receiveBuffer;
        char* sendBuffer;
        std::size_t receiveBufferSize;
        std::size_t sendBufferSize;
    };

    std::vector<SocketInfo> sockets_;

   public:
    UdpCommunication(const std::string& address, uint16_t port, char* receiveBuffer, std::size_t receiveBufferSize,
                     char* sendBuffer, std::size_t sendBufferSize);
    ~UdpCommunication();

    void addSocket(const std::string& address, uint16_t port, char* receiveBuffer, std::size_t receiveBufferSize,
                   char* sendBuffer, std::size_t sendBufferSize);
    void registerReceiveCallback(int socketIndex, ReceiveCallback callback);

    inline const SocketInfo& getSocketInfo(int socketIndex) const { return sockets_[socketIndex]; };

    bool send();
    bool sendToSocket(int socketIndex);

    void startReceiveCallbacks();
};
