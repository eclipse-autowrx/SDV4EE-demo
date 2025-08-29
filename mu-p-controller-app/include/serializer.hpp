/**
 * Copyright (c) 2025 Bosch Global Software Technologies Private Limited.
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include "config/com_parameters.hpp"
#include <arpa/inet.h>  // For ntohl and network functions


struct CANFrame {
    uint32_t id;       // CAN ID (11-bit or 29-bit)
    uint8_t dlc;       // Data length code (0-8 bytes)
    uint8_t data[8];   // Data payload
};


template <typename PacketType>
class Serializer {
   public:
    static void serialize(const PacketType &packet, char *buffer, std::size_t bufferSize) {
        if (bufferSize < sizeof(PacketType)) {
            throw std::runtime_error("Buffer size is too small");
        }

        std::size_t offset = 0;
        serializeFields(packet, buffer, offset);
    }
 
    static void deserialize(PacketType &packet, const char *buffer, std::size_t bufferSize) {
        if (bufferSize + 8 < sizeof(PacketType)) {
            std::cout << "Buffer small size..." << std::endl;
            throw std::runtime_error("Buffer size is too small");
        }

        std::size_t offset = 0;
        deserializeFields(packet, buffer, offset);
    }

    // Method to determine the required buffer size
    static std::size_t getRequiredBufferSize(const PacketType &packet) {
        return sizeof(PacketType);
        // If dynamic fields are present, additional logic would be required here.
    }

   private:
    static void serializeFields(const PacketType &packet, char *buffer, std::size_t &offset);
    static void deserializeFields(PacketType &packet, const char *buffer, std::size_t &offset);
};

// Template specializations for ControlData_Tx
template <>
void Serializer<ControlData_Tx>::serializeFields(const ControlData_Tx &packet, char *buffer, std::size_t &offset);

// Template specializations for MotorData_Rx
template <>
void Serializer<MotorData_Rx>::deserializeFields(MotorData_Rx &packet, const char *buffer, std::size_t &offset);

