/**
 * Copyright (c) 2025 Robert Bosch GmbH.
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#include <cstring>
#include <stdexcept>
#include <iostream>  // Add this for debugging output

#include "../../include/serializer.hpp"

template <>
void Serializer<ControlData_Tx>::serializeFields(const ControlData_Tx &packet, char *buffer, std::size_t &offset) {
    // Prepare CAN frame
    CANFrame canFrame;


    offset = 0;
    
    // Set CAN ID (0x20)
    canFrame.id = 0x20;
    
    // Set Data Length Code (DLC) = 8
    canFrame.dlc = 8;
    
    // Set the data field (first byte of data after DLC should be packet.set_torque)
    // Assuming packet.set_torque is a float
    std::memcpy(canFrame.data, &packet.set_torque, sizeof(packet.set_torque));
    
    // Copy CAN frame to the buffer
    // Copy CAN ID (4 bytes)
    std::memcpy(buffer + offset, &canFrame.id, sizeof(canFrame.id));
    offset += sizeof(canFrame.id);

    // Copy Data Length Code (DLC) (1 byte)
    std::memcpy(buffer + offset, &canFrame.dlc, sizeof(canFrame.dlc));
    offset += sizeof(canFrame.dlc);

    // Copy data payload (8 bytes)
    std::memcpy(buffer + offset, canFrame.data, sizeof(canFrame.data));
    offset += sizeof(canFrame.data);
}

template <>
void Serializer<MotorData_Rx>::deserializeFields(MotorData_Rx &packet, const char *buffer, std::size_t &offset) {
    offset = 0;  // Start at the beginning of the buffer
    
    // Extract CAN ID (4 bytes)
    uint32_t canId;
    std::memcpy(&canId, buffer + offset, sizeof(canId));
    canId = ntohl(canId);  // Convert from network byte order (big-endian) to host byte order (little-endian)
    offset += sizeof(canId);  // Increment offset after reading CAN ID
  
    // Extract Data Length Code (DLC) (1 byte)
    uint8_t dlc;
    std::memcpy(&dlc, buffer + offset, sizeof(dlc));
    offset += sizeof(dlc);  // Increment offset after reading DLC
    
    packet.can_frame_id = canId;
    // If CAN ID is 0x21, extract the payload
    if (canId == 0x21) {
        // Check if there's enough space for motor_angle in the payload
        if (dlc >= sizeof(packet.motor_angle)) {

             
            // Copy the data payload (motor_angle) into the packet's motor_angle field
            std::memcpy(&packet.motor_angle, buffer + offset, sizeof(packet.motor_angle));
            offset += sizeof(packet.motor_angle);  // Move offset after copying motor_angle


          //  std::cout << "deserializeFields called! motor angle:" << packet.motor_angle << std::endl;  // Debug message

        } else {
            // Handle error if DLC is too small to contain the required data (optional)

        }
    }
    
  
    if (canId == 0x51) {
        // Check if there's timestamp in payload
        if (dlc == 8) {

             
            std::memcpy(&packet.time_stamp, buffer + offset, sizeof(packet.time_stamp));
        }
    }
    
    if (canId == 0x71) {
        // Check if there's timestamp in payload
        if (dlc == 8) {
            std::memcpy(&packet.can_latency, buffer + offset, sizeof(packet.can_latency));
            offset += 2;

            std::memcpy(&packet.eth_latency, buffer + offset, sizeof(packet.eth_latency));

        }
    }
    
}
