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

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

// com parameters
constexpr bool LOG_LATENCY = false;

constexpr const char* ZONE_LEFT_IP_DEFAULT = "192.168.88.201";
constexpr const char* ZONE_RIGHT_IP_DEFAULT = "192.168.88.202";
constexpr const char* ZONE_BUZZWIRE_LEFT_IP_DEFAULT = "192.168.88.204";
constexpr const char* ZONE_BUZZWIRE_RIGHT_IP_DEFAULT = "192.168.88.203";
constexpr std::uint16_t ZONE_DEFAULT_PORT = 12345;

// transmit data struct
struct ControlData_Tx {
    uint64_t time_stamp;
    float set_torque;
};

struct MotorData_Rx {
    uint64_t time_stamp;
    uint32_t can_frame_id;
    float motor_angle;
    uint16_t can_latency;
    uint16_t eth_latency;
};

// Declare the global buffers and their sizes as extern
extern char buffer_rx_[];
extern char buffer_tx_[];
extern char buffer2_rx_[];
extern char buffer2_tx_[];
extern char buffer3_rx_[];
extern char buffer3_tx_[];
extern char buffer4_rx_[];
extern char buffer4_tx_[];
extern char buffer5_rx_[];
extern char buffer5_tx_[];

// Declare their sizes as constexpr
constexpr std::size_t buffer_size_rx_ = sizeof(MotorData_Rx);
constexpr std::size_t buffer_size_tx_ = sizeof(ControlData_Tx);
constexpr std::size_t buffer2_size_rx_ = sizeof(MotorData_Rx);
constexpr std::size_t buffer2_size_tx_ = sizeof(ControlData_Tx);
constexpr std::size_t buffer3_size_rx_ = sizeof(MotorData_Rx);
constexpr std::size_t buffer3_size_tx_ = sizeof(ControlData_Tx);
constexpr std::size_t buffer4_size_rx_ = sizeof(MotorData_Rx);
constexpr std::size_t buffer4_size_tx_ = sizeof(ControlData_Tx);
constexpr std::size_t buffer5_size_rx_ = sizeof(MotorData_Rx);
constexpr std::size_t buffer5_size_tx_ = sizeof(ControlData_Tx);

// filepath for storing measurement data
extern std::string filepath;
extern std::chrono::duration<double> timedif;

