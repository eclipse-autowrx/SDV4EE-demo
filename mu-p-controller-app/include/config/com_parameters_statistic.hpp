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

#include <cstdint>



#pragma pack(push, 1)
struct StatisticData {
    double thread_accuracy_s;
    double small_motor_1_angle;
    double small_motor_1_torque;
    double small_motor_2_angle;
    double small_motor_2_torque;
};
#pragma pack(pop)

extern StatisticData statistic_data;
constexpr std::uint32_t send_period_statistic_ns = 3e8;  // every 300ms


