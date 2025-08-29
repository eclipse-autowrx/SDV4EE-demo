/**
 * Copyright (c) 2025 Bosch Global Software Technologies Private Limited.
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#include "../../include/config/com_parameters.hpp"

//std::string filepath{"/home/bluebox/repos/baremetalMotorControl/measurements/measurement.csv"};

// Define the global buffers
char buffer_rx_[sizeof(MotorData_Rx)];
char buffer_tx_[sizeof(ControlData_Tx)];
char buffer2_rx_[sizeof(MotorData_Rx)];
char buffer2_tx_[sizeof(ControlData_Tx)];
char buffer3_rx_[sizeof(MotorData_Rx)];
char buffer3_tx_[sizeof(ControlData_Tx)];
char buffer4_rx_[sizeof(MotorData_Rx)];
char buffer4_tx_[sizeof(ControlData_Tx)];
char buffer5_rx_[sizeof(MotorData_Rx)];
char buffer5_tx_[sizeof(ControlData_Tx)];
std::chrono::duration<double> timedif{0};

