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

#include <algorithm>
#include <shared_mutex>

#include "motor_control.hpp"
#include "kuksaclient.h"

class MultiControl3 {
   public:
    MultiControl3(MotorControl& control_left, MotorControl& control_right, MotorControl& control_buzzwire_left, MotorControl& control_buzzwire_right, bool is_buzzwire);
    void calc();
    double getLeftAngle();
    double getRightAngle();
    double getBuzzWireLeftAngle();
    double getBuzzWireRightAngle();

    MotorState& getLeftState();
    MotorState& getRightState();
    MotorState& getBuzzWireLeftState();
    MotorState& getBuzzWireRightState();

    double getLeftTorque();

    double getRightTorque();

    double getBuzzWireLeftTorque();

    double getBuzzWireRightTorque();

    uint16_t getLeftRoundtripLatency();
    uint16_t getRightRoundtripLatency();
    uint16_t getLeftCanLatency();
    uint16_t getRightCanLatency();
    uint16_t getLeftEthLatency();
    uint16_t getRightEthLatency();
    uint64_t getLeftTimestamp();
    uint64_t getRightTimestamp();


    MotorControl& getLeftControl();
    MotorControl& getRightControl();
    MotorControl& getBuzzWireLeftControl();
    MotorControl& getBuzzWireRightControl();

    void setTargetSteeringWheelAngle(int32_t target_steering_wheel_angle);
    void setMoveToTargetSteeringWheelAngle(bool move_to_target_steering_wheel_angle);

   private:
    int32_t target_steering_wheel_angle_;
    bool move_to_target_steering_wheel_angle_;
    std::shared_mutex mutex_; // R/W lock for concurrent access from different threads
    MotorControl& control_left_;
    MotorControl& control_right_;
    MotorControl& control_buzzwire_left_;
    MotorControl& control_buzzwire_right_;
    bool is_buzzwire;
    double filtered_left_angle_;
    double applyLowPassFilter(double current_angle, double& filtered_angle); // Declare the method
    void calc_left();
    void calc_right();
    void calc_buzzwire_left();
    void calc_buzzwire_right();
};

