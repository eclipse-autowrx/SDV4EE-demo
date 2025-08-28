/**
 * Copyright (c) 2025 Robert Bosch GmbH.
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#include "kuksa_thread.hpp"

KuksaThread::KuksaThread(int priority, int policy, std::vector<int> cores, int period_ns,
        kuksa::KuksaClient& kuksa_client, bool is_kuksa_connected, MultiControl3 &multi_control3)
    : CyclicRTthread(priority, policy, cores, period_ns)
    , multi_control3_(multi_control3)
    , kuksa_client_(kuksa_client)
    , is_kuksa_connected_(is_kuksa_connected)
    , target_steering_wheel_angle_(0)
    , move_to_target_steering_wheel_angle_(false)
    , counter_(0)
{
    // Subscribe to KUKSA signals
    if (is_kuksa_connected_) {
        using namespace std::placeholders;
        std::vector<std::string> signals = {KUKSA_SET_ANGLE_SIGNAL};
        kuksa_client_.subscribe(signals, std::bind(&KuksaThread::on_target_changed, this, _1, _2));
    }
};

bool KuksaThread::timerCallback() {
    // Publish current angle to KUKSA
    if (is_kuksa_connected_) {
        kuksa::val::v2::Value value{};
        value.set_float_(multi_control3_.getRightAngle());
        // if (current_angle >= 0) {
        // } else {
        //     value.set_int32(- static_cast<int>(current_angle));
        // }
        kuksa_client_.publishValue(KUKSA_CURRENT_ANGLE_SIGNAL, value);
    }

    counter_++;
    return false;
};

void KuksaThread::on_target_changed(const std::string &path, const kuksa::val::v2::Value &value) {
    if (value.typed_value_case() == kuksa::val::v2::Value::kInt32) {
        // targetSteeringWheelAngle_ = 0;
        target_steering_wheel_angle_ = value.int32();
        move_to_target_steering_wheel_angle_ = true;
        multi_control3_.setTargetSteeringWheelAngle(-target_steering_wheel_angle_);
        multi_control3_.setMoveToTargetSteeringWheelAngle(move_to_target_steering_wheel_angle_);
    }
};
