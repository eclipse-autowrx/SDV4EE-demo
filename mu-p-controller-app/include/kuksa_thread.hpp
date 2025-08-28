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

#include <cstdint>

#include "MixTools/RTthreads/RTthreads.hpp"
#include "multi_control3.hpp"

#define KUKSA_CURRENT_ANGLE_SIGNAL  "Vehicle.Chassis.SteeringWheel.Angle"
#define KUKSA_SET_ANGLE_SIGNAL      "Vehicle.ADAS.LaneAssist.TargetSteeringWheelAngle"
#define KUKSA_SET_TORQUE_SIGNAL     "Vehicle.ADAS.LaneAssist.SteeringTorque"

class KuksaThread : public CyclicRTthread {
public:
    KuksaThread(int priority, int policy, std::vector<int> cores, int period_ns,
            kuksa::KuksaClient& kuksa_client, bool is_kuksa_connected, MultiControl3 &multi_control3);

    inline bool loop() override { return timerCallback(); }

private:
    MultiControl3 &multi_control3_;
    kuksa::KuksaClient& kuksa_client_;
    bool is_kuksa_connected_;
    int32_t target_steering_wheel_angle_;
    bool move_to_target_steering_wheel_angle_;
    uint64_t counter_;

    bool timerCallback();

    void on_target_changed(const std::string &path, const kuksa::val::v2::Value &value);
};
