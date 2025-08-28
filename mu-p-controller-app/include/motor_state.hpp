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
#include <chrono>

class MotorState {
   public:
    MotorState();

    void updateMotorAngle(double new_angle);

    inline double getActualAngle() const { return current_angle_; }
    inline uint32_t getTimeCount() const { return time_count_; }
    inline uint16_t getRoundtripLatency() const { return rt_latency_; }
    inline uint16_t getCanLatency() const { return can_latency_; }
    inline uint16_t getEthLatency() const { return eth_latency_; }
    inline uint64_t getRoundtripTimestamp() const { return time_stamp_; }
    inline double getOldAngle() const {return old_angle_;}
    inline void incrementTimeCount() { time_count_++; }
    inline void setInitialAngle(double angle) { old_angle_ = angle; }
    inline void setRoundtripLatency(uint16_t latency) { rt_latency_ = latency; }
    inline void setCanLatency(uint16_t latency) { can_latency_ = latency; }
    inline void setEthLatency(uint16_t latency) { eth_latency_ = latency; }
    inline void setRoundtripTimestamp(uint64_t time_stamp) { time_stamp_ = time_stamp; }
    inline void setTimepoint() { time_point_ = std::chrono::steady_clock::now(); }
    inline uint64_t getDuration() const {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - time_point_
        ).count();
    }
   private:
    // the sensor on the motor only returns states from 0 to 360 degrees to increase accuracy as there is a fixed number
    // of bytes to return. To not be limited to this range, when controlling the motor, the actual angle needs to be
    // calculated this is done by comparing the new value with the old one and using knowledge about the motors physics
    // (there won't be more than 150 degrees of rotation within one millisecond)
    double reverseModulo(double motor_angle, double old_angle);

    double old_angle_;
    double current_angle_;
    double current_angle_raw_;
    uint32_t time_count_;
    uint16_t rt_latency_ = 0;
    uint16_t can_latency_ = 0;
    uint16_t eth_latency_ = 0;
    uint64_t time_stamp_ = 0;
    std::chrono::steady_clock::time_point time_point_;

};
