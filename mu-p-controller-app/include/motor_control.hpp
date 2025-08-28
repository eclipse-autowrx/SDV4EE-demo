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

#include "controller.hpp"
#include "motor_state.hpp"

class MotorControl {
   public:
    MotorControl(DOFcontroller& controller);

    inline MotorState& getMotorState() { return motor_state_; }
    inline double getR() const { return r_; };
    inline double getTorque() const { return set_torque_; };
    inline u_int32_t getTime() const { return time_count_; };

    inline void setMotorState(MotorState motor_state) { motor_state_ = motor_state; };
    inline void setR(double r) { r_ = r; };
    inline void setTorque(double torque) { set_torque_ = torque; };
    inline void setTime(u_int32_t time) { time_count_ = time; };

    void calc();
    void calc(double target_angle);

   private:
    double r_;
    double set_torque_;
    uint32_t time_count_;
    MotorState motor_state_;
    DOFcontroller& controller_;

   protected:
    void updateR();
    void updateSetTorque();
};
