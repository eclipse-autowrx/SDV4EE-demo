/**
 * Copyright (c) 2025 Robert Bosch GmbH.
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#include "../include/motor_control.hpp"

#include "../include/config/control_parameters.hpp"

MotorControl::MotorControl(DOFcontroller& controller)

    :

      r_(0.0),
      set_torque_(0.0),
      time_count_(0),
      controller_(controller){};


void MotorControl::calc(double target_angle) {
    setR(target_angle);
    updateSetTorque();
}

void MotorControl::updateSetTorque() {
    motor_state_.incrementTimeCount();
    set_torque_ = (controller_.update(r_, motor_state_.getActualAngle()));
}
