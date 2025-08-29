/**
 * Copyright (c) 2025 Bosch Global Software Technologies Private Limited.
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#include "../../include/motor_state.hpp"

MotorState::MotorState()

    :

      old_angle_(0.0),
      current_angle_(0.0),
      time_count_(0) {}

void MotorState::updateMotorAngle(double new_angle) {

    current_angle_ = new_angle;

    old_angle_ = new_angle;
}

double MotorState::reverseModulo(double motor_angle, double old_angle) {
    double angle_difference = motor_angle - old_angle; // angle_difference ist motor movement

    if ((angle_difference) > 150) { 
        // If motor turns into negative direction and jumps from low angles to very high angles 
        // Example: (e.g. 5° -> 355°)
        // measured angle-difference = 350
        // correct angle-difference = -10
        angle_difference = angle_difference - 360; // so, correct movement is -10°
    } else if ((angle_difference) < -150) { // Example from above in reverse
        angle_difference = angle_difference + 360;
    } 
    return angle_difference;
}

