/**
 * Copyright (c) 2025 Bosch Global Software Technologies Private Limited.
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#include "../include/multi_control3.hpp"
#include <iostream>
#include <algorithm>
#include <functional>
#include <mutex>

#define MAX_ANGLE   1000.0

#define USE_MUTEX   1

MultiControl3::MultiControl3(MotorControl& control_left, MotorControl& control_right, MotorControl& control_buzzwire_left, MotorControl& control_buzzwire_right, bool is_buzzwire)
    : target_steering_wheel_angle_(0),
      move_to_target_steering_wheel_angle_(false),
      control_left_(control_left),
      control_right_(control_right),
      control_buzzwire_left_(control_buzzwire_left),
      control_buzzwire_right_(control_buzzwire_right),
      is_buzzwire(is_buzzwire),
      filtered_left_angle_(0.0)
{
}

void MultiControl3::calc() {
#if USE_MUTEX
    std::unique_lock lock(mutex_); // Acquire write lock
#endif

    double current_angle = control_right_.getMotorState().getActualAngle();

    if (move_to_target_steering_wheel_angle_){

        if (std::abs(current_angle - target_steering_wheel_angle_) < 10 ) {
            move_to_target_steering_wheel_angle_ = false;
        }
        else {
            current_angle = target_steering_wheel_angle_;
        }
    }

    current_angle = current_angle > MAX_ANGLE ? MAX_ANGLE : current_angle < -MAX_ANGLE ? -MAX_ANGLE : current_angle;

    // if (update_counter_ % 1000 == 0) {
    //     std::cout << "Target Steering Wheel Angle " << target_steering_wheel_angle_ << std::endl;
    //     std::cout << "Left torque " << getLeftTorque() << std::endl;
    //     std::cout << "Current angle " << current_angle << std::endl;
    // }

    control_left_.calc(current_angle);
    calc_right();

    if (is_buzzwire) {
        control_buzzwire_left_.calc(current_angle);
        control_buzzwire_right_.calc(-current_angle);
    }
}

double MultiControl3::getLeftAngle() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_left_.getMotorState().getActualAngle();
}

double MultiControl3::getRightAngle() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_right_.getMotorState().getActualAngle();
}

double MultiControl3::getBuzzWireLeftAngle() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_left_.getMotorState().getActualAngle();
}

double MultiControl3::getBuzzWireRightAngle() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_right_.getMotorState().getActualAngle();
}

MotorState& MultiControl3::getLeftState() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_left_.getMotorState();
}

MotorState& MultiControl3::getRightState() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_right_.getMotorState();
}

MotorState& MultiControl3::getBuzzWireLeftState() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_buzzwire_left_.getMotorState();
}

MotorState& MultiControl3::getBuzzWireRightState() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_buzzwire_right_.getMotorState();
}

double MultiControl3::getLeftTorque() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return  std::clamp<double>(control_left_.getTorque() * 1.0, -10.0, 10.0);
}

double MultiControl3::getRightTorque() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return  control_right_.getTorque() * 1.0;
}

double MultiControl3::getBuzzWireLeftTorque() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return  control_buzzwire_left_.getTorque() * 2.5;
}

double MultiControl3::getBuzzWireRightTorque() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return  - control_buzzwire_right_.getTorque() * 2.5;
}

uint16_t MultiControl3::getLeftRoundtripLatency() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_left_.getMotorState().getRoundtripLatency();
}

uint16_t MultiControl3::getRightRoundtripLatency() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_right_.getMotorState().getRoundtripLatency();
}

uint16_t MultiControl3::getLeftCanLatency() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_left_.getMotorState().getCanLatency();
}

uint16_t MultiControl3::getRightCanLatency() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_right_.getMotorState().getCanLatency();
}

uint16_t MultiControl3::getLeftEthLatency() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_left_.getMotorState().getEthLatency();
}

uint16_t MultiControl3::getRightEthLatency() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_right_.getMotorState().getEthLatency();
}


uint64_t MultiControl3::getLeftTimestamp() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_left_.getMotorState().getRoundtripTimestamp();
}

uint64_t MultiControl3::getRightTimestamp() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_right_.getMotorState().getRoundtripTimestamp();
}



MotorControl& MultiControl3::getLeftControl() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_left_;
}

MotorControl& MultiControl3::getRightControl() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_right_;
}

MotorControl& MultiControl3::getBuzzWireLeftControl() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_buzzwire_left_;
}

MotorControl& MultiControl3::getBuzzWireRightControl() {
#if USE_MUTEX
    std::shared_lock lock(mutex_); // Acquire read lock
#endif
    return control_buzzwire_right_;
}

void MultiControl3::calc_left() {
    // This method is now handled in calc()
}

void MultiControl3::calc_right() {
    control_right_.calc(control_left_.getMotorState().getActualAngle());
}

void MultiControl3::calc_buzzwire_left() {
    // This method is now handled in calc()
}

void MultiControl3::calc_buzzwire_right() {
    // This method is now handled in calc()
}

void MultiControl3::setTargetSteeringWheelAngle(int32_t target_steering_wheel_angle)
{
#if USE_MUTEX
    std::unique_lock lock(mutex_); // Acquire write lock
#endif
    target_steering_wheel_angle_ = target_steering_wheel_angle;
}
void MultiControl3::setMoveToTargetSteeringWheelAngle(bool move_to_target_steering_wheel_angle)
{
#if USE_MUTEX
    std::unique_lock lock(mutex_); // Acquire write lock
#endif
    move_to_target_steering_wheel_angle_ = move_to_target_steering_wheel_angle;
}

