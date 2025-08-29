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

// #include <unistd.h>

#include <cstdint>
#include <vector>

#include "controller.hpp"

struct dataSweep {
    double w0;
    double w1;
    double r_max;
    double T_sweep;  // = (T-20000.0)/1000.0
};

struct dataController {
    std::vector<double> numeratorF;
    std::vector<double> numeratorC;
    std::vector<double> denominatorF;
    std::vector<double> denominatorC;
    double u_min;
    double u_max;
};


extern DOFcontroller controllerSmallMotorLeft;
extern DOFcontroller controllerSmallMotorRight;
extern DOFcontroller controllerBuzzWireLeft;
extern DOFcontroller controllerBuzzWireRight;

extern std::vector<int> receiverCPUCore;
extern std::vector<int> senderCPUCore;
extern std::vector<int> kuksaCPUCore;

// constexpr guaranteed to be no duplicate definition per compilation unit without extern
// constexpr auto inline from c++17 and that's why to be defined in header



constexpr double u_min = -7.0;  // maximum allowed input. In theory one can go up to 32 but this is enough and reduces

constexpr double u_max = 7.0;


