/**
 * Copyright (c) 2025 Robert Bosch GmbH.
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#include "../../include/config/control_parameters.hpp"

// parameters of the 2DOF controller for the small motor
std::vector<double> numeratorF = {0.26401307473133517334, -0.31959969611430943948, 0.13238184870010982763};
std::vector<double> denominatorF = {1, -1.8181990055294141584, 0.89499423284654966437};
std::vector<double> numeratorC = {-0.55620051656205504731, -0.48067348538851412121, 0.056320922006560934781,
                                  0.48512221888985240792, 0.17631215737364100415};
std::vector<double> denominatorC = {1, 0.57124346874842291122, 0.21399463094184736911, 0.077514461143111584862,
                                    0.0034258976974677174919};
dataController valuesController = {numeratorF, numeratorC, denominatorF, denominatorC, u_min, u_max};

DOFcontroller controllerSmallMotorLeft(valuesController.numeratorF, valuesController.numeratorC,
                                       valuesController.denominatorF, valuesController.denominatorC,
                                       valuesController.u_min, valuesController.u_max);

DOFcontroller controllerSmallMotorRight(valuesController.numeratorF, valuesController.numeratorC,
                                        valuesController.denominatorF, valuesController.denominatorC,
                                        valuesController.u_min, valuesController.u_max);

DOFcontroller controllerBuzzWireLeft(valuesController.numeratorF, valuesController.numeratorC,
                                        valuesController.denominatorF, valuesController.denominatorC,
                                        valuesController.u_min, valuesController.u_max);

DOFcontroller controllerBuzzWireRight(valuesController.numeratorF, valuesController.numeratorC,
                                        valuesController.denominatorF, valuesController.denominatorC,
                                        valuesController.u_min, valuesController.u_max);


std::vector<int> receiverCPUCore = {1,2,3,4,5,6,7};
std::vector<int> senderCPUCore = {1,2,3,4,5,6,7};
std::vector<int> kuksaCPUCore = {0};
