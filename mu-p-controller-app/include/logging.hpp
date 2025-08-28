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

#include <iostream>

#define ENABLE_LOGGING 0  // Set to 0 to disable logging in cout

#if ENABLE_LOGGING
#define LOG(msg) std::cout << msg << std::endl
#else
#define LOG(msg)  // No operation
#endif

#define STORE_DATAVEC 0  // Set to 0 to disable storing data in datavec (DataCollector)
