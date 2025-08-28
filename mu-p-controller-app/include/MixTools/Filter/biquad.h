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
#include <vector>
#include <array>

class BiquadFilter {
private:
    std::vector<std::array<double, 6>> sos;  // Coefficients for each biquad section each 6 element array defines section completly
    std::vector<double> x1, x2;  // Input delay elements
    std::vector<double> y1, y2;  // Output delay elements
public:
    // Constructor that initializes the filter with an SOS array
    //initializes the filter regardless of size. 
    BiquadFilter(const std::vector<std::array<double, 6>>& sosArray)
        : sos(sosArray), x1(sosArray.size(), 0.0), x2(sosArray.size(), 0.0),
          y1(sosArray.size(), 0.0), y2(sosArray.size(), 0.0) {}

    // Method to process a single sample through the filter
    double process(double input) {
        double output = input;

        // Apply each biquad section
        for (size_t i = 0; i < sos.size(); ++i) {
            double x0 = output;
            output = sos[i][0] * x0 + sos[i][1] * x1[i] + sos[i][2] * x2[i]
                              - sos[i][4] * y1[i] - sos[i][5] * y2[i];
            
            // Update the state variables for the next sample
            x2[i] = x1[i];
            x1[i] = x0;
            y2[i] = y1[i];
            y1[i] = output;
        }

        return output;
    }

    // Method to reset the filter states
    void reset() {
        std::fill(x1.begin(), x1.end(), 0.0);
        std::fill(x2.begin(), x2.end(), 0.0);
        std::fill(y1.begin(), y1.end(), 0.0);
        std::fill(y2.begin(), y2.end(), 0.0);
    }


};


