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

#include <vector>
#include <utility> // For std::pair

class DirectForm2TransposedFilter {
public:
    // Constructor that initializes the filter with numerator and denominator coefficients
    DirectForm2TransposedFilter(const std::vector<double>& numerator, const std::vector<double>& denominator)
        : b(numerator), a(denominator), x(std::max(numerator.size(), denominator.size()) - 1, 0.0) 
    {
        normalizeCoefficients();
    }

    // Method to process a single sample through the filter
    double process(double input) {
        double output = b[0] * input + x[0];
        //update states
        for (std::size_t i = 1; i < x.size(); ++i) {
            x[i - 1] = b[i] * input + x[i] - a[i] * output;
        }
        //calculate the last element, that does not depend on the others. .back()is just that element
        if (!x.empty()) {
            x.back() = b.back() * input - a.back() * output;
        }

        return output;
    }

    // Method to reset the filter states
    void reset() {
        std::fill(x.begin(), x.end(), 0.0);
    }

    // Method to get the internal states of the filter
    std::vector<double> getInternalStates() const {
        return x;
    }
    void setStates(const std::vector<double> states){
        x = states;
    }
    // Method to get the filter coefficients
    std::pair<std::vector<double>, std::vector<double>> getCoefficients() const {
        return {b, a};
    }
    std::vector<double> getNum(){
        return b;
    }

    // Method to set the filter coefficients
    void setCoefficients(const std::vector<double>& numerator, const std::vector<double>& denominator) {
        b = numerator;
        a = denominator;
        x.resize(std::max(b.size(), a.size()) - 1, 0.0);
        normalizeCoefficients();
    }

    // Method to print the state corresponding to a given index
    // this is needed for anti wind up, returns -10 if no state at that index to avoid crashes
    double getState(int index) const {
        if (index >= 0 && static_cast<std::size_t>(index) < x.size()) {
            return x[index];
        } else {
            return -10;
        }
    }

private:
    // Method to normalize coefficients if a[0] is not 1
    void normalizeCoefficients() {
        if (a[0] != 1.0) {
            for (double& coef : b) {
                coef /= a[0];
            }
            for (double& coef : a) {
                coef /= a[0];
            }
        }
    }

    std::vector<double> b;    // Numerator coefficients
    std::vector<double> a;    // Denominator coefficients
    std::vector<double> x;    // State variables
};

