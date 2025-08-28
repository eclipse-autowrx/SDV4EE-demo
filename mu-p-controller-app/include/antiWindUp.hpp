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

#include "algorithm"

class AntiWindUp {
   private:
    double bf1_;    // first coefficient of Prefilter
    double bc1_;    // first coefficient of Controller (both as digital Filters in DF2 trnasposed form)
    double u_min_;  // limits for u
    double u_max_;

   public:
    AntiWindUp(double bf1, double bc1, double u_min, double u_max)

        :

          bf1_(bf1),
          bc1_(bc1),
          u_min_(u_min),
          u_max_(u_max) {}
    /** this works based on the fact, that every controller and prefilter can be transformed into a filter of
     * direct form 2 transposed. In this form u can be calculated using the internal states of both prefilter and
     * controller to
     * u = e*numC(1) + xc1
     * e = numF(1)*r + xf1 -y
     * -> u = numc(1)*(numF(1)*r + xf1 -y)+xc1
     * this we can now solve for r in relation to u
     * r = ((u-xc1)/numC(1)-xf1+y)/numF(1)
     * We can now replace u with our u_min/u_max to calculate the maximum r, that is able to be send into the system to
     * get the maximum u without winding up the integrator part of the controller This version straight up returns the
     * resulting r depending on if r hits those extrema. Could be disadvantageous for debugging but increases
     * readability
     * */
    double process(double r, double xc1, double xf1, double y) {
        double r1 =
            (-xc1 + u_min_ + bc1_ * (-xf1 + y)) / (bc1_ * bf1_);  // calculates the edge cases for minimum and maximum
        double r2 = (-xc1 + u_max_ + bc1_ * (-xf1 + y)) / (bc1_ * bf1_);
        double r_max = std::max(r1, r2);
        double r_min = std::min(r1, r2);
        double rl = std::min(std::max(r, r_min), r_max);
        return rl;
    }
};