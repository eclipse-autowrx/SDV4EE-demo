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

#include "MixTools/Filter/digFilter.h"
#include "antiWindUp.hpp"
#include <vector>
#include <array>

class DOFcontroller{
    private: 
    DirectForm2TransposedFilter prefilter_; 
    DirectForm2TransposedFilter controller_;
    AntiWindUp AW_;
    public: 
    DOFcontroller(DirectForm2TransposedFilter prefilter, DirectForm2TransposedFilter controller, AntiWindUp aw): 
    prefilter_(prefilter), controller_(controller), AW_(aw){}

    //alternate constructor that just takes the necessary variables
    DOFcontroller(const std::vector<double>& numeratorPrefilter,const std::vector<double>& numeratorController,
    const std::vector<double>& denominatorPrefilter, const std::vector<double>& denominatorController, double u_min, double u_max)
        : prefilter_(numeratorPrefilter, denominatorPrefilter), controller_(numeratorController, denominatorController),
         AW_(numeratorPrefilter[0], numeratorController[0], u_min, u_max) {}
    //standard 2DOF controller with special Anti Wind up using the fact, that each controller can be 
    //transformed into a digital Filter
    double update(double r, double y){
        double r_limited = AW_.process(r, controller_.getState(0), prefilter_.getState(0),y);
        double r_filtered = prefilter_.process(r_limited);
        double ef = r_filtered-y;
        double u = controller_.process(ef);
        return u; 
    }
   DirectForm2TransposedFilter getFilter(){
    return prefilter_;
   } 
   DirectForm2TransposedFilter getController(){
    return controller_;
   }
   void setControllerStates(const std::vector<double> states){
    controller_.setStates(states);
   }
   void setFilterStates(const std::vector<double> states){
    prefilter_.setStates(states);
   }
};
