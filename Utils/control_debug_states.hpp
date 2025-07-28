/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#pragma once

#include <stdint.h>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "units.hpp"

class ControlDebugStates {
    // attributes
 public:
    // Were originally C-types long int, Dor changed to int64_t
    // Perhaps can make int16 (didn't know value ranges for variables)
    // std::vector<std::vector<PreciseMps>> vehicle_speed_;
    // std::vector<std::vector<PreciseRadians>> vehicle_heading_;
    std::vector<std::pair<PreciseMps, PreciseSeconds>>
        vehicle_speed_;  // <vehicle_heading_, time>
    
    
    std::vector<std::pair<PreciseRadians, PreciseSeconds>>
        vehicle_heading_;  // <vehicle_heading_, time>
    std::vector<std::pair<std::vector<double>, double>>
        vehicle_state_;  // <<x,y,psi>, time>
    std::vector<std::pair<PreciseRadians, PreciseSeconds>>
        delta_;  // <steering command, time>
    std::vector<std::pair<PreciseRadians, PreciseSeconds>>
        steering_measured_;  // <steering, time>
    // Reference velocity and velocity errors of different orders
    // NOTE: Controller's vehicle_velocity_ is updated via controller,
    // unlike vehicle_speed_, which is updated from localization obj
    // via UpdateSpeed(). Duplication for compatibility with Python code.
    std::vector<std::pair<PreciseMps, PreciseSeconds>> vehicle_velocity_;
    std::vector<std::pair<PreciseMps, PreciseSeconds>>
        vehicle_velocity_filtered_;
    std::vector<std::pair<PreciseSeconds, PreciseSeconds>> clock_;
    std::vector<std::pair<PreciseSeconds, PreciseSeconds>> dt_;

    // methods
    std::mutex lock_;
    ControlDebugStates() {}
    ControlDebugStates(bool create_debug_dir, bool localization_mode,
                       std::filesystem::path control_module_dir);
    void WriteLocalizationStatesToFile(PreciseSeconds clock,
                                       PreciseMps car_speed,
                                       PreciseRadians steering_angle,
                                       PreciseMeters vehicle_x,
                                       PreciseMeters vehicle_y,
                                       PreciseRadians vehicle_psi);
    void WriteAHRSStatesToFile(PreciseSeconds OS_clock,
                               PreciseSeconds IMU_clock,
                               PreciseRadians phi_hat,
                               PreciseRadians theta_hat,
                               PreciseRadians psi_hat, 
                               PreciseRadians phiIMU,
                               PreciseRadians thetaIMU,
                               PreciseRadians psiIMU
                               );

 private:
    // std::ofstream delta_file_;
    // std::ofstream ef_file_;
    // std::ofstream vehicle_state_file_;
    std::ofstream localization_debug_file_;
    std::ofstream AHRS_debug_file_;
    bool control_module_dir_exist_;
    void CreateDebugFiles(std::filesystem::path path, bool localization_mode);
};
