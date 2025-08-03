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

class LocalizationDebugStates {
    // attributes
 public:
    // methods
    std::mutex lock_;
    LocalizationDebugStates() {}
    LocalizationDebugStates(bool create_debug_dir,
                       std::filesystem::path localization_module_dir);
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
    std::ofstream localization_debug_file_;
    std::ofstream AHRS_debug_file_;
    bool localization_module_dir_exist_;
    void CreateDebugFiles(std::filesystem::path path);
};
