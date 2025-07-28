/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/

#include "control_debug_states.hpp"

#include <iostream>

#include "DataHandling.hpp"

ControlDebugStates::ControlDebugStates(
    bool create_debug_dir, bool localization_mode,
    std::filesystem::path control_module_dir) {
    /* B10: Removed as part of control code removal
    motion_planning_num_updates_ = 0;
    steering_num_updates_ = 0;
    */
    if (create_debug_dir) {
        // if control_module_dir doesn't exist dont try saving files
        control_module_dir_exist_ = std::filesystem::exists(
            control_module_dir);
        if (control_module_dir_exist_) {
            std::filesystem::path temp_results_dir_path =
                control_module_dir / "data/temp_results";
            // data dir is backed in repo,
            // if temp_results dir doesn't exist create it
            if (!std::filesystem::exists(temp_results_dir_path)) {
                std::cout << "Directory does not exist." << std::endl;
                (std::filesystem::create_directory(temp_results_dir_path));
                std::cout << temp_results_dir_path << " Created" << std::endl;
            }
            std::filesystem::path current_run_debug_dir =
                        temp_results_dir_path / WhatsTheTimeString();
            std::filesystem::create_directory(current_run_debug_dir);
            std::cout << current_run_debug_dir << " Created" << std::endl;
            CreateDebugFiles(current_run_debug_dir, localization_mode);
        } else {
            std::cout << "control module dir from json file doesn't exist, not "
                         "saving debug data"
                      << "\n";
            PrintCurrentPath();
        }
    } 
}
void ControlDebugStates::CreateDebugFiles(std::filesystem::path path,
                                          bool localization_mode) {
    localization_debug_file_.open(path / "debug_localization.csv");
    localization_debug_file_ << "time [sec],"
                            << "vel [mps],"
                            << "steering [rad],"
                            << "x [m]"
                            << ","
                            << "y [m]"
                            << ","
                            << "psi [rad]"
                            << "\n";  // <<t,v,delta,x,y,psi>
    AHRS_debug_file_.open(path / "debug_AHRS.csv");
    AHRS_debug_file_
        << "time_IMU,"
        << "time_OS,"
        << "phi_hat,"
        << "theta_hat,"
        << "psi_hat,"
        << "phiIMU,"
        << "thetaIMU,"
        << "psiIMU,"
        << "\n";  // <<time,phi_hat,theta_hat,psi_hat>
    std::cout << "created " << (path / "debug_AHRS.csv").string()
                << "\n";
    
}

void ControlDebugStates::WriteLocalizationStatesToFile(
    PreciseSeconds clock, PreciseMps car_speed, PreciseRadians steering_angle,
    PreciseMeters vehicle_x, PreciseMeters vehicle_y,
    PreciseRadians vehicle_psi) {
    if (control_module_dir_exist_) {
        std::lock_guard<std::mutex> guard(lock_);
        localization_debug_file_ << std::to_string(clock) << "," << car_speed
                                 << "," << steering_angle << ","
                                 << std::to_string(vehicle_x) << ","
                                 << std::to_string(vehicle_y) << ","
                                 << vehicle_psi
                                 << ","
                                 << "\n";
    }
}
void ControlDebugStates::WriteAHRSStatesToFile(PreciseSeconds OS_clock,
                                               PreciseSeconds IMU_clock,
                                               PreciseRadians phi_hat,
                                               PreciseRadians theta_hat,
                                               PreciseRadians psi_hat, 
                                               PreciseRadians phiIMU,
                                               PreciseRadians thetaIMU,
                                               PreciseRadians psiIMU) {
    if (control_module_dir_exist_) {
        std::lock_guard<std::mutex> guard(lock_);
        AHRS_debug_file_ << std::to_string(IMU_clock) << ","
                         << std::to_string(OS_clock) << ","
                         << std::to_string(phi_hat) << ","
                         << std::to_string(theta_hat) << ","
                         << std::to_string(psi_hat) << ","
                         << std::to_string(phiIMU) << ","
                         << std::to_string(thetaIMU) << ","
                         << std::to_string(psiIMU) << ","
                         << "\n";
    }
}

