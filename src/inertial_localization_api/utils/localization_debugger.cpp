// Copyright (c) 2024 Imagry. All Rights Reserved.
// Unauthorized copying of this file, via any medium is strictly prohibited.
// Proprietary and confidential.

#include "inertial_localization_api/utils/localization_debugger.h"

#include <filesystem>
#include <iostream>
#include <string>

#include "inertial_localization_api/utils/data_handling.h"

namespace inertial_localization_api {

LocalizationDebugger::LocalizationDebugger(
    bool create_debug_dir,
    const std::filesystem::path &localization_module_dir) {
  if (create_debug_dir) {
    localization_module_dir_exist_ =
        std::filesystem::exists(localization_module_dir);
    if (localization_module_dir_exist_) {
      std::filesystem::path temp_results_dir_path =
          localization_module_dir / "data" / "temp_results";
      if (!std::filesystem::exists(temp_results_dir_path)) {
        std::filesystem::create_directory(temp_results_dir_path);
      }
      std::filesystem::path current_run_debug_dir =
          temp_results_dir_path / WhatsTheTimeString();
      std::filesystem::create_directory(current_run_debug_dir);
      CreateDebugFiles(current_run_debug_dir);
    } else {
      std::cerr << "Localization module directory does not exist: "
                << localization_module_dir << std::endl;
    }
  }
}

void LocalizationDebugger::CreateDebugFiles(const std::filesystem::path &path) {
  localization_debug_file_.open(path / "debug_localization.csv");
  localization_debug_file_
      << "time [sec],vel [mps],steering [rad],x [m],y [m],psi [rad]"
      << std::endl;

  ahrs_debug_file_.open(path / "debug_ahrs.csv");
  ahrs_debug_file_
      << "time_IMU,time_OS,phi_hat,theta_hat,psi_hat,phiIMU,thetaIMU,psiIMU"
      << std::endl;
}

void LocalizationDebugger::WriteLocalizationStatesToFile(
    PreciseSeconds clock, PreciseMps car_speed, PreciseRadians steering_angle,
    PreciseMeters vehicle_x, PreciseMeters vehicle_y,
    PreciseRadians vehicle_psi) {
  if (localization_module_dir_exist_) {
    std::lock_guard<std::mutex> guard(mutex_);
    localization_debug_file_ << clock << "," << car_speed << ","
                             << steering_angle << "," << vehicle_x << ","
                             << vehicle_y << "," << vehicle_psi << std::endl;
  }
}

void LocalizationDebugger::WriteAhrsStatesToFile(
    PreciseSeconds os_clock, PreciseSeconds imu_clock, PreciseRadians phi_hat,
    PreciseRadians theta_hat, PreciseRadians psi_hat, PreciseRadians phi_imu,
    PreciseRadians theta_imu, PreciseRadians psi_imu) {
  if (localization_module_dir_exist_) {
    std::lock_guard<std::mutex> guard(mutex_);
    ahrs_debug_file_ << imu_clock << "," << os_clock << "," << phi_hat << ","
                     << theta_hat << "," << psi_hat << "," << phi_imu << ","
                     << theta_imu << "," << psi_imu << std::endl;
  }
}

} // namespace inertial_localization_api
