// Copyright (c) 2024 Imagry. All Rights Reserved.
// Unauthorized copying of this file, via any medium is strictly prohibited.
// Proprietary and confidential.

#ifndef INERTIAL_LOCALIZATION_LOCALIZATION_DEBUGGER_H_
#define INERTIAL_LOCALIZATION_LOCALIZATION_DEBUGGER_H_

#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>

#include "inertial_localization_api/utils/units.h"

namespace inertial_localization_api {

class LocalizationDebugger {
public:
  LocalizationDebugger() = default;
  LocalizationDebugger(bool create_debug_dir,
                       const std::filesystem::path &localization_module_dir);

  void WriteLocalizationStatesToFile(PreciseSeconds clock, PreciseMps car_speed,
                                     PreciseRadians steering_angle,
                                     PreciseMeters vehicle_x,
                                     PreciseMeters vehicle_y,
                                     PreciseRadians vehicle_psi);

  void WriteAhrsStatesToFile(PreciseSeconds os_clock, PreciseSeconds imu_clock,
                             PreciseRadians phi_hat, PreciseRadians theta_hat,
                             PreciseRadians psi_hat, PreciseRadians phi_imu,
                             PreciseRadians theta_imu, PreciseRadians psi_imu);

private:
  void CreateDebugFiles(const std::filesystem::path &path);

  std::ofstream localization_debug_file_;
  std::ofstream ahrs_debug_file_;
  bool localization_module_dir_exist_ = false;
  std::mutex mutex_;
};

} // namespace inertial_localization_api

#endif // INERTIAL_LOCALIZATION_LOCALIZATION_DEBUGGER_H_
