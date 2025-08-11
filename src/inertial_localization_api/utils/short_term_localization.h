// Copyright (c) 2024 Imagry. All Rights Reserved.
// Unauthorized copying of this file, via any medium is strictly prohibited.
// Proprietary and confidential.

#ifndef INERTIAL_LOCALIZATION_SHORT_TERM_LOCALIZATION_H_
#define INERTIAL_LOCALIZATION_SHORT_TERM_LOCALIZATION_H_

#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include "inertial_localization_api/utils/sensor_types.h"
#include "inertial_localization_api/utils/units.h"

namespace inertial_localization_api {

struct WheelOdometrySample {
  double time_stamp = 0.0;
  PreciseMps rear_right_speed = 0.0;
  PreciseMps rear_left_speed = 0.0;
};

struct LocState {
  std::vector<PreciseMeters> pos = {0.0, 0.0};
  PreciseRadians psi = NAN;
  PreciseRadians delta = NAN;
  PreciseMps speed = NAN;

  bool IsValid() const {
    return !std::isnan(psi) && !std::isnan(delta) && !std::isnan(speed);
  }
};

class ShortTermLocalization {
public:
  ShortTermLocalization(const std::string &heading_update_mode,
                        const std::string &vehicle_speed_estimation_mode,
                        double vehicle_wheelbase);

  LocState State() const;
  LocState StateEnu() const;
  PreciseSeconds Clock() const;
  WheelOdometrySample *GetRearWheelsOdometry();

  void UpdateImu(const ImuSample &sample);
  void UpdateRearRightSpeed(PreciseMps rear_right_speed,
                            PreciseSeconds time_stamp);
  void UpdateRearLeftSpeed(PreciseMps rear_left_speed,
                           PreciseSeconds time_stamp);
  void UpdateSpeed(PreciseMps speed);
  void UpdateHeading(PreciseRadians psi);
  void UpdateDelta(PreciseRadians delta);
  void ResetVehicleState(PreciseSeconds clock, const LocState &state);
  void UpdateFrontAxlePosition(PreciseSeconds clock);

private:
  PreciseSeconds update_time_ = -1.0;
  LocState state_;
  ImuSample imu_;
  WheelOdometrySample wheel_odometry_;
  std::string vehicle_heading_estimation_mode_;
  std::string vehicle_speed_estimation_mode_;
  double vehicle_wheelbase_;
  mutable std::mutex mutex_;
};

} // namespace inertial_localization_api

#endif // INERTIAL_LOCALIZATION_SHORT_TERM_LOCALIZATION_H_
