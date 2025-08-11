// Copyright (c) 2024 Imagry. All Rights Reserved.
// Unauthorized copying of this file, via any medium is strictly prohibited.
// Proprietary and confidential.

#ifndef INERTIAL_LOCALIZATION_AHRS_LOC_HANDLER_H_
#define INERTIAL_LOCALIZATION_AHRS_LOC_HANDLER_H_

#include <json/json.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "utils/attitude_estimator.h"
#include "utils/localization_debugger.h"
#include "utils/sensor_types.h"
#include "utils/short_term_localization.h"
#include "utils/speed_estimator.h"
#include "utils/units.h"

namespace inertial_localization_api {

// Handles the localization process by integrating AHRS data.
class AhrsLocHandler {
public:
  AhrsLocHandler(const Json::Value &vehicle_config,
                 const Json::Value &localization_config);

  // Returns the short-term localization state.
  const ShortTermLocalization &GetLoc() const { return localization_obj_; }

  // Updates the position estimate. Returns true on success.
  bool UpdatePosition(PreciseSeconds clock);

  // Updates the handler with a new IMU sample.
  void UpdateImu(const ImuSample &sample, PreciseSeconds clock);

  // Updates the handler with a new speed measurement.
  void UpdateSpeed(PreciseMps speed, PreciseSeconds clock);

  // Updates the handler with a new rear-right wheel speed measurement.
  void UpdateRearRightSpeed(PreciseMps rear_right_speed, PreciseSeconds clock);

  // Updates the handler with a new rear-left wheel speed measurement.
  void UpdateRearLeftSpeed(PreciseMps rear_left_speed, PreciseSeconds clock);

  // Estimates the speed based on the current sensor data.
  void EstimateSpeed(PreciseSeconds clock);

  // Updates the handler with a new steering wheel angle measurement.
  void UpdateSteeringWheel(PreciseRadians steering_wheel_angle,
                           PreciseSeconds clock);

  // Updates the heading of the vehicle.
  void UpdateHeading(PreciseRadians psi, PreciseSeconds clock);

  // Sets or resets the vehicle state.
  void UpdateVehicleState(PreciseSeconds clock,
                          const std::vector<double> &state);

  // Returns the current position estimate.
  std::vector<PreciseMeters> GetPosition() const;

  // Returns the current vehicle heading estimate.
  PreciseRadians GetVehicleHeading() const;

  // Returns the heading estimation mode.
  std::string GetVehicleHeadingEstimationMode() const;

private:
  // Initializes the speed estimator based on the configuration.
  void InitializeSpeedEstimator();

  Json::Value vehicle_config_;
  Json::Value localization_config_;

  ShortTermLocalization localization_obj_;
  AttitudeEstimator ahrs_obj_;
  std::unique_ptr<SpeedEstimator> speed_estimator_;
  std::mutex debug_obj_lock_;
  std::mutex update_position_lock_;
  bool debug_mode_;
  LocalizationDebugger debug_obj_;
};

} // namespace inertial_localization_api

#endif // INERTIAL_LOCALIZATION_AHRS_LOC_HANDLER_H_
