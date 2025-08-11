// Copyright (c) 2024 Imagry. All Rights Reserved.
// Unauthorized copying of this file, via any medium is strictly prohibited.
// Proprietary and confidential.

#include "ahrs_loc_handler.h"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "utils/speed_estimator.h"

namespace inertial_localization_api {

void AhrsLocHandler::InitializeSpeedEstimator() {
  const std::string speed_estimation_mode =
      localization_config_["vehicle_speed_estimation_mode"].asString();
  if (speed_estimation_mode == "kalman") {
    speed_estimator_ = std::make_unique<KalmanFilterSpeedEstimator>(
        localization_config_["imu_acc_noise_density"].asDouble(),
        localization_config_["imu_acc_bias_instability"].asDouble(),
        localization_config_["wheel_speed_noise_std"].asDouble(),
        0.0, // initial_vehicle_speed
        0.0, // initial_imu_bias
        1.0, // initial_posterior_speed_std
        1.0  // initial_posterior_bias_std
    );
  } else if (speed_estimation_mode == "rear_average") {
    speed_estimator_ = std::make_unique<RearAverageSpeedEstimator>();
  } else {
    speed_estimator_ = nullptr;
  }
}

AhrsLocHandler::AhrsLocHandler(const Json::Value &vehicle_config,
                               const Json::Value &localization_config)
    : vehicle_config_(vehicle_config),
      localization_config_(localization_config),
      localization_obj_(
          localization_config_["vehicle_heading_estimation_mode"].asString(),
          localization_config_["vehicle_speed_estimation_mode"].asString(),
          vehicle_config_["WB"].asDouble()),
      ahrs_obj_(1.0 / static_cast<double>(
                          localization_config_["nominal_IMU_freq"].asDouble()),
                localization_config_["AHRS_gain"].asDouble(), "NED"),
      debug_mode_(localization_config_["debug_mode"].asBool()),
      debug_obj_(localization_config_["debug_mode"].asBool(),
                 localization_config_["control_modul_dir"].asString()) {
  InitializeSpeedEstimator();
}

void AhrsLocHandler::UpdateImu(const ImuSample &sample, PreciseSeconds clock) {
  localization_obj_.UpdateImu(sample);
  if (!ahrs_obj_.rotation_initialized_) {
    if (localization_config_["initialize_heading_from_IMU"].asBool()) {
      ahrs_obj_.InitializeRotation(sample.roll, sample.pitch, sample.yaw);
    } else {
      ahrs_obj_.InitializeRotation(sample.roll, sample.pitch, 0.0);
    }
    ahrs_obj_.rotation_initialized_ = true;
  }
  ahrs_obj_.GyroPromotion({sample.gyro.x, sample.gyro.y, sample.gyro.z},
                          sample.time_stamp);
  ahrs_obj_.UpdateGravity({sample.acc.x, sample.acc.y, sample.acc.z});
  if (localization_config_["vehicle_heading_estimation_mode"].asString() ==
      "IMU") {
    std::vector<double> euler = RotMat2Euler(ahrs_obj_.rnb_);
    UpdateHeading(euler[2], clock);
  }
  if (debug_mode_) {
    std::vector<double> euler = RotMat2Euler(ahrs_obj_.rnb_);
    debug_obj_.WriteAhrsStatesToFile(clock, sample.time_stamp, euler[0],
                                     euler[1], euler[2], sample.roll,
                                     sample.pitch, sample.yaw);
  }
  UpdatePosition(clock);
  if (speed_estimator_) {
    speed_estimator_->UpdateImu(&sample);
  }
}

void AhrsLocHandler::UpdateRearRightSpeed(PreciseMps rear_right_speed,
                                          PreciseSeconds clock) {
  localization_obj_.UpdateRearRightSpeed(
      rear_right_speed *
          localization_config_["speed_measurement_scale_factor"].asDouble(),
      clock);
  if (speed_estimator_) {
    speed_estimator_->UpdateRearSpeeds(
        localization_obj_.GetRearWheelsOdometry());
    EstimateSpeed(clock);
    UpdateSpeed(0.0, clock);
  }
}

void AhrsLocHandler::UpdateRearLeftSpeed(PreciseMps rear_left_speed,
                                         PreciseSeconds clock) {
  localization_obj_.UpdateRearLeftSpeed(
      rear_left_speed *
          localization_config_["speed_measurement_scale_factor"].asDouble(),
      clock);
  if (speed_estimator_) {
    speed_estimator_->UpdateRearSpeeds(
        localization_obj_.GetRearWheelsOdometry());
    EstimateSpeed(clock);
    UpdateSpeed(0.0, clock);
  }
}

void AhrsLocHandler::EstimateSpeed(PreciseSeconds clock) {
  if (speed_estimator_) {
    speed_estimator_->UpdateState(clock);
  }
}

void AhrsLocHandler::UpdateSpeed(PreciseMps speed, PreciseSeconds clock) {
  if (speed_estimator_ && speed_estimator_->IsEstimatedSpeedValid()) {
    localization_obj_.UpdateSpeed(speed_estimator_->GetEstimatedSpeed());
  } else {
    localization_obj_.UpdateSpeed(
        speed *
        localization_config_["speed_measurement_scale_factor"].asDouble());
  }
  UpdatePosition(clock);
}

void AhrsLocHandler::UpdateSteeringWheel(PreciseRadians steering_wheel_angle,
                                         PreciseSeconds clock) {
  const PreciseRadians delta =
      steering_wheel_angle /
      static_cast<double>(vehicle_config_["steering_ratio"].asDouble());
  localization_obj_.UpdateDelta(delta);
}

void AhrsLocHandler::UpdateHeading(PreciseRadians psi, PreciseSeconds clock) {
  localization_obj_.UpdateHeading(psi);
  UpdatePosition(clock);
}

bool AhrsLocHandler::UpdatePosition(PreciseSeconds clock) {
  LocState state = localization_obj_.State();

  if (!state.IsValid()) {
    std::cerr << "Warning: Invalid state detected in UpdatePosition. "
              << "psi=" << state.psi << ", speed=" << state.speed
              << ", delta=" << state.delta << std::endl;
    return false;
  }

  localization_obj_.UpdateFrontAxlePosition(clock);
  LocState updated_state = localization_obj_.State();

  if (debug_mode_) {
    debug_obj_.WriteLocalizationStatesToFile(
        clock, updated_state.speed, updated_state.delta, updated_state.pos[0],
        updated_state.pos[1], updated_state.psi);
  }

  return true;
}

void AhrsLocHandler::UpdateVehicleState(PreciseSeconds clock,
                                        const std::vector<double> &state) {
  if (state.size() < 4) {
    std::cerr << "Warning: Insufficient state data for vehicle state update"
              << std::endl;
    return;
  }

  LocState loc_state = {{state[0], state[1]}, state[2], 0, state[3]};
  localization_obj_.ResetVehicleState(clock, loc_state);
}

std::vector<PreciseMeters> AhrsLocHandler::GetPosition() const {
  return localization_obj_.State().pos;
}

PreciseRadians AhrsLocHandler::GetVehicleHeading() const {
  return localization_obj_.State().psi;
}

std::string AhrsLocHandler::GetVehicleHeadingEstimationMode() const {
  return localization_config_["vehicle_heading_estimation_mode"].asString();
}

} // namespace inertial_localization_api
