// Copyright (c) 2024 Imagry. All Rights Reserved.
// Unauthorized copying of this file, via any medium is strictly prohibited.
// Proprietary and confidential.

#include "inertial_localization_api/utils/short_term_localization.h"

#include <iomanip>
#include <iostream>

namespace inertial_localization_api {

ShortTermLocalization::ShortTermLocalization(
    const std::string &heading_update_mode,
    const std::string &vehicle_speed_estimation_mode, double vehicle_wheelbase)
    : vehicle_heading_estimation_mode_(heading_update_mode),
      vehicle_speed_estimation_mode_(vehicle_speed_estimation_mode),
      vehicle_wheelbase_(vehicle_wheelbase) {}

LocState ShortTermLocalization::State() const {
  std::lock_guard<std::mutex> guard(mutex_);
  return state_;
}

LocState ShortTermLocalization::StateEnu() const {
  LocState state_ned = State();
  LocState state_enu = {{state_ned.pos[0], -state_ned.pos[1]},
                        -state_ned.psi,
                        state_ned.delta,
                        state_ned.speed};
  return state_enu;
}

PreciseSeconds ShortTermLocalization::Clock() const {
  std::lock_guard<std::mutex> guard(mutex_);
  return update_time_;
}

void ShortTermLocalization::UpdateImu(const ImuSample &sample) {
  std::lock_guard<std::mutex> guard(mutex_);
  imu_ = sample;
  if (vehicle_heading_estimation_mode_ == "INS") {
    state_.psi = imu_.yaw;
  }
}

void ShortTermLocalization::UpdateRearRightSpeed(PreciseMps rear_right_speed,
                                                 PreciseSeconds time_stamp) {
  std::lock_guard<std::mutex> guard(mutex_);
  wheel_odometry_.rear_right_speed = rear_right_speed;
  wheel_odometry_.time_stamp = time_stamp;
}

void ShortTermLocalization::UpdateRearLeftSpeed(PreciseMps rear_left_speed,
                                                PreciseSeconds time_stamp) {
  std::lock_guard<std::mutex> guard(mutex_);
  wheel_odometry_.rear_left_speed = rear_left_speed;
  wheel_odometry_.time_stamp = time_stamp;
}

void ShortTermLocalization::UpdateSpeed(PreciseMps speed) {
  std::lock_guard<std::mutex> guard(mutex_);
  state_.speed = speed;
}

void ShortTermLocalization::UpdateHeading(PreciseRadians psi) {
  std::lock_guard<std::mutex> guard(mutex_);
  if (vehicle_heading_estimation_mode_ == "IMU") {
    state_.psi = psi;
  }
}

void ShortTermLocalization::UpdateDelta(PreciseRadians delta) {
  std::lock_guard<std::mutex> guard(mutex_);
  state_.delta = delta;
}

void ShortTermLocalization::ResetVehicleState(PreciseSeconds clock,
                                              const LocState &state) {
  std::lock_guard<std::mutex> guard(mutex_);
  state_ = state;
  update_time_ = clock;
}

void ShortTermLocalization::UpdateFrontAxlePosition(PreciseSeconds clock) {
  std::lock_guard<std::mutex> guard(mutex_);
  double dt = 0;
  if (update_time_ >= 0) {
    dt = clock - update_time_;
  }
  update_time_ = clock;
  double dx = cos(state_.psi + state_.delta) * state_.speed * dt;
  double dy = sin(state_.psi + state_.delta) * state_.speed * dt;
  state_.pos[0] += dx;
  state_.pos[1] += dy;
  if (vehicle_heading_estimation_mode_ == "steering_wheel") {
    state_.psi += state_.speed * sin(state_.delta) / vehicle_wheelbase_ * dt;
  }
}

WheelOdometrySample *ShortTermLocalization::GetRearWheelsOdometry() {
  return &wheel_odometry_;
}

} // namespace inertial_localization_api
