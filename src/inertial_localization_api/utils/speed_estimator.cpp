// Copyright (c) 2024 Imagry. All Rights Reserved.
// Unauthorized copying of this file, via any medium is strictly prohibited.
// Proprietary and confidential.

#include "inertial_localization_api/utils/speed_estimator.h"

#include <cmath>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "inertial_localization_api/utils/units.h"

namespace inertial_localization_api {

KalmanFilterSpeedEstimator::KalmanFilterSpeedEstimator(
    double imu_acc_noise_density, double imu_acc_bias_instability,
    double wheel_speed_noise_std, PreciseMps initial_vehicle_speed,
    Mps2Precise initial_imu_bias, double initial_posterior_speed_std,
    double initial_posterior_bias_std)
    : imu_acc_noise_density_(imu_acc_noise_density),
      imu_acc_bias_instability_(imu_acc_bias_instability),
      wheel_speed_noise_std_(wheel_speed_noise_std), r_(wheel_speed_noise_std) {
  q_ << 1e-6, 0.0, 0.0, pow(imu_acc_bias_instability, 2) / 2;
  x_ << initial_vehicle_speed, initial_imu_bias;
  p_ << initial_posterior_speed_std, 0.0, 0.0, initial_posterior_bias_std;
}

void KalmanFilterSpeedEstimator::UpdateImu(const ImuSample *sample) {
  std::lock_guard<std::mutex> lock(mutex_);
  imu_ = sample;
}

void KalmanFilterSpeedEstimator::UpdateRearSpeeds(
    const WheelOdometrySample *sample) {
  std::lock_guard<std::mutex> lock(mutex_);
  wheel_odometry_ = sample;
}

void KalmanFilterSpeedEstimator::UpdateState(PreciseSeconds clock) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!imu_ || !wheel_odometry_) {
    return;
  }
  double dt = clock - update_time_;
  if (dt < 1e-7) {
    return;
  }

  // Prediction
  Eigen::Matrix2d f;
  f << 1, -dt, 0, 1;
  Eigen::Vector2d b;
  b << dt, 0;
  q_(0, 0) = pow(imu_acc_noise_density_, 2) / dt;
  q_(1, 1) = pow(imu_acc_bias_instability_, 2) * dt;
  x_ = f * x_ + b * imu_->acc.x;
  p_ = f * p_ * f.transpose() + q_;

  // Update
  Eigen::Matrix<double, 1, 2> h;
  h << 1, 0;
  PreciseMps wheels_average_speed =
      (wheel_odometry_->rear_right_speed + wheel_odometry_->rear_left_speed) /
      2.0;
  double y = wheels_average_speed - h * x_;
  double s = h * p_ * h.transpose() + r_;
  Eigen::Matrix<double, 2, 1> k = p_ * h.transpose() / s;
  x_ = x_ + k * y;
  p_ = (Eigen::Matrix2d::Identity() - k * h) * p_;
  update_time_ = clock;
}

bool KalmanFilterSpeedEstimator::IsEstimatedSpeedValid() {
  return imu_ && wheel_odometry_ && update_time_ >= 0.0;
}

void KalmanFilterSpeedEstimator::GetState(Eigen::Vector2d &state,
                                          Eigen::Matrix2d &covariance) const {
  state = x_;
  covariance = p_;
}

PreciseMps KalmanFilterSpeedEstimator::GetEstimatedSpeed() { return x_(0); }

RearAverageSpeedEstimator::RearAverageSpeedEstimator()
    : wheel_odometry_(nullptr), update_time_(-1.0), estimated_speed_(0.0) {}

void RearAverageSpeedEstimator::UpdateImu(const ImuSample *sample) {}

void RearAverageSpeedEstimator::UpdateRearSpeeds(
    const WheelOdometrySample *sample) {
  std::lock_guard<std::mutex> lock(mutex_);
  wheel_odometry_ = sample;
}

void RearAverageSpeedEstimator::UpdateState(PreciseSeconds clock) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (wheel_odometry_) {
    estimated_speed_ =
        (wheel_odometry_->rear_right_speed + wheel_odometry_->rear_left_speed) /
        2.0;
    update_time_ = clock;
  }
}

PreciseMps RearAverageSpeedEstimator::GetEstimatedSpeed() {
  return estimated_speed_;
}

bool RearAverageSpeedEstimator::IsEstimatedSpeedValid() {
  return update_time_ >= 0.0;
}

} // namespace inertial_localization_api
