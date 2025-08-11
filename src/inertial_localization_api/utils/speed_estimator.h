// Copyright (c) 2024 Imagry. All Rights Reserved.
// Unauthorized copying of this file, via any medium is strictly prohibited.
// Proprietary and confidential.

#ifndef INERTIAL_LOCALIZATION_SPEED_ESTIMATOR_H_
#define INERTIAL_LOCALIZATION_SPEED_ESTIMATOR_H_

#include <Eigen/Dense>
#include <mutex>

#include "inertial_localization_api/utils/sensor_types.h"
#include "inertial_localization_api/utils/short_term_localization.h"
#include "inertial_localization_api/utils/units.h"

namespace inertial_localization_api {

// Abstract base class for speed estimators.
class SpeedEstimator {
public:
  virtual ~SpeedEstimator() = default;

  virtual void UpdateImu(const ImuSample *sample) = 0;
  virtual void UpdateRearSpeeds(const WheelOdometrySample *sample) = 0;
  virtual void UpdateState(PreciseSeconds clock) = 0;
  virtual PreciseMps GetEstimatedSpeed() = 0;
  virtual bool IsEstimatedSpeedValid() = 0;
};

// Kalman filter-based speed estimator.
class KalmanFilterSpeedEstimator : public SpeedEstimator {
public:
  KalmanFilterSpeedEstimator(double imu_acc_noise_density,
                             double imu_acc_bias_instability,
                             double wheel_speed_noise_std,
                             PreciseMps initial_vehicle_speed,
                             Mps2Precise initial_imu_bias,
                             double initial_posterior_speed_std,
                             double initial_posterior_bias_std);

  void UpdateImu(const ImuSample *sample) override;
  void UpdateRearSpeeds(const WheelOdometrySample *sample) override;
  void UpdateState(PreciseSeconds clock) override;
  PreciseMps GetEstimatedSpeed() override;
  bool IsEstimatedSpeedValid() override;

  void GetState(Eigen::Vector2d &state, Eigen::Matrix2d &covariance) const;

private:
  double imu_acc_noise_density_;
  double imu_acc_bias_instability_;
  double wheel_speed_noise_std_;
  Eigen::Vector2d x_;
  Eigen::Matrix2d p_;
  Eigen::Matrix2d q_;
  double r_;
  const ImuSample *imu_ = nullptr;
  const WheelOdometrySample *wheel_odometry_ = nullptr;
  PreciseSeconds update_time_ = -1.0;
  mutable std::mutex mutex_;
};

// Speed estimator based on the average of the rear wheels' speed.
class RearAverageSpeedEstimator : public SpeedEstimator {
public:
  RearAverageSpeedEstimator();

  void UpdateImu(const ImuSample *sample) override;
  void UpdateRearSpeeds(const WheelOdometrySample *sample) override;
  void UpdateState(PreciseSeconds clock) override;
  PreciseMps GetEstimatedSpeed() override;
  bool IsEstimatedSpeedValid() override;

private:
  const WheelOdometrySample *wheel_odometry_ = nullptr;
  PreciseSeconds update_time_ = -1.0;
  PreciseMps estimated_speed_ = 0.0;
  mutable std::mutex mutex_;
};

} // namespace inertial_localization_api

#endif // INERTIAL_LOCALIZATION_SPEED_ESTIMATOR_H_
