/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Nov 28 2024 by Dor Siman Tov
*/
#pragma once
#include <iostream>
#include <math.h>
#include <mutex>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "short_term_localization.hpp"
#include "units.hpp"
// TODO(Dor): add ImuSample and WheelOdometrySample references

class SpeedEstimator {
 public:
    virtual ~SpeedEstimator() = default;
    // Update with a new IMU sample
    virtual void UpdateIMU(const ImuSample* sample) = 0;
    // Update with a new wheel odometry sample
    virtual void UpdateRearSpeeds(const WheelOdometrySample* sample) = 0;
    // Estimate new state
    virtual void UpdateState(
      PreciseSeconds clock) = 0;
    // Get estimated speed
    virtual PreciseMps GetEstimatedSpeed() = 0;
    virtual bool IsEstimatedSpeedValid() = 0;
};

class KalmanFilter : public SpeedEstimator {
 private:
    double IMU_ACC_NOISE_DENSITY_;
    double IMU_ACC_BIAS_INSTABILITY_;
    double WHEEL_SPEED_NOISE_STD_;
    Eigen::Vector2d x_;  // State vector: [velocity, bias]
    Eigen::Matrix2d P_;  // State covariance
    Eigen::Matrix2d Q_;  // Process noise covariance
    double R_;   // Measurement noise covariance
    const ImuSample* IMU_;  // Last IMU sample
    const WheelOdometrySample* wheel_odometry_;   // Last wheel odometry sample
    PreciseSeconds update_time_ = -1.0;  // Last estimation time
    mutable std::mutex lock_;
 public:
    // Constructor
    KalmanFilter(
        double imu_acc_noise_density,
        double imu_acc_bias_instability,
        double wheel_speed_noise_std,
        PreciseMps initial_vehicle_speed,
        Mps2Precise initial_imu_bias,
        double initial_posterior_speed_std,
        double initial_posterior_bias_std);
    // Set new IMU observation
    void UpdateIMU(const ImuSample* sample) override;
    // Set new wheel odometry observation
    void UpdateRearSpeeds(const WheelOdometrySample* sample) override;
    // Estimate new state
    void UpdateState(PreciseSeconds clock) override;
    // Get estimated state
    void GetState(Eigen::Vector2d &state, Eigen::Matrix2d &covariance) const;
    // Get estimated speed
    PreciseMps GetEstimatedSpeed() override;
    bool IsEstimatedSpeedValid() override;
};

class RearAverage : public SpeedEstimator {
 private:
    const WheelOdometrySample* wheel_odometry_;   // Last wheel odometry sample
    PreciseSeconds update_time_ = -1.0;  // Last estimation time
    PreciseMps estimated_speed_;
    mutable std::mutex lock_;
 public:
    // Constructor
    RearAverage();
    // Set new IMU observation
    void UpdateIMU(const ImuSample* sample) override;
    // Set new wheel odometry observation
    void UpdateRearSpeeds(const WheelOdometrySample* sample) override;
    // Estimate new state
    void UpdateState(PreciseSeconds clock) override;
    // Get estimated speed
    PreciseMps GetEstimatedSpeed() override;
    bool IsEstimatedSpeedValid() override;
};