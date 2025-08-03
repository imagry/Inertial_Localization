/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Nov 28 2024 by Dor Siman Tov
*/
#include "SpeedEstimators.hpp"

#include <cmath>  // instead of math.h

#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "units.hpp"

KalmanFilter::KalmanFilter(
        double imu_acc_noise_density,
        double imu_acc_bias_instability,
        double wheel_speed_noise_std,
        PreciseMps initial_vehicle_speed,
        Mps2Precise initial_imu_bias,
        double initial_posterior_speed_std,
        double initial_posterior_bias_std) :
        imu_acc_noise_density_(imu_acc_noise_density),
        imu_acc_bias_instability_(imu_acc_bias_instability),
        wheel_speed_noise_std_(wheel_speed_noise_std),
        R_(wheel_speed_noise_std),
        IMU_(nullptr),
        wheel_odometry_(nullptr),
        update_time_(-1.0)
         {
            // Just the initial Q matrix
            // Note that Q generally depends on dt (time interval
            // since the previous step of the filter)
            Q_ << 1e-6, 0.0,
                  0.0, pow(imu_acc_bias_instability, 2) / 2;
            x_ << initial_vehicle_speed, initial_imu_bias;
            P_ << initial_posterior_speed_std, 0.0,
                  0.0, initial_posterior_bias_std;
         }

// Set new IMU observation
void KalmanFilter::UpdateIMU(const ImuSample* sample) {
    std::lock_guard<std::mutex> lock(lock_);
    IMU_ = sample;
}

// Set new wheel odometry observation
void KalmanFilter::UpdateRearSpeeds(const WheelOdometrySample* sample) {
    std::lock_guard<std::mutex> lock(lock_);
    wheel_odometry_ = sample;
}

// Estimate new state
void KalmanFilter::UpdateState(PreciseSeconds clock) {
    std::lock_guard<std::mutex> lock(lock_);
    double dt;
    if (!IMU_ || !wheel_odometry_) {
        return;
    }
    dt = clock - update_time_;
    // TODO(Dor): add const to config
    if (dt < 1e-7) {
        return;
    }

    // Prediction
    Eigen::Matrix2d F;
    F << 1, -dt,
         0, 1;
    Eigen::Vector2d B;
    B << dt, 0;
    Q_(0, 0) = pow(imu_acc_noise_density_, 2) / dt;
    Q_(1, 1) = pow(imu_acc_bias_instability_, 2) * dt;
    x_ = F * x_ + B * IMU_->acc_.x;
    P_ = F * P_ * F.transpose() + Q_;

    // Update
    Eigen::Matrix<double, 1, 2> H;
    H << 1, 0;
    PreciseMps wheels_average_speed = (
        wheel_odometry_->rear_right_speed_ +
        wheel_odometry_->rear_left_speed_) / 2.0;
    double y = wheels_average_speed - H * x_;
    double S = H * P_ * H.transpose() + R_;
    Eigen::Matrix<double, 2, 1> K = P_ * H.transpose() / S;
    x_ = x_ + K * y;
    P_ = (Eigen::Matrix2d::Identity() - K * H) * P_;
    update_time_ = clock;
}

bool KalmanFilter::IsEstimatedSpeedValid() {
    return IMU_ && wheel_odometry_ && update_time_ >= 0.0;
}

// Get estimated state
void KalmanFilter::GetState(
    Eigen::Vector2d &state,
    Eigen::Matrix2d &covariance) const {
        state = x_;
        covariance = P_;
}

// Get estimated speed
PreciseMps KalmanFilter::GetEstimatedSpeed() {
    return x_(0);
}

RearAverage::RearAverage() :
        wheel_odometry_(nullptr),
        estimated_speed_(0.0),
        update_time_(-1.0) {}

// Set new IMU observation - irrelevant for RearAverage
void RearAverage::UpdateIMU(const ImuSample* sample) {
    return;
}

// Set new wheel odometry observation
void RearAverage::UpdateRearSpeeds(const WheelOdometrySample* sample) {
    std::lock_guard<std::mutex> lock(lock_);
    wheel_odometry_ = sample;
    // std::cout << "LEFT REAR SPEED IN ESTIMATOR:" << wheel_odometry_->rear_left_speed_ << "\n";
    // std::cout << "RIGHT REAR SPEED IN ESTIMATOR:" << wheel_odometry_->rear_right_speed_ << "\n";
}

// Estimate new state (speed only)
void RearAverage::UpdateState(PreciseSeconds clock) {
    std::lock_guard<std::mutex> lock(lock_);
    if (wheel_odometry_) {
        estimated_speed_ = (
            wheel_odometry_->rear_right_speed_ +
            wheel_odometry_->rear_left_speed_) / 2.0; 
        update_time_ = clock;
    }
    // std::cout << "ESTIMATED SPEED INSIDE ESTIMATOR: " << estimated_speed_ << "\n";
}

// Get estimated speed
PreciseMps RearAverage::GetEstimatedSpeed() {
    return estimated_speed_;
}

bool RearAverage::IsEstimatedSpeedValid() {
    return update_time_ >= 0.0;
}