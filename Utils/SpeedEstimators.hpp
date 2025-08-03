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

/**
 * @brief Abstract base class for vehicle speed estimation.
 * 
 * This class defines the interface for different speed estimation strategies.
 * Each concrete implementation provides a specific algorithm for estimating
 * vehicle speed based on sensor data.
 */
class SpeedEstimator {
 public:
    virtual ~SpeedEstimator() = default;
    
    /**
     * @brief Updates the estimator with a new IMU sample.
     * 
     * @param sample Pointer to the new IMU sample
     */
    virtual void UpdateIMU(const ImuSample* sample) = 0;
    
    /**
     * @brief Updates the estimator with new wheel odometry data.
     * 
     * @param sample Pointer to the new wheel odometry sample
     */
    virtual void UpdateRearSpeeds(const WheelOdometrySample* sample) = 0;
    
    /**
     * @brief Estimates the vehicle state based on current sensor data.
     * 
     * @param clock Current timestamp
     */
    virtual void UpdateState(PreciseSeconds clock) = 0;
    
    /**
     * @brief Gets the current estimated vehicle speed.
     * 
     * @return PreciseMps Estimated speed in meters per second
     */
    virtual PreciseMps GetEstimatedSpeed() = 0;
    
    /**
     * @brief Checks if the current speed estimate is valid.
     * 
     * @return bool True if the estimate is valid, false otherwise
     */
    virtual bool IsEstimatedSpeedValid() = 0;
};

/**
 * @brief Kalman filter implementation for vehicle speed estimation.
 * 
 * This class implements a Kalman filter that fuses IMU acceleration
 * and wheel odometry measurements to estimate vehicle speed.
 * The state vector consists of vehicle speed and IMU bias.
 */
class KalmanFilter : public SpeedEstimator {
 private:
    double imu_acc_noise_density_;
    double imu_acc_bias_instability_;
    double wheel_speed_noise_std_;
    Eigen::Vector2d x_;  // State vector: [velocity, bias]
    Eigen::Matrix2d P_;  // State covariance
    Eigen::Matrix2d Q_;  // Process noise covariance
    double R_;   // Measurement noise covariance
    const ImuSample* IMU_;  // Last IMU sample
    const WheelOdometrySample* wheel_odometry_;   // Last wheel odometry sample
    PreciseSeconds update_time_ = -1.0;  // Last estimation time
    mutable std::mutex lock_;
 public:
    /**
     * @brief Constructs a Kalman filter with specified parameters.
     * 
     * @param imu_acc_noise_density IMU acceleration noise density
     * @param imu_acc_bias_instability IMU acceleration bias instability
     * @param wheel_speed_noise_std Wheel speed measurement noise standard deviation
     * @param initial_vehicle_speed Initial vehicle speed estimate
     * @param initial_imu_bias Initial IMU bias estimate
     * @param initial_posterior_speed_std Initial uncertainty in speed estimate
     * @param initial_posterior_bias_std Initial uncertainty in bias estimate
     */
    KalmanFilter(
        double imu_acc_noise_density,
        double imu_acc_bias_instability,
        double wheel_speed_noise_std,
        PreciseMps initial_vehicle_speed,
        Mps2Precise initial_imu_bias,
        double initial_posterior_speed_std,
        double initial_posterior_bias_std);
    
    /**
     * @brief Updates the filter with a new IMU sample.
     * 
     * @param sample Pointer to the new IMU sample
     */
    void UpdateIMU(const ImuSample* sample) override;
    
    /**
     * @brief Updates the filter with new wheel odometry data.
     * 
     * @param sample Pointer to the new wheel odometry sample
     */
    void UpdateRearSpeeds(const WheelOdometrySample* sample) override;
    
    /**
     * @brief Performs Kalman filter prediction and update steps.
     * 
     * @param clock Current timestamp
     */
    void UpdateState(PreciseSeconds clock) override;
    
    /**
     * @brief Gets the current state vector and covariance matrix.
     * 
     * @param state Output parameter for state vector [velocity, bias]
     * @param covariance Output parameter for state covariance matrix
     */
    void GetState(Eigen::Vector2d &state, Eigen::Matrix2d &covariance) const;
    
    /**
     * @brief Gets the current estimated vehicle speed.
     * 
     * @return PreciseMps Estimated speed in meters per second
     */
    PreciseMps GetEstimatedSpeed() override;
    
    /**
     * @brief Checks if the current speed estimate is valid.
     * 
     * @return bool True if the estimate is valid, false otherwise
     */
    bool IsEstimatedSpeedValid() override;
};

/**
 * @brief Simple speed estimator that averages rear wheel speeds.
 * 
 * This class implements a basic speed estimation strategy that
 * simply averages the speeds of the rear wheels to determine
 * the vehicle's forward velocity.
 */
class RearAverage : public SpeedEstimator {
 private:
    const WheelOdometrySample* wheel_odometry_;   // Last wheel odometry sample
    PreciseSeconds update_time_ = -1.0;  // Last estimation time
    PreciseMps estimated_speed_;
    mutable std::mutex lock_;
 public:
    /**
     * @brief Constructs a RearAverage speed estimator.
     */
    RearAverage();
    
    /**
     * @brief Updates with a new IMU sample (not used in this estimator).
     * 
     * @param sample Pointer to the new IMU sample
     */
    void UpdateIMU(const ImuSample* sample) override;
    
    /**
     * @brief Updates with new wheel odometry data.
     * 
     * @param sample Pointer to the new wheel odometry sample
     */
    void UpdateRearSpeeds(const WheelOdometrySample* sample) override;
    
    /**
     * @brief Computes the average of rear wheel speeds.
     * 
     * @param clock Current timestamp
     */
    void UpdateState(PreciseSeconds clock) override;
    
    /**
     * @brief Gets the current estimated vehicle speed.
     * 
     * @return PreciseMps Estimated speed in meters per second
     */
    PreciseMps GetEstimatedSpeed() override;
    
    /**
     * @brief Checks if the current speed estimate is valid.
     * 
     * @return bool True if the estimate is valid, false otherwise
     */
    bool IsEstimatedSpeedValid() override;
};