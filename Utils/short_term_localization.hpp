/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#pragma once

#include <math.h>
#include <mutex>
#include <string>
#include <vector>

#include "units.hpp"

struct Vec3d {
    double x;
    double y;
    double z;
};
struct ImuSample {
    double time_stamp;
    Vec3d acc_;
    Vec3d acc_b_;
    Vec3d gyro_;
    Vec3d gyro_b_;
    Vec3d mag_;
    PreciseRadians pitch_;
    PreciseRadians roll_;
    PreciseRadians yaw_;
};

struct WheelOdometrySample {
    // Time stamp of last wheel speed update (either left or right)
    double time_stamp_;
    // Speed measurement from rear right wheel odometer
    PreciseMps rear_right_speed_ = 0.0;
    // Speed measurement from rear left wheel odometer
    PreciseMps rear_left_speed_ = 0.0;
};

struct LocState {
    std::vector<PreciseMeters> pos_ = {0, 0};
    PreciseRadians psi_ = nan("");
    PreciseRadians delta_ = nan("");
    PreciseMps speed_ = nan("");  // Estimated vehicle speed
};

class ShortTermLocalization {
 private:
    PreciseSeconds update_time_ = -1.0;
    LocState state_;
    ImuSample IMU_;
    WheelOdometrySample wheel_odometry_;
    // can be INS, IMU or steering_wheel
    std::string vehicle_heading_estimation_mode_;
    // Can be default, rear_average, kalman_first_order or kalman_second_order
    std::string vehicle_speed_estimation_mode_;
    double vehicle_wheelbase_;
    mutable std::mutex lock_;
 public:
    ShortTermLocalization(
        std::string heading_update_mode,
        std::string vehicle_speed_estimation_mode,
        double vehicle_wheelbase);

    LocState State() const;
    LocState StateENU() const;
    PreciseSeconds Clock() const;
    WheelOdometrySample* GetRearWheelsOdometry();

    void UpdateIMU(PreciseSeconds time_stamp, Mps2Precise acc_x,
                   Mps2Precise acc_y, Mps2Precise acc_z, Mps2Precise acc_x_b,
                   Mps2Precise acc_y_b, Mps2Precise acc_z_b,
                   PreciseRadians pitch, PreciseRadians roll,
                   PreciseRadians yaw, RadiansPerSec gyro_x,
                   RadiansPerSec gyro_y, RadiansPerSec gyro_z,
                   RadiansPerSec gyro_x_b, RadiansPerSec gyro_y_b,
                   RadiansPerSec gyro_z_b, Gauss mag_x, Gauss mag_y,
                   Gauss mag_z);
    void UpdateIMU(const ImuSample& sample);
    void UpdateRearRightSpeed(PreciseMps rear_right_speed,
        PreciseSeconds time_stamp);
    void UpdateRearLeftSpeed(PreciseMps rear_left_speed,
        PreciseSeconds time_stamp);
    // NOTE: UpdateSpeed will call the chosen speed estimator
    void UpdateSpeed(PreciseMps speed);
    void UpdateHeading(PreciseRadians psi);
    void UpdateDelta(PreciseRadians delta);
    void ResetVehicleState(PreciseSeconds clock,
                           const LocState& state);
    void UpdatePosition(PreciseSeconds clock);
    // void UpdateFrontAxlePosition(PreciseRadians delta, PreciseSeconds clock);
    void UpdateFrontAxlePosition(PreciseSeconds clock);
};
