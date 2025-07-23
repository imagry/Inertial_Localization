/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/

#include "short_term_localization.hpp"

#include <iomanip>
#include <iostream>

ShortTermLocalization::ShortTermLocalization(
    std::string heading_update_mode,
    std::string vehicle_speed_estimation_mode,
    double vehicle_wheelbase)
    : vehicle_heading_estimation_mode_(heading_update_mode),
    vehicle_speed_estimation_mode_(vehicle_speed_estimation_mode),
    vehicle_wheelbase_(vehicle_wheelbase) {
    std::cout << "short term localization created" << '\n';
    std::cout << " heading estimation mode is " <<
        vehicle_heading_estimation_mode_<< "\n";
    std::cout << "speed estimation mode is " <<
        vehicle_speed_estimation_mode_<< "\n";
}

LocState ShortTermLocalization::State() const {
    std::lock_guard<std::mutex> guard(lock_);
    return state_;
}

LocState ShortTermLocalization::StateENU() const {
    LocState state_NED = State();
    // TODO(myoresh): Use 3D matrix to properly convert NED to ENU.
    LocState state_ENU = {{state_NED.pos_[0], -state_NED.pos_[1]},
        -state_NED.psi_, state_NED.delta_, state_NED.speed_};
    return state_ENU;
}

PreciseSeconds ShortTermLocalization::Clock() const {
    std::lock_guard<std::mutex> guard(lock_);
    return update_time_;
}

void ShortTermLocalization::UpdateIMU(
    PreciseSeconds time_stamp, Mps2Precise acc_x, Mps2Precise acc_y,
    Mps2Precise acc_z, Mps2Precise acc_x_b, Mps2Precise acc_y_b,
    Mps2Precise acc_z_b, PreciseRadians pitch, PreciseRadians roll,
    PreciseRadians yaw, RadiansPerSec gyro_x, RadiansPerSec gyro_y,
    RadiansPerSec gyro_z, RadiansPerSec gyro_x_b, RadiansPerSec gyro_y_b,
    RadiansPerSec gyro_z_b, Gauss mag_x, Gauss mag_y, Gauss mag_z) {
    std::lock_guard<std::mutex> guard(lock_);
    this->IMU_.time_stamp = time_stamp;
    this->IMU_.acc_.x = acc_x;
    this->IMU_.acc_.y = acc_y;
    this->IMU_.acc_.z = acc_z;
    this->IMU_.acc_b_.x = acc_x_b;
    this->IMU_.acc_b_.y = acc_y_b;
    this->IMU_.acc_b_.z = acc_z_b;
    this->IMU_.pitch_ = pitch;
    this->IMU_.roll_ = roll;
    this->IMU_.yaw_ = yaw;
    this->IMU_.gyro_.x = gyro_x;
    this->IMU_.gyro_.y = gyro_y;
    this->IMU_.gyro_.z = gyro_z;
    this->IMU_.gyro_b_.x = gyro_x_b;
    this->IMU_.gyro_b_.y = gyro_y_b;
    this->IMU_.gyro_b_.z = gyro_z_b;
    this->IMU_.mag_.x = mag_x;
    this->IMU_.mag_.y = mag_y;
    this->IMU_.mag_.z = mag_z;
    if (vehicle_heading_estimation_mode_ == "INS") {
        // update the heading state with the IMU yaw state. in the future this
        // should be replaced with AHRS/EKF filtering
        state_.psi_ = this->IMU_.yaw_;
    }
}
void ShortTermLocalization::UpdateIMU(const ImuSample& sample) {
    std::lock_guard<std::mutex> guard(lock_);
    IMU_ = sample;
    if (vehicle_heading_estimation_mode_ == "INS") {
        // update the heading state with the IMU yaw state. in the future this
        // should be replaced with AHRS/EKF filtering
        state_.psi_ = this->IMU_.yaw_;
    }
}
void ShortTermLocalization::UpdateRearRightSpeed(
    PreciseMps rear_right_speed,
    PreciseSeconds time_stamp) {
    std::lock_guard<std::mutex> guard(lock_);
    wheel_odometry_.rear_right_speed_ = rear_right_speed;
    wheel_odometry_.time_stamp_ = time_stamp;
}
void ShortTermLocalization::UpdateRearLeftSpeed(
    PreciseMps rear_left_speed,
    PreciseSeconds time_stamp) {
    std::lock_guard<std::mutex> guard(lock_);
    wheel_odometry_.rear_left_speed_ = rear_left_speed;
    wheel_odometry_.time_stamp_ = time_stamp;
}
void ShortTermLocalization::UpdateSpeed(PreciseMps speed) {
    std::lock_guard<std::mutex> guard(lock_);
    // TODO(Dor): add switch statement over estimator
    // Update estimator, then set state_.speed_ to estimation
    state_.speed_ = speed;
    // std::cout << "SPEED SET IN LOC: " << speed << "\n";
}

void ShortTermLocalization::UpdateHeading(PreciseRadians psi) {
    std::lock_guard<std::mutex> guard(lock_);
    if (vehicle_heading_estimation_mode_ == "IMU") {
        //  this option is used to insert a heading angle calculated
        // by AHRS in th parent class
        state_.psi_ = psi;
        }
    }
void ShortTermLocalization::UpdateDelta(PreciseRadians delta) {
    std::lock_guard<std::mutex> guard(lock_);
    state_.delta_ = delta;
}
void ShortTermLocalization::ResetVehicleState(
    PreciseSeconds clock, const LocState& state) {
    std::lock_guard<std::mutex> guard(lock_);
    state_ = state;
    update_time_ = clock;
    // std::cout << "ShortTermLocalization::ResetVehicleState:"<< "\n";
    // std::cout << "State reset to: p = [" << state_.pos_[0] << ", " << 
    //                                         state_.pos_[1] << ", " << state_.psi_<<"]\n";
    // std::cout << "Time reset to: " << clock << "\n";
}
void ShortTermLocalization::UpdatePosition(PreciseSeconds clock) {
    // this method will be generalize to any control point along the wheelbase
    // for now dont use!!!!!!

    // double dt = clock - update_time_;
    // this->update_time_ = clock;
    // double dx = cos(psi_) * speed_ * dt;
    // double dy = sin(psi_) * speed_ * dt;
    // pos_[0] += dx;
    // pos_[1] += dy;
}
void ShortTermLocalization::UpdateFrontAxlePosition(PreciseSeconds clock) {
    std::lock_guard<std::mutex> guard(lock_);
    double dt = 0;
    if (update_time_ > 0) {
        dt = clock - update_time_;
    }
    this->update_time_ = clock;
    double dx = cos(state_.psi_ + state_.delta_) * state_.speed_ * dt;
    double dy = sin(state_.psi_ + state_.delta_) * state_.speed_ * dt;
    state_.pos_[0] += dx;
    state_.pos_[1] += dy;
    if (vehicle_heading_estimation_mode_ == "steering_wheel") {
        //  use bicycle modelto estimate change in heading
        state_.psi_ += state_.speed_ * sin(state_.delta_) / vehicle_wheelbase_ * dt;
    }
}

WheelOdometrySample* ShortTermLocalization::GetRearWheelsOdometry() {
    return &wheel_odometry_;
}
