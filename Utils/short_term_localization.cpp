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

void ShortTermLocalization::UpdateIMU(const ImuSample& sample) {
    std::lock_guard<std::mutex> guard(lock_);
    IMU_ = sample;
    if (vehicle_heading_estimation_mode_ == "INS") {
        // update the heading state with the IMU yaw state. in the future this
        // should be replaced with AHRS/EKF filtering
        state_.psi_ = IMU_.yaw_;
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
    state_.speed_ = speed;
    
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
}

void ShortTermLocalization::UpdateFrontAxlePosition(PreciseSeconds clock) {
    std::lock_guard<std::mutex> guard(lock_);
    double dt = 0;
    if (update_time_ >= 0) {
        dt = clock - update_time_;
    }
    update_time_ = clock;
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
