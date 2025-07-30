/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/

#include "ahrs_loc_handler.hpp"
#include "Utils/SpeedEstimators.hpp"

AHRSLocHandler::AHRSLocHandler(const json& vehicle_config,
                               const json& localization_config)
    : vehicle_config_(vehicle_config),
      localization_config_(localization_config),
      localization_obj_(localization_config_["vehicle_heading_estimation_mode"],
                        localization_config["vehicle_speed_estimation_mode"],
                        vehicle_config_["WB"]),
      AHRS_obj_(AttitudeEstimator(
                1 / static_cast<double>(localization_config_["nominal_IMU_freq"]),
                localization_config_["AHRS_gain"],
                (std::string)"NED")),
      debug_mode_(localization_config_["debug_mode"]),
      debug_obj_(localization_config_["debug_mode"],
                 localization_config_["control_modul_dir"]) {
        // Speed estimation modes:
        //  - "kalman" for Kalman filter using rear-wheels speeds and IMU
        //  - "rear_average" for the average of rear-wheels speeds
        //  - "default" (or anything else) for default wheel odometry
        if (localization_config_["vehicle_speed_estimation_mode"] == "kalman") {
            speed_estimator_ = std::make_unique<KalmanFilter>(
                static_cast<double>(
                    localization_config_["imu_acc_noise_density"]),
                static_cast<double>(
                    localization_config_["imu_acc_bias_instability"]),
                static_cast<double>(localization_config_["wheel_speed_noise_std"]),
                static_cast<PreciseMps>(0.0),
                static_cast<Mps2Precise>(0.0),
                static_cast<double>(1.0),
                static_cast<double>(1.0)
            );
        } else if (
            localization_config_[
                "vehicle_speed_estimation_mode"] == "rear_average") {
                speed_estimator_ = std::make_unique<RearAverage>();
            } else {
                speed_estimator_ = nullptr;
            }
    }
AHRSLocHandler::AHRSLocHandler(const std::string& vehicle_config_path,
                        const std::string& control_config_path)
    : vehicle_config_(json::parse(std::ifstream(vehicle_config_path))),
      localization_config_(json::parse(std::ifstream(control_config_path))),
      localization_obj_(
        localization_config_["vehicle_heading_estimation_mode"],
        localization_config_["vehicle_speed_estimation_mode"],
        vehicle_config_["WB"]),
      AHRS_obj_(AttitudeEstimator(
                1 / static_cast<double>(localization_config_["nominal_IMU_freq"]),
                localization_config_["AHRS_gain"],
                (std::string)"NED")),
      debug_mode_(localization_config_["debug_mode"]),
      debug_obj_(localization_config_["debug_mode"],
                 localization_config_["control_modul_dir"]) {
        // Speed estimation modes:
        //  - "kalman" for Kalman filter using rear-wheels speeds and IMU
        //  - "rear_average" for the average of rear-wheels speeds
        //  - "default" (or anything else) for default wheel odometry
        if (localization_config_["vehicle_speed_estimation_mode"] == "kalman") {
            speed_estimator_ = std::make_unique<KalmanFilter>(
                static_cast<double>(
                    localization_config_["imu_acc_noise_density"]),
                static_cast<double>(
                    localization_config_["imu_acc_bias_instability"]),
                static_cast<double>(localization_config_["wheel_speed_noise_std"]),
                static_cast<double>(0.0),
                static_cast<double>(0.0),
                static_cast<double>(1.0),
                static_cast<double>(1.0)
            );
        } else if (
            localization_config_[
                "vehicle_speed_estimation_mode"] == "rear_average") {
                speed_estimator_ = std::make_unique<RearAverage>();
            } else {
                speed_estimator_ = nullptr;
            }
    }

void AHRSLocHandler::UpdateIMU(const ImuSample& sample, PreciseSeconds clock) {
    localization_obj_.UpdateIMU(sample);    // this is used by aidriver,
    // the estimated heading angle is updated by the INS estimation if
    // vehicle_heading_estimation_mode == "INS"
    // option is chosen in the localization_config.json file
    if (!AHRS_obj_.rotation_initialized_) {
        if (localization_config_["initialize_heading_from_IMU"]){
            AHRS_obj_.InitializeRotation(sample.roll_, sample.pitch_, sample.yaw_);
        }
        else{
            AHRS_obj_.InitializeRotation(sample.roll_, sample.pitch_, 0.0);
        }
        AHRS_obj_.rotation_initialized_ = true;
    }
    AHRS_obj_.GyroPromotion({sample.gyro_.x, sample.gyro_.y, sample.gyro_.z},
                            sample.time_stamp);
    //notice gyro integration is done using imu sample timestamp
    AHRS_obj_.UpdateGravity({sample.acc_.x, sample.acc_.y, sample.acc_.z});
    if (localization_config_["vehicle_heading_estimation_mode"] == "IMU") {
        vector<double> euler = rot_mat2euler(AHRS_obj_.Rnb);
        UpdateHeading(euler[2], clock);
        /*this function updates the localization heading and updates the position,
        the used clock is the OS clock */

    }
    if (debug_mode_) {
        vector<double> euler = rot_mat2euler(AHRS_obj_.Rnb);
        debug_obj_.WriteAHRSStatesToFile(clock, sample.time_stamp, 
            euler[0], euler[1], euler[2], 
            sample.roll_, sample.pitch_, sample.yaw_);
    }
    UpdatePosition(clock);//OS clock
    if (speed_estimator_) {
        speed_estimator_->UpdateIMU(&sample);
    }
}
void AHRSLocHandler::UpdateRearRightSpeed(
    PreciseMps rear_right_speed,
    PreciseSeconds clock) {
    localization_obj_.UpdateRearRightSpeed(rear_right_speed * 
        double(localization_config_["speed_measurement_scale_factor"]), clock);
    if (speed_estimator_) {
        speed_estimator_->UpdateRearSpeeds(
            localization_obj_.GetRearWheelsOdometry());// update a measurement to the estimator
        EstimateSpeed(clock); // update the estimation
        UpdateSpeed(0.0, clock);// pass the estimated speed to the localization object
    }
}
void AHRSLocHandler::UpdateRearLeftSpeed(
    PreciseMps rear_left_speed,
    PreciseSeconds clock) {
    localization_obj_.UpdateRearLeftSpeed(rear_left_speed * 
        double(localization_config_["speed_measurement_scale_factor"]), clock);
    if (speed_estimator_) {
        speed_estimator_->UpdateRearSpeeds(
            localization_obj_.GetRearWheelsOdometry());
        EstimateSpeed(clock); // update the estimation
        UpdateSpeed(0.0, clock);// pass the estimated speed to the localization object
    }
}

/*
NOTE: EstimateSpeed() only calls an update step for the estimator.
It DOES NOT update the speed in the localization object - this is done by
UpdateSpeed().
*/
void AHRSLocHandler::EstimateSpeed(PreciseSeconds clock) {
    if (speed_estimator_) {
        speed_estimator_->UpdateState(clock);
    }
}

/*
NOTE: UpdateSpeed() now takes a low-resolution odometry measurement but
estimates and sets current vehicle speed using OTHER measurements (rear wheels
and IMU acceleration).
Only if estimation is unavailable or DefaultSpeed estimator was chosen in
localization_config.json, current speed is set to the provided low-resolution value.
*/
void AHRSLocHandler::UpdateSpeed(PreciseMps speed, PreciseSeconds clock) {
    if (!speed_estimator_)
    { // use speed odometry 
        localization_obj_.UpdateSpeed(speed * 
            double(localization_config_["speed_measurement_scale_factor"]));
        std::lock_guard<std::mutex> guard(debug_obj_lock_);
        debug_obj_.vehicle_speed_.push_back(std::make_pair(speed, clock));
    } else { // this call is followed by a rear wheel update
        localization_obj_.UpdateSpeed(speed_estimator_->GetEstimatedSpeed());
        std::lock_guard<std::mutex> guard(debug_obj_lock_);
        debug_obj_.vehicle_speed_.push_back(std::make_pair(
            speed_estimator_->GetEstimatedSpeed(), clock));
    }
    UpdatePosition(clock);
}

void AHRSLocHandler::UpdateSteeringWheel(PreciseRadians steering_wheel_angle,
                                     PreciseSeconds clock) {
    PreciseRadians delta =
        steering_wheel_angle / static_cast<double>(
            vehicle_config_["steering_ratio"]);
    localization_obj_.UpdateDelta(delta);
}

void AHRSLocHandler::UpdateHeading(PreciseRadians psi, PreciseSeconds clock)  {
    localization_obj_.UpdateHeading(psi);
    UpdatePosition(clock);
}

bool AHRSLocHandler::UpdatePosition(PreciseSeconds clock) {
    std::lock_guard<std::mutex> guard(UpdatePosition_lock_);
    LocState state = localization_obj_.State();
    if (!std::isnan(state.psi_) &&
        !std::isnan(state.speed_) &&
        !std::isnan(state.delta_)) {
        localization_obj_.UpdateFrontAxlePosition(clock);
        std::vector<double> x_y_psi{state.pos_[0], state.pos_[1], state.psi_};
        std::pair<std::vector<double>, double>
            vehicle_state_sample;  // <<x,y,psi>, time>
        vehicle_state_sample.first = x_y_psi;
        vehicle_state_sample.second = clock;

        std::pair<PreciseRadians, PreciseSeconds>
            vehicle_heading_sample;  // <psi, time>
        vehicle_heading_sample.first = state.psi_;
        vehicle_heading_sample.second = clock;
        {
            std::lock_guard<std::mutex> guard(debug_obj_lock_);
            debug_obj_.vehicle_state_.push_back(vehicle_state_sample);
        }
        if (debug_mode_) {
            debug_obj_.WriteLocalizationStatesToFile(
                clock, state.speed_, state.delta_, state.pos_[0],
                state.pos_[1], state.psi_);
        }
        return true;
    } else {
        return false;
    }
}

void AHRSLocHandler::UpdateVehicleState(PreciseSeconds clock,
                                   const std::vector<double>& state) {
    localization_obj_.ResetVehicleState(clock, {{state[0], state[1]},
                                                 state[2], 0, state[3]});                                            
}
void AHRSLocHandler::ResetVehicleState(PreciseSeconds clock,
                                    const std::vector<double>& state) {
    localization_obj_.ResetVehicleState(clock, {{state[0], state[1]},
                                            state[2], 0, state[3]});                                                  
}
std::vector<PreciseMeters> AHRSLocHandler::GetPosition() const {
    return localization_obj_.State().pos_;
}
PreciseRadians AHRSLocHandler::GetVehicleHeading() const {
    return localization_obj_.State().psi_;
}
std::string AHRSLocHandler::GetVehicleHeadingEstimationMode() const{
    return localization_config_["vehicle_heading_estimation_mode"];
}