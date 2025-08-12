/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/

#include "ahrs_loc_handler.hpp"
#include "Utils/SpeedEstimators.hpp"

// Private helper method to initialize speed estimator based on config
void AHRSLocHandler::InitializeSpeedEstimator() {
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
        localization_config_["vehicle_speed_estimation_mode"] == "rear_average") {
        speed_estimator_ = std::make_unique<RearAverage>();
    } else {
        speed_estimator_ = nullptr;
    }
}

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
                 localization_config_["control_modul_dir"]),
      static_dynamic_test_obj_(localization_config_)
{
    InitializeSpeedEstimator();
}
AHRSLocHandler::AHRSLocHandler(const std::string& vehicle_config_path,
                        const std::string& localization_config_path)
    : vehicle_config_(json::parse(std::ifstream(vehicle_config_path))),
      localization_config_(json::parse(std::ifstream(localization_config_path))),
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
                 localization_config_["control_modul_dir"]),
      static_dynamic_test_obj_(localization_config_)
{
    InitializeSpeedEstimator();
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

    // Update static/dynamic test with IMU sample
    static_dynamic_test_obj_.UpdateIMU(sample);

    if (localization_config_["vehicle_heading_estimation_mode"] == "IMU") {
        vector<double> euler = Rot_mat2euler(AHRS_obj_.Rnb_);
        UpdateHeading(euler[2], clock);
        /*this function updates the localization heading and updates the position,
        the used clock is the OS clock */

    }
    if (debug_mode_) {
        vector<double> euler = Rot_mat2euler(AHRS_obj_.Rnb_);
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

        // Update static/dynamic test with measured car speed
        static_dynamic_test_obj_.UpdateCarSpeed(speed);
    } else { // this call is followed by a rear wheel update
        localization_obj_.UpdateSpeed(speed_estimator_->GetEstimatedSpeed());

        // Update static/dynamic test with estimated car speed
        static_dynamic_test_obj_.UpdateCarSpeed(speed_estimator_->GetEstimatedSpeed());
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
    
    // Check for NaN values in critical state variables
    bool has_valid_state = !std::isnan(state.psi_) &&
                           !std::isnan(state.speed_) &&
                           !std::isnan(state.delta_);
                           
    if (!has_valid_state) {
        std::cerr << "Warning: Invalid state detected in UpdatePosition. "
                  << "psi=" << state.psi_ << ", speed=" << state.speed_ 
                  << ", delta=" << state.delta_ << std::endl;
        return false;
    }
    
    try {
        localization_obj_.UpdateFrontAxlePosition(clock);
        LocState updated_state = localization_obj_.State(); // Use a different variable name to avoid shadowing
        
        // Write debug info to file if debug mode is enabled
        if (debug_mode_) {
            debug_obj_.WriteLocalizationStatesToFile(
                clock, updated_state.speed_, updated_state.delta_, updated_state.pos_[0],
                updated_state.pos_[1], updated_state.psi_);
        }
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Exception in UpdatePosition: " << e.what() << std::endl;
        return false;
    }
}

void AHRSLocHandler::UpdateVehicleState(PreciseSeconds clock,
                                   const std::vector<double>& state) {
    // Check if the state vector has the correct size
    if (state.size() < 4) {
        std::cerr << "Warning: Insufficient state data for vehicle state update" << std::endl;
        return;
    }
    
    // Create a LocState object from the provided state vector
    LocState loc_state = {{state[0], state[1]}, state[2], 0, state[3]};
    
    // Reset the vehicle state
    localization_obj_.ResetVehicleState(clock, loc_state);
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