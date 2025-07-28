/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#include <stdio.h>  // to use NULL
#include <utility>  // for use of pair
#include <cassert>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <string>
#include <tuple>
#include "Utils/Controllers.hpp"// NOLINT
#include "Utils/Functions.hpp"// NOLINT
#include "Utils/Classes.hpp"// NOLINT
#include "Utils/units.hpp"// NOLINT
#include "Utils/AHRS.hpp"// NOLINT
#include "wrapper/control_api_wrapper.h"
#include "ControlAPI.hpp" // NOLINT
#include <nlohmann/json.hpp>
using json = nlohmann::json;

using aidriver::control_api::GetAHRSLocHandlerInstance;

std::shared_ptr<SteeringController> create_controller(const json& control_config, const json& vehicle_config){
    if (control_config["steering_controller"] == "Stanley") {
        return std::make_shared<StanleyController>(control_config, vehicle_config);
    } else if (control_config["steering_controller"] == "LQR") {
        return std::make_shared<LQRController>(control_config, vehicle_config);
    } else {
        assert("Invalid steering controller type !!!");
    }
}

ControlAPI::ControlAPI(const json& vehicle_config,
                       const json& control_config)
    : vehicle_config_(vehicle_config),
      control_config_(control_config),
      steering_controller_ptr_(create_controller(control_config_, vehicle_config_)),
      steering_cmd_rate_limiter_obj_(
                RateLimiter(
                    static_cast<double>(
                        control_config_["rate_limit_for_steering_command"]),
                    (PreciseSeconds)0.0)),
      apply_path_smoothing_(control_config_["apply_path_smoothing"]),
      debug_mode_(control_config_["debug_mode"]),
      valid_motion_planning_path_(false),
      debug_obj_(control_config_["debug_mode"], false,
                 control_config_["control_modul_dir"]),
      localization_handler(aidriver::control_api::GetAHRSLocHandlerInstance(
        std::map<std::string, std::string>(), vehicle_config_, control_config_)),
      longitudinal_controller_(control_config_) {}
ControlAPI::ControlAPI(const std::string& vehicle_config_path,
                       const std::string& control_config_path)
    : vehicle_config_(json::parse(std::ifstream(vehicle_config_path))),
      control_config_(json::parse(std::ifstream(control_config_path))),
      steering_cmd_rate_limiter_obj_(
        RateLimiter(
            static_cast<double>(
                control_config_["rate_limit_for_steering_command"]),
                (PreciseSeconds)0.0)),
                apply_path_smoothing_(control_config_["apply_path_smoothing"]),
                debug_mode_(control_config_["debug_mode"]),
                valid_motion_planning_path_(false),
                debug_obj_(control_config_["debug_mode"],
                false,
                control_config_["control_modul_dir"]),
      steering_controller_ptr_(create_controller(control_config_, vehicle_config_)),
      longitudinal_controller_(control_config_),
      localization_handler(aidriver::control_api::GetAHRSLocHandlerInstance(std::map<std::string, std::string>(), vehicle_config_path, control_config_path)) {}
bool ControlAPI::MotionPlanningUpdate(const std::vector<PreciseMeters>& traj_x,
                                      const std::vector<PreciseMeters>& traj_y,
                                      PreciseSeconds system_time,
                                      const std::string& frame,
                                      PreciseSeconds image_timestamp, 
                                      PreciseSeconds timestamp_for_delay_compensation) {
    assert(traj_x.size() == traj_y.size());
    if (!traj_x.size() == traj_y.size()) {
        std::cout << "ControlAPI.cpp::MotionPlanningUpdate: traj_x.size() != traj_y.size()" << std::endl;
        return false;
    }
    std::vector<PreciseMeters> traj_x_reference;
    std::vector<PreciseMeters> traj_y_reference;
    std::vector<PreciseRadians> traj_psi_reference;
    std::vector<PreciseMeters> minus_traj_y = MultiplyVectorByScalar(
        traj_y, -1.0);
    // in the controler y axis is defined positive right in
    // motion planning left. 
    debug_obj_.motion_planning_path_x_.push_back(traj_x);
    debug_obj_.motion_planning_path_y_.push_back(minus_traj_y);
    if (frame == "EGO") {
        // trajectory is given in EGO coordinate system.
        // therefore we convert it to nav (world)
        int num_points = traj_x.size();
        // create an Eigen array  of size nX2
        Eigen::Matrix<double, Eigen::Dynamic, 2> trajectory_points_ego;
        trajectory_points_ego =
            Eigen::Matrix<double, Eigen::Dynamic, 2>::Zero(num_points, 2);
        for (int i = 0; i < num_points; i++) {
            trajectory_points_ego(i, 0) = traj_x[i];
            trajectory_points_ego(i, 1) = minus_traj_y[i];
        }
        // control_config_["car_camera_center_calibration"] and control_config_["car_camera_calibration_angle"] 
        // are aplied using affine transformation

        Eigen::Matrix3d path_calibration_matrix = AffineTransformation2D(0.0, 
                                                                    control_config_["car_camera_center_calibration"],
                                                                    control_config_["car_camera_calibration_angle"]);
        Eigen::Matrix<double, Eigen::Dynamic, 2> trajectory_points_ego_fixed =
            ProjectPoints2D(path_calibration_matrix, trajectory_points_ego);                                                                
        std::vector<double> vehicle_state;
        if (control_config_["apply_delay_compensation"]) {
            /* 
            NOTE: timestamp_for_delay_compensation is the time of the image if not compensated, but 
            if compensated before the MPC, it the time before MPC calculation. this way we compensate for 
            the MPC calculation time.
            */
            vehicle_state = localization_handler->GetDelay().GetDelayedValue(timestamp_for_delay_compensation);
            PreciseSeconds time_of_compensation = 
                localization_handler->GetDelay().GetLatestTime();
            if (print_delay_compensation) {
                std::cout<<"ControlAPI.cpp::MotionPlanningUpdate: data timestamp (before MPC) = " << 
                    std::to_string(image_timestamp)<<std::endl;
                std::cout<<"latest localization time = " << 
                    std::to_string(time_of_compensation)<<std::endl;
                std::cout<<"MotionPlanningUpdate: compensated for" << 
                    time_of_compensation - timestamp_for_delay_compensation<<std::endl;
            }

        } else {
            LocState state = localization_handler->GetLoc().State();
            vehicle_state = {state.pos_[0], state.pos_[1], state.psi_};
        }
        Eigen::Matrix3d T_nav_2_ego = AffineTransformation2D(
            vehicle_state[0], vehicle_state[1], vehicle_state[2]);
        Eigen::Matrix3d T_ego_2_nav = InvAffineTransformation2D(T_nav_2_ego);
        // project points to nav frame
        Eigen::Matrix<double, Eigen::Dynamic, 2> trajectory_points_nav =
            ProjectPoints2D(T_ego_2_nav, trajectory_points_ego_fixed);
        for (int i = 0; i < num_points; i++) {
            traj_x_reference.push_back(trajectory_points_nav(i, 0));
            traj_y_reference.push_back(trajectory_points_nav(i, 1));
        }
    } else if (frame == "NAV") {
        // notice calibration is not applied to this option nor delay compensation
        traj_x_reference = traj_x;
        traj_y_reference = minus_traj_y;
    } else {
        assert("invalid frame input !!!");
        return false;
    }

    if (apply_path_smoothing_ && traj_x_reference.size() > 2) {
        auto [s_interp, x_interp, y_interp] = SplineInterpulationEigen(
            traj_x_reference, traj_y_reference,
            double(control_config_["ds_for_path_smoothing"]));
        steering_controller_ptr_->UpdatePath(x_interp, y_interp);
    } else {
        steering_controller_ptr_->UpdatePath(traj_x_reference, traj_y_reference);
    }
    CheckMotionPlanningPathValidity();
    debug_obj_.motion_planning_path_x_processed_.push_back(
        steering_controller_ptr_->traj_.x_coor);
    debug_obj_.motion_planning_path_y_processed_.push_back(
        steering_controller_ptr_->traj_.y_coor);
    debug_obj_.motion_planning_path_psi_processed_.push_back(
        steering_controller_ptr_->traj_.psi);
    debug_obj_.motion_planning_update_time_.push_back(system_time);
    debug_obj_.motion_planning_num_updates_++;
    if (debug_mode_) {
        PreciseSeconds latest_loc_time = localization_handler->GetDelay().GetLatestTime();
        debug_obj_.WritePathProcessingStatesToFile(
            latest_loc_time, image_timestamp, traj_x, minus_traj_y,
            steering_controller_ptr_->traj_.x_coor,
            steering_controller_ptr_->traj_.y_coor, 
            steering_controller_ptr_->traj_.psi);
        }
    if (control_config_["online_visualization"]) {
        debug_obj_.Write_file_for_visualization(
        std::filesystem::path(control_config_["visualization_data_path"]),
                              "motion_planning_path");
        }
    return true;
}
bool ControlAPI::ReferenceSpeedUpdate(
    PreciseMps traj_velocity,
    PreciseSeconds system_time) {
    // NOTE: updates controller, not path object!
    // NOTE: no need to insert to debug object here, will be inserted by ctrlr
    longitudinal_controller_.SetReferenceVelocity(traj_velocity);
    return true;
}

/* Update throttle pedal measurement, currently not doing anything */
void ControlAPI::UpdateThrottle(Percentage throttle, PreciseSeconds clock) {
    return;
}

/* Update throttle brake measurement, currently not doing anything. */
void ControlAPI::UpdateBrake(Percentage brake, PreciseSeconds clock) {
    return;
}

bool ControlAPI::CalculateSteeringCommand(PreciseSeconds system_time) {
    CheckMotionPlanningPathValidity();
    if (!valid_motion_planning_path_) {
        return false;
    }
    LocState state = localization_handler->GetLoc().State();
    steering_controller_ptr_->CalcSteeringCommand(state.pos_[0], state.pos_[1], 
                                                  state.psi_, state.speed_, 
                                                  state.delta_, system_time);
    debug_obj_.heading_reference_raw_.push_back(
        std::make_pair(steering_controller_ptr_->reference_psi_raw_, system_time));
    
    debug_obj_.heading_reference_filtered_.push_back(
        std::make_pair(steering_controller_ptr_->reference_psi_filtered_, system_time));

    debug_obj_.heading_error_.push_back(
        std::make_pair(steering_controller_ptr_->e_psi_, system_time));

    debug_obj_.steering_measured_.push_back(
        std::make_pair(state.delta_, system_time));

    debug_obj_.lateral_error_.push_back(
        std::make_pair(steering_controller_ptr_->el_, system_time));

    debug_obj_.vehicle_heading_.push_back(
        std::make_pair(state.psi_, system_time));

    debug_obj_.delta_.push_back(
        std::make_pair(GetDelta(), system_time));

    // create a sample of the control target point and store it in debug object
    std::pair<std::vector<PreciseMeters>, PreciseSeconds>
        control_traget_point_sample;  // <<x,y>, time>
    PreciseMeters target_point_x = steering_controller_ptr_->traj_.x_coor[
            steering_controller_ptr_->target_index_];
    PreciseMeters target_point_y = steering_controller_ptr_->traj_.y_coor[
            steering_controller_ptr_->target_index_];
    std::vector<PreciseMeters> control_traget_point{target_point_x,
                                            target_point_y};
    control_traget_point_sample.first = control_traget_point;
    control_traget_point_sample.second = system_time;
    debug_obj_.control_target_points_.push_back(control_traget_point_sample);

    double delta_limited = LimitValue(steering_controller_ptr_->delta_command_,
                                      -static_cast<double>(
                                        vehicle_config_["MAX_STEER"]),
                                      vehicle_config_["MAX_STEER"]);
    PreciseRadians steering_wheel_command_raw =
        delta_limited * double(vehicle_config_["steering_ratio"]);
    debug_obj_.steering_wheel_command_raw_.push_back(
        std::make_pair(steering_wheel_command_raw, system_time));
    if (control_config_["apply_rate_limiter_to_steering_command"]) {
        steering_wheel_command_ = steering_cmd_rate_limiter_obj_.Update(
                                system_time, steering_wheel_command_raw);
    } else {
        steering_wheel_command_ = steering_wheel_command_raw;
    }
    debug_obj_.steering_wheel_command_filtered_.push_back(
        std::make_pair(steering_wheel_command_, system_time));
    debug_obj_.steering_num_updates_++;
    if (debug_mode_) {
        PreciseMeters target_point_x = steering_controller_ptr_->traj_.x_coor[
            steering_controller_ptr_->target_index_];
        PreciseMeters target_point_y = steering_controller_ptr_->traj_.y_coor[
            steering_controller_ptr_->target_index_];
        debug_obj_.WriteControlStatesToFile(
            system_time, target_point_x, target_point_y,
            steering_controller_ptr_->reference_psi_raw_,
            steering_controller_ptr_->reference_psi_filtered_,
            steering_controller_ptr_->vehicle_psi_, 
            steering_controller_ptr_->e_psi_,
            steering_controller_ptr_->el_, 
            steering_controller_ptr_->delta_command_, 
            delta_limited,//wheel angle
            steering_wheel_command_ / double(vehicle_config_["steering_ratio"])//wheel angle
            );
    }
    if (control_config_["online_visualization"]) {
        debug_obj_.Write_file_for_visualization(
        std::filesystem::path(control_config_["visualization_data_path"]),
                              "steering");
        }
    return true;
}

void ControlAPI::GetSteeringControllerStates(PreciseRadians* delta,
                                             PreciseMeters* ef) const {
    *delta = steering_controller_ptr_->delta_command_;
    *ef = steering_controller_ptr_->el_;
}
std::tuple<std::vector<double>, std::vector<double>>
ControlAPI::GetReferenceTrajectory() const {
    std::tuple<std::vector<double>, std::vector<double>> result;
    result = std::make_tuple(steering_controller_ptr_->traj_.x_coor,
                             steering_controller_ptr_->traj_.y_coor);
    return result;
}
PreciseRadians ControlAPI::GetDelta() const {
    double delta_limited = LimitValue(steering_controller_ptr_->delta_command_,
                                      -static_cast<double>(
                                        vehicle_config_["MAX_STEER"]),
                                      vehicle_config_["MAX_STEER"]);
    return delta_limited;
}
PreciseMeters ControlAPI::GetLateralError() const {
    return steering_controller_ptr_->el_;
}
PreciseRadians ControlAPI::GetHeadingError() const {
    return steering_controller_ptr_->e_psi_;
}
PreciseRadians ControlAPI::GetHeadingReference() const {
    return steering_controller_ptr_->reference_psi_raw_;
}
PreciseRadians ControlAPI::GetSteeringCmd() const {
    return steering_wheel_command_;
}
void ControlAPI::UpdateControlDebugStates() {}
void ControlAPI::WriteDebugStates(std::string path) const {
    debug_obj_.WriteToFile(path);
}
void ControlAPI::CheckMotionPlanningPathValidity() {
    steering_controller_ptr_->Lock();
    /*lock th controller while accessing internal state so it is not 
    written while it is read.*/
    if (steering_controller_ptr_->traj_.x_coor.size() == 0 ||
        steering_controller_ptr_->traj_.y_coor.size() == 0) {
        valid_motion_planning_path_ = false;
        std::cout << "ControlAPI::CheckMotionPlanningPathValidity:" <<
            "traj length = 0" << std::endl;
        steering_controller_ptr_->Unlock();
        return;
    }
    LocState state = localization_handler->GetLoc().State();

    // assuming conterller has updated trajectory and localization object has
    // the updated vehicle position
    PreciseMeters vehicle_pos_x = state.pos_[0];
    PreciseMeters vehicle_pos_y = state.pos_[1];
    int idx =
        steering_controller_ptr_->traj_.ProjectPoint(vehicle_pos_x, vehicle_pos_y,
                                state.psi_, true);
    if (idx == -1){
        std::cout << "ControlAPI::CheckMotionPlanningPathValidity:" <<
            "Motion planning path is not valid, all points behind vehicle" << std::endl;
        std::cout << "vehicle_pos_x: " << vehicle_pos_x << std::endl;
        std::cout << "vehicle_pos_y: " << vehicle_pos_y << std::endl;
        std::cout << "state.psi_: " << state.psi_ << std::endl;
        std::cout << "traj_.x: ";
        for (const auto& x : steering_controller_ptr_->traj_.x_coor) {
            std::cout << x << " ";
        }
        std::cout << std::endl;

        std::cout << "traj_.y: ";
        for (const auto& y : steering_controller_ptr_->traj_.y_coor) {
            std::cout << y << " ";
        }
        std::cout << std::endl;
        valid_motion_planning_path_ = false;
        steering_controller_ptr_->Unlock();
        return;
    }
    
    PreciseMeters dx = vehicle_pos_x - steering_controller_ptr_->traj_.x_coor[idx];
    PreciseMeters dy = vehicle_pos_y - steering_controller_ptr_->traj_.y_coor[idx];
    std::vector<PreciseMeters> dxdy{dx, dy};
    PreciseMeters closest_dist = VectorNorm2D(dxdy);
    PreciseMeters dist_from_end_point_along_path =
        steering_controller_ptr_->traj_.s.back() -
        steering_controller_ptr_->traj_.s[idx];
    valid_motion_planning_path_ = false;
    if (dist_from_end_point_along_path >
            control_config_["minimal_distance_for_valid_path"] &&
        closest_dist < control_config_["maximal_distance_for_valid_path"]) {
        valid_motion_planning_path_ = true;
    }
    else {
        if (!(dist_from_end_point_along_path >
            control_config_["minimal_distance_for_valid_path"])){
                std::cout << "ControlAPI::CheckMotionPlanningPathValidity:" <<
            "minimal distance condition" << std::endl;
            std::cout << "vehicle_pos_x: " << vehicle_pos_x << std::endl;
            std::cout << "vehicle_pos_y: " << vehicle_pos_y << std::endl;
            std::cout << "state.psi_: " << state.psi_ << std::endl;
            std::cout << "traj_.x: ";
            for (const auto& x : steering_controller_ptr_->traj_.x_coor) {
                std::cout << x << " ";
            }
            std::cout << std::endl;

            std::cout << "traj_.y: ";
            for (const auto& y : steering_controller_ptr_->traj_.y_coor) {
                std::cout << y << " ";
            }
            std::cout << std::endl;
            }
        else if (!(closest_dist < control_config_["maximal_distance_for_valid_path"]))
        {
            std::cout << "ControlAPI::CheckMotionPlanningPathValidity:" <<
            "maximal distance condition" << std::endl;
            std::cout << "vehicle_pos_x: " << vehicle_pos_x << std::endl;
            std::cout << "vehicle_pos_y: " << vehicle_pos_y << std::endl;
            std::cout << "state.psi_: " << state.psi_ << std::endl;
            std::cout << "traj_.x: ";
            for (const auto& x : steering_controller_ptr_->traj_.x_coor) {
                std::cout << x << " ";
            }
            std::cout << std::endl;

            std::cout << "traj_.y: ";
            for (const auto& y : steering_controller_ptr_->traj_.y_coor) {
                std::cout << y << " ";
            }
            std::cout << std::endl;
        }
    }
    steering_controller_ptr_->Unlock();
    return;
}
void ControlAPI::TerminateControlStates() {
    // delete steering_controller_;
}

void ControlAPI::TestStart(int test_id) {
    // ignore test_id for now
    ongoing_test_id_ = kInvalidTestId;
}

void ControlAPI::TestFinish() {
    // set to NO test state
    ongoing_test_id_ = kInvalidTestId;
}

bool ControlAPI::IsInTestState() const {
    return ongoing_test_id_ != kInvalidTestId;
}

json ControlAPI::GetControlConfiguration() const {
    return control_config_;
}
bool ControlAPI::CompensateForDelayBeforeMPC() const{
    return control_config_["compensate_for_delay_before_MPC"];
}
PreciseSeconds ControlAPI::GetMotionPlanningUpdateCallFrequency() const {
    return (PreciseSeconds)control_config_["path_update_call_freq"];
}
std::tuple<std::vector<std::vector<PreciseMeters>>, 
        PreciseSeconds> ControlAPI::CompensateForDelay(
            vector<vector<PreciseMeters>> traj_ego, 
            PreciseSeconds path_time_stamp, 
            std::string control_point) const{
    /* 
    to compensate localization object is used this implies that NED convention is used
    motion planning and MPC use ENU convention verify conversion before calling the function.
    the function returns the path in NED convention at EGO frame. 
       */
    std::vector<PreciseMeters> traj_x = traj_ego[0];
    std::vector<PreciseMeters> traj_y= traj_ego[1];
    int num_points = traj_x.size();
    // create an Eigen array  of size nX2
    Eigen::Matrix<double, Eigen::Dynamic, 2> trajectory_points_ego;
    trajectory_points_ego =
            Eigen::Matrix<double, Eigen::Dynamic, 2>::Zero(num_points, 2);
    for (int i = 0; i < num_points; i++) {
        trajectory_points_ego(i, 0) = traj_x[i];
        trajectory_points_ego(i, 1) = traj_y[i];
    }
    std::vector<double> vehicle_state;
    vehicle_state = localization_handler->GetDelay().GetDelayedValue(path_time_stamp);// currently is achieved in front axle
    if (control_point == "Rear"){
        vehicle_state[0] -= double(vehicle_config_["WB"]) * std::cos(vehicle_state[2]);
        vehicle_state[1] -= double(vehicle_config_["WB"]) * std::sin(vehicle_state[2]);
    }
    Eigen::Matrix3d T_nav_2_ego = AffineTransformation2D(
            vehicle_state[0], vehicle_state[1], vehicle_state[2]);
    Eigen::Matrix3d T_ego_2_nav = InvAffineTransformation2D(T_nav_2_ego);
        // project points to nav frame
    Eigen::Matrix<double, Eigen::Dynamic, 2> trajectory_points_nav =
            ProjectPoints2D(T_ego_2_nav, trajectory_points_ego);
    // now we have the path in navigation frame convert it back to ego frame using vehicle state at current time
    std::vector<PreciseMeters> current_pos = localization_handler->GetPosition();//todo convert to rear axle
    PreciseRadians current_heading = localization_handler->GetVehicleHeading();
    if (control_point == "Rear"){
        current_pos[0] -= double(vehicle_config_["WB"]) * std::cos(current_heading);
        current_pos[1] -= double(vehicle_config_["WB"]) * std::sin(current_heading);
    }
    Eigen::Matrix3d T_nav_2_ego_current = AffineTransformation2D(
            current_pos[0], current_pos[1], current_heading);   
    Eigen::Matrix<double, Eigen::Dynamic, 2> trajectory_points_ego_compensated =
            ProjectPoints2D(T_nav_2_ego_current, trajectory_points_nav);
    std::vector<PreciseMeters> trajectory_ego_compensated_x;
    std::vector<PreciseMeters> trajectory_ego_compensated_y;
    for (int i = 0; i < num_points; i++) {
        trajectory_ego_compensated_x.push_back(trajectory_points_ego_compensated(i, 0));
        trajectory_ego_compensated_y.push_back(trajectory_points_ego_compensated(i, 1));
    }
    std::vector<std::vector<PreciseMeters>> trajectory_ego_compensated;
    trajectory_ego_compensated.push_back(trajectory_ego_compensated_x);
    trajectory_ego_compensated.push_back(trajectory_ego_compensated_y);
    PreciseSeconds time_of_compensation = localization_handler->GetDelay().GetLatestTime();
    if (print_delay_compensation){
        std::cout << "=====ControlAPI::CompensateForDelay====="<<std::endl;
        PreciseMeters delta_pos = std::sqrt(
            std::pow(current_pos[0] - vehicle_state[0], 2) + 
            std::pow(current_pos[1] - vehicle_state[1], 2));    
        std::cout << ""<< "delta_pos = " << delta_pos << std::endl;
        std::cout << "ControlAPI::CompensateForDelay: path_time_stamp = "<< 
        std::to_string(path_time_stamp)<<", compensated for = " << time_of_compensation - path_time_stamp << std::endl;
    }
    return std::make_tuple(trajectory_ego_compensated, time_of_compensation);
}   
bool ControlAPI::CalculateLongitudinalCommand(PreciseSeconds system_time) {
    LocState state = localization_handler->GetLoc().State();
    longitudinal_controller_.UpdateClock(system_time);
    longitudinal_controller_.SetVehicleVelocity(
        state.speed_);
    longitudinal_controller_.CalcVelocityError();
    longitudinal_controller_.CalcVelocityErrorIntegral();
    longitudinal_controller_.CalcVelocityErrorDoubleIntegral();
    longitudinal_controller_.CalcVelocityErrorDerivative();
    longitudinal_controller_.UpdatePIDController(system_time);
    longitudinal_controller_.UpdateThrottleBrakeSwitch(driving_mode_);
    longitudinal_controller_.CalcLongitudinalCommand(system_time);
    longitudinal_command_ = longitudinal_controller_.GetLongitudinalCommand();
    std::apply([&](auto&&... args) {
        debug_obj_.UpdateLongitudinalStates(args...);
        }, longitudinal_controller_.GetControllerStates(system_time));
    if (debug_mode_) {
        std::apply([&](auto&&... args) {
        debug_obj_.WriteLongitudinalControlStatesToFile(args...);
        }, longitudinal_controller_.GetControllerStates(system_time));
    }
    if (control_config_["online_visualization"]) {
        debug_obj_.Write_file_for_visualization(
        std::filesystem::path(control_config_["visualization_data_path"]),
                              "longitudinal");
    }
    return true;
}

std::pair<Percentage, Percentage> ControlAPI::GetLongitudinalCommand() {
    return longitudinal_command_;
}

std::tuple<
    PreciseSeconds,    // Log timestamp
    bool,   // throttle_mode
    bool, // brake_mode,
    double, // kp_throttle,
    double, // ki_throttle,
    double, // kii_throttle,
    double, // kd_throttle,
    double, // kp_brake,
    double, // ki_brake,
    double, // kii_brake,
    double, // kd_brake,
    PreciseMps, // vehicle_velocity
    PreciseMps, // vehicle_velocity_filtered
    PreciseMps, // reference_velocity,
    PreciseMps, // reference_velocity_filtered,
    PreciseMps, // velocity_error,
    PreciseMps, // previous_velocity_error
    PreciseMeters, // velocity_error_integral_throttle,
    PreciseMeters,   // velocity_error_double_integral_throttle
    PreciseMeters, // velocity_error_integral_brake,
    PreciseMeters,   // velocity_error_double_integral_brake,
    PreciseMeters, // velocity_error_integral_global,
    Mps2Precise, // velocity_error_derivative,
    double, // p_term_throttle,
    double, // i_term_throttle,
    double, // ii_term_throttle,
    double, // d_term_throttle,
    double, // p_term_brake,
    double, // i_term_brake,
    double, // ii_term_brake,
    double, // d_term_brake,
    PreciseSeconds, // last_update_throttle,
    PreciseSeconds, // last_update_brake,
    PreciseSeconds, // last_command_update,
    PreciseSeconds, // dt,
    PreciseSeconds, // clock,
    double, // raw_control_throttle,
    double, // raw_control_brake,
    Percentage, // control_throttle,
    Percentage, // control_brake,
    Percentage, // halt_mode_additional_brake,
    Percentage, // command_throttle,
    Percentage // command_brake
>
ControlAPI::GetLongitudinalControllerStates(PreciseSeconds system_time) {
    return longitudinal_controller_.GetControllerStates(system_time);
}

void ControlAPI::SetDrivingMode(float driving_mode) {
    driving_mode_ = driving_mode;
}   
std::tuple<std::vector<PreciseMeters>, std::vector<PreciseMeters>,
std::vector<PreciseMeters>, std::vector<PreciseRadians>, bool>
ControlAPI::InterpulatePath(const std::vector<PreciseMeters> &x,
                            const std::vector<PreciseMeters> &y,
                            PreciseMeters ds)  const{
        auto [s_interp, x_interp, y_interp] = SplineInterpulationEigen(x, y, ds);
        std::vector<PreciseRadians> psi_interp;
        CalcCurveHeading(x_interp, y_interp, &psi_interp);
        // in CalcCurveHeading n-1 values of psi_interp are filled so the n'th 
        // value is filled with the same value as the n-1'th value   
        psi_interp.back() = psi_interp[psi_interp.size() - 2];
        bool success = true; /*TODO: implement some way to catch failurs*/
        return std::make_tuple(x_interp, y_interp, s_interp, psi_interp, success);
    }
std::tuple<VectorXd, VectorXd, VectorXd, bool>
ControlAPI::InterpulatePath(const std::vector<PreciseMeters> &x,
                            const std::vector<PreciseMeters> &y,
                            const std::vector<PreciseMeters> &s_interp, 
                            std::string method)  const{  
        std::vector<double> x_interp, y_interp;
        bool success;
        VectorXd x_interp_eigen;
        VectorXd y_interp_eigen;
        VectorXd psi_interp_eigen;
        if (method == "linear"){
            std::tie(x_interp, y_interp, success) = LinearPathInterpulation(x, y, s_interp);
        } else if (method == "splines"){
            std::tie(x_interp, y_interp) = SplineInterpulationEigen(x, y, s_interp);
            success = true; /*TODO: not implemented in SplineInterpulationEigen*/
        } else {
            throw std::invalid_argument("invalid method input !!!");
        }
        if (!success){
            std::cout << "ControlAPI::InterpulatePath: linear interpolation failed" << std::endl;
            return std::make_tuple(x_interp_eigen, y_interp_eigen, psi_interp_eigen, success);
        }
        std::vector<PreciseRadians> psi_interp;
        CalcCurveHeading(x_interp, y_interp, &psi_interp);
        // in CalcCurveHeading n-1 values of psi_interp are filled so the n'th 
        // value is filled with the same value as the n-1'th value   
        PreciseRadians last_psi = psi_interp.back();
        psi_interp.push_back(last_psi);
        x_interp_eigen = ConvertVectorToEigen(x_interp);
        y_interp_eigen = ConvertVectorToEigen(y_interp);
        psi_interp_eigen = ConvertVectorToEigen(psi_interp);
        return std::make_tuple(x_interp_eigen, y_interp_eigen, psi_interp_eigen, true);
 }
std::tuple<std::vector<double>, bool>
ControlAPI::LinearInterpWrapper(const std::vector<double>& x, 
                                const std::vector<double>& y, 
                                const std::vector<double>& x_query) const {
    return LinearInterp(x, y, x_query);
}
std::tuple<std::vector<PreciseMeters>, 
           std::vector<PreciseMeters>, 
           std::vector<PreciseRadians>> 
ControlAPI::ConvertPathControlPoint(
                    const std::vector<PreciseMeters>& path_points_cp1_x,
                    const std::vector<PreciseMeters>& path_points_cp1_y,
                    PreciseMeters lr1, PreciseMeters lr2, PreciseMeters WB,
                    bool convert_to_cp2_frame) const {
                    return convert_path_control_points(path_points_cp1_x, path_points_cp1_y, lr1, lr2, WB, convert_to_cp2_frame);
}
