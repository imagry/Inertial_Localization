/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/

#include "control_debug_states.hpp"

#include <iostream>

#include "DataHandling.hpp"

ControlDebugStates::ControlDebugStates(
    bool create_debug_dir, bool localization_mode,
    std::filesystem::path control_module_dir) {
    /* B10: Removed as part of control code removal
    motion_planning_num_updates_ = 0;
    steering_num_updates_ = 0;
    */
    if (create_debug_dir) {
        // if control_module_dir doesn't exist dont try saving files
        control_module_dir_exist_ = std::filesystem::exists(
            control_module_dir);
        if (control_module_dir_exist_) {
            std::filesystem::path temp_results_dir_path =
                control_module_dir / "data/temp_results";
            // data dir is backed in repo,
            // if temp_results dir doesn't exist create it
            if (!std::filesystem::exists(temp_results_dir_path)) {
                std::cout << "Directory does not exist." << std::endl;
                (std::filesystem::create_directory(temp_results_dir_path));
                std::cout << temp_results_dir_path << " Created" << std::endl;
            }
            std::filesystem::path current_run_debug_dir =
                        temp_results_dir_path / WhatsTheTimeString();
            std::filesystem::create_directory(current_run_debug_dir);
            std::cout << current_run_debug_dir << " Created" << std::endl;
            CreateDebugFiles(current_run_debug_dir, localization_mode);
        } else {
            std::cout << "control module dir from json file doesn't exist, not "
                         "saving debug data"
                      << "\n";
            PrintCurrentPath();
        }
    } 
}
void ControlDebugStates::CreateDebugFiles(std::filesystem::path path,
                                          bool localization_mode) {
    localization_debug_file_.open(path / "debug_localization.csv");
    localization_debug_file_ << "time [sec],"
                            << "vel [mps],"
                            << "steering [rad],"
                            << "x [m]"
                            << ","
                            << "y [m]"
                            << ","
                            << "psi [rad]"
                            << "\n";  // <<t,v,delta,x,y,psi>
    AHRS_debug_file_.open(path / "debug_AHRS.csv");
    AHRS_debug_file_
        << "time_IMU,"
        << "time_OS,"
        << "phi_hat,"
        << "theta_hat,"
        << "psi_hat,"
        << "phiIMU,"
        << "thetaIMU,"
        << "psiIMU,"
        << "\n";  // <<time,phi_hat,theta_hat,psi_hat>
    std::cout << "created " << (path / "debug_AHRS.csv").string()
                << "\n";
    /* B10: Removed as part of control code removal
    if (localization_mode) {
        localization_debug_file_.open(path / "debug_localization.csv");
        localization_debug_file_ << "time [sec],"
                                << "vel [mps],"
                                << "steering [rad],"
                                << "x [m]"
                                << ","
                                << "y [m]"
                                << ","
                                << "psi [rad]"
                                << "\n";  // <<t,v,delta,x,y,psi>
        AHRS_debug_file_.open(path / "debug_AHRS.csv");
        AHRS_debug_file_
            << "time_IMU,"
            << "time_OS,"
            << "phi_hat,"
            << "theta_hat,"
            << "psi_hat,"
            << "phiIMU,"
            << "thetaIMU,"
            << "psiIMU,"
            << "\n";  // <<time,phi_hat,theta_hat,psi_hat>
        std::cout << "created " << (path / "debug_AHRS.csv").string()
                << "\n";
    } else {
        path_processing_debug_file_.open(path / "debug_path_processing.csv");
        steering_control_debug_file_.open(path / "debug_steering_control.csv");
        steering_control_debug_file_
            << "time [sec],"
            << "target_point_x [m]" << ","
            << "target_point_y [m]" << ","
            << "reference_heading_raw [rad],"
            << "reference_heading_filtered [rad],"
            << "vehicle_heading [rad],"
            << "heading_error [rad],"
            << "lateral_error [m],"
            << "delta_cmd [rad],"
            << "steering_wheel_cmd_raw [rad],"
            << "steering_wheel_cmd_filtered [rad],"
            << "\n";  // <<time,heading_error,lateral_error,delta>
        std::cout << "created " << (path / "debug_steering_control.csv").string()
                << "\n";
    }
    longitudinal_control_debug_file_.open(path / "debug_longitudinal_control.csv");
    longitudinal_control_debug_file_
        << "time [sec],"
        << "throttle_mode [bool]" << ","
        << "brake_mode [bool]" << ","
        << "kp_throttle [-]" << ","
        << "ki_throttle [-]" << ","
        << "kii_throttle [-]" << ","
        << "kd_throttle [-]" << ","
        << "kp_brake [-]" << ","
        << "ki_brake [-]" << ","
        << "kii_brake [-]" << ","
        << "kd_brake [-]" << ","
        << "vehicle_velocity [mps]" << ","
        << "vehicle_velocity_filtered [mps]" << ","
        << "reference_velocity [mps]" << ","
        << "reference_velocity_filtered [mps]" << ","
        << "velocity_error [mps]" << ","
        << "previous_velocity_error [mps]" << ","
        << "velocity_error_integral_throttle [m]" << ","
        << "velocity_error_double_integral_throttle [m*s]" << ","
        << "velocity_error_integral_brake [m]" << ","
        << "velocity_error_double_integral_brake [m*s]" << ","
        << "velocity_error_integral_global [m]" << ","
        << "velocity_error_derivative [mps^2]" << ","
        << "p_term_throttle [-]" << ","
        << "i_term_throttle [-]" << ","
        << "ii_term_throttle [-]" << ","
        << "d_term_throttle [-]" << ","
        << "p_term_brake [-]" << ","
        << "i_term_brake [-]" << ","
        << "ii_term_brake [-]" << ","
        << "d_term_brake [-]" << ","
        << "last_update_throttle [sec]" << ","
        << "last_update_brake [sec]" << ","
        << "last_command_update [sec]" << ","
        << "dt [sec]" << ","
        << "clock [sec]" << ","
        << "raw_control_throttle [-]" << ","
        << "raw_control_brake [-]" << ","
        << "control_throttle [%]" << ","
        << "control_brake [%]" << ","
        << "halt_mode_additional_brake [%]" << ","
        << "command_throttle [%]" << ","
        << "command_brake [%]" << ","
        << "\n";
    std::cout << "created " << (
        path / "debug_longitudinal_control.csv").string()
            << "\n";
    */
}
/* B10: Removed as part of control code removal
void ControlDebugStates::UpdateLongitudinalStates(
    PreciseSeconds time,
    bool throttle_mode,
    bool brake_mode,
    double kp_throttle,
    double ki_throttle,
    double kii_throttle,
    double kd_throttle,
    double kp_brake,
    double ki_brake,
    double kii_brake,
    double kd_brake,
    PreciseMps vehicle_velocity,
    PreciseMps vehicle_velocity_filtered,
    PreciseMps reference_velocity,
    PreciseMps reference_velocity_filtered,
    PreciseMps velocity_error,
    PreciseMps previous_velocity_error,
    PreciseMeters velocity_error_integral_throttle,
    PreciseMeters
    velocity_error_double_integral_throttle,
    PreciseMeters velocity_error_integral_brake,
    PreciseMeters
    velocity_error_double_integral_brake,
    PreciseMeters velocity_error_integral_global,
    Mps2Precise velocity_error_derivative,
    double p_term_throttle,
    double i_term_throttle,
    double ii_term_throttle,
    double d_term_throttle,
    double p_term_brake,
    double i_term_brake,
    double ii_term_brake,
    double d_term_brake,
    PreciseSeconds last_update_throttle,
    PreciseSeconds last_update_brake,
    PreciseSeconds last_command_update,
    PreciseSeconds dt,
    PreciseSeconds clock,
    double raw_control_throttle,
    double raw_control_brake,
    Percentage control_throttle,
    Percentage control_brake,
    Percentage halt_mode_additional_brake,
    Percentage command_throttle,
    Percentage command_brake
) {
    throttle_mode_.push_back(
            std::make_pair(throttle_mode, clock));

    brake_mode_.push_back(
            std::make_pair(brake_mode, clock));

    kp_throttle_.push_back(
            std::make_pair(kp_throttle, clock));

    ki_throttle_.push_back(
            std::make_pair(ki_throttle, clock));

    kii_throttle_.push_back(
            std::make_pair(kii_throttle, clock));

    kd_throttle_.push_back(
            std::make_pair(kd_throttle, clock));

    kp_brake_.push_back(
            std::make_pair(kp_brake, clock));

    ki_brake_.push_back(
            std::make_pair(ki_brake, clock));

    kii_brake_.push_back(
            std::make_pair(kii_brake, clock));

    kd_brake_.push_back(
            std::make_pair(kd_brake, clock));

    vehicle_velocity_.push_back(
            std::make_pair(vehicle_velocity, clock));

    vehicle_velocity_filtered_.push_back(
            std::make_pair(vehicle_velocity_filtered, clock));

    reference_velocity_.push_back(
            std::make_pair(reference_velocity, clock));

    reference_velocity_filtered_.push_back(
            std::make_pair(reference_velocity_filtered, clock));

    velocity_error_.push_back(
            std::make_pair(velocity_error, clock));

    previous_velocity_error_.push_back(
            std::make_pair(previous_velocity_error, clock));

    velocity_error_integral_throttle_.push_back(
            std::make_pair(velocity_error_integral_throttle, clock));

    velocity_error_double_integral_throttle_.push_back(
            std::make_pair(velocity_error_double_integral_throttle, clock));

    velocity_error_integral_brake_.push_back(
            std::make_pair(velocity_error_integral_brake, clock));

    velocity_error_double_integral_brake_.push_back(
            std::make_pair(velocity_error_double_integral_brake, clock));

    velocity_error_integral_global_.push_back(
            std::make_pair(velocity_error_integral_global, clock));

    velocity_error_derivative_.push_back(
            std::make_pair(velocity_error_derivative, clock));

    p_term_throttle_.push_back(
            std::make_pair(p_term_throttle, clock));

    i_term_throttle_.push_back(
            std::make_pair(i_term_throttle, clock));

    ii_term_throttle_.push_back(
            std::make_pair(ii_term_throttle, clock));

    d_term_throttle_.push_back(
            std::make_pair(d_term_throttle, clock));

    p_term_brake_.push_back(
            std::make_pair(p_term_brake, clock));

    i_term_brake_.push_back(
            std::make_pair(i_term_brake, clock));

    ii_term_brake_.push_back(
            std::make_pair(ii_term_brake, clock));

    d_term_brake_.push_back(
            std::make_pair(d_term_brake, clock));

    last_update_throttle_.push_back(
            std::make_pair(last_update_throttle, clock));

    last_update_brake_.push_back(
            std::make_pair(last_update_brake, clock));

    last_command_update_.push_back(
            std::make_pair(last_command_update, clock));

    dt_.push_back(
            std::make_pair(dt, clock));

    clock_.push_back(
            std::make_pair(clock, clock));

    raw_control_throttle_.push_back(
            std::make_pair(raw_control_throttle, clock));

    raw_control_brake_.push_back(
            std::make_pair(raw_control_brake, clock));

    control_throttle_.push_back(
            std::make_pair(control_throttle, clock));

    control_brake_.push_back(
            std::make_pair(control_brake, clock));

    halt_mode_additional_brake_.push_back(
            std::make_pair(halt_mode_additional_brake, clock));

    command_throttle_.push_back(
            std::make_pair(command_throttle, clock));

    command_brake_.push_back(
            std::make_pair(command_brake, clock));
}

void ControlDebugStates::WriteLongitudinalControlStatesToFile(
    PreciseSeconds time,
    bool throttle_mode,
    bool brake_mode,
    double kp_throttle,
    double ki_throttle,
    double kii_throttle,
    double kd_throttle,
    double kp_brake,
    double ki_brake,
    double kii_brake,
    double kd_brake,
    PreciseMps vehicle_velocity,
    PreciseMps vehicle_velocity_filtered,
    PreciseMps reference_velocity,
    PreciseMps reference_velocity_filtered,
    PreciseMps velocity_error,
    PreciseMps previous_velocity_error,
    PreciseMeters velocity_error_integral_throttle,
    PreciseMeters
    velocity_error_double_integral_throttle,
    PreciseMeters velocity_error_integral_brake,
    PreciseMeters
    velocity_error_double_integral_brake,
    PreciseMeters velocity_error_integral_global,
    Mps2Precise velocity_error_derivative,
    double p_term_throttle,
    double i_term_throttle,
    double ii_term_throttle,
    double d_term_throttle,
    double p_term_brake,
    double i_term_brake,
    double ii_term_brake,
    double d_term_brake,
    PreciseSeconds last_update_throttle,
    PreciseSeconds last_update_brake,
    PreciseSeconds last_command_update,
    PreciseSeconds dt,
    PreciseSeconds clock,
    double raw_control_throttle,
    double raw_control_brake,
    Percentage control_throttle,
    Percentage control_brake,
    Percentage halt_mode_additional_brake,
    Percentage command_throttle,
    Percentage command_brake
) {
    if (control_module_dir_exist_) {
        longitudinal_control_debug_file_
        << std::to_string(time) << ","
        << throttle_mode << ","
        << brake_mode << ","
        << kp_throttle << ","
        << ki_throttle << ","
        << kii_throttle << ","
        << kd_throttle << ","
        << kp_brake << ","
        << ki_brake << ","
        << kii_brake << ","
        << kd_brake << ","
        << vehicle_velocity << ","
        << vehicle_velocity_filtered << ","
        << reference_velocity << ","
        << reference_velocity_filtered << ","
        << velocity_error << ","
        << previous_velocity_error << ","
        << velocity_error_integral_throttle << ","
        << velocity_error_double_integral_throttle << ","
        << velocity_error_integral_brake << ","
        << velocity_error_double_integral_brake << ","
        << velocity_error_integral_global << ","
        << velocity_error_derivative << ","
        << p_term_throttle << ","
        << i_term_throttle << ","
        << ii_term_throttle << ","
        << d_term_throttle << ","
        << p_term_brake << ","
        << i_term_brake << ","
        << ii_term_brake << ","
        << d_term_brake << ","
        << std::to_string(last_update_throttle) << ","
        << std::to_string(last_update_brake) << ","
        << std::to_string(last_command_update) << ","
        << std::to_string(dt) << ","
        << std::to_string(clock) << ","
        << raw_control_throttle << ","
        << raw_control_brake << ","
        << control_throttle << ","
        << control_brake << ","
        << halt_mode_additional_brake << ","
        << command_throttle << ","
        << command_brake << ","
        << "\n";} else {
        std::cout<< "control_module_dir_exist_ = false!!!"<< "\n";
    }
}
void ControlDebugStates::WriteControlStatesToFile(
    PreciseSeconds clock,
    PreciseMeters target_point_x,
    PreciseMeters target_point_y,
    PreciseRadians reference_heading_raw,
    PreciseRadians reference_heading_filtered,
    PreciseRadians vehicle_heading,
    PreciseRadians heading_error,
    PreciseMeters lateral_error,
    PreciseRadians delta_i, 
    PreciseRadians steering_cmd_raw,// in wheel angle
    PreciseRadians steering_cmd_filtered) {
    if (control_module_dir_exist_) {
        steering_control_debug_file_
        << std::to_string(clock) << ","
        << target_point_x << ","
        << target_point_y << ","
        << reference_heading_raw << ","
        << reference_heading_filtered << ","
        << vehicle_heading << ","
        << heading_error << ","
        << lateral_error << ","
        << delta_i << ","
        << steering_cmd_raw << ","
        << steering_cmd_filtered << ","
        // << steering_wheel_command_raw_.back().first << ","
        // << steering_wheel_command_filtered_.back().first << ","
        << "\n";
} else {
        std::cout<< "control_module_dir_exist_ = false!!!"<< "\n";
    }
}
*/
void ControlDebugStates::WriteLocalizationStatesToFile(
    PreciseSeconds clock, PreciseMps car_speed, PreciseRadians steering_angle,
    PreciseMeters vehicle_x, PreciseMeters vehicle_y,
    PreciseRadians vehicle_psi) {
    if (control_module_dir_exist_) {
        std::lock_guard<std::mutex> guard(lock_);
        localization_debug_file_ << std::to_string(clock) << "," << car_speed
                                 << "," << steering_angle << ","
                                 << std::to_string(vehicle_x) << ","
                                 << std::to_string(vehicle_y) << ","
                                 << vehicle_psi
                                 << ","
                                 << "\n";
    }
}
void ControlDebugStates::WriteAHRSStatesToFile(PreciseSeconds OS_clock,
                                               PreciseSeconds IMU_clock,
                                               PreciseRadians phi_hat,
                                               PreciseRadians theta_hat,
                                               PreciseRadians psi_hat, 
                                               PreciseRadians phiIMU,
                                               PreciseRadians thetaIMU,
                                               PreciseRadians psiIMU) {
    if (control_module_dir_exist_) {
        std::lock_guard<std::mutex> guard(lock_);
        AHRS_debug_file_ << std::to_string(IMU_clock) << ","
                         << std::to_string(OS_clock) << ","
                         << std::to_string(phi_hat) << ","
                         << std::to_string(theta_hat) << ","
                         << std::to_string(psi_hat) << ","
                         << std::to_string(phiIMU) << ","
                         << std::to_string(thetaIMU) << ","
                         << std::to_string(psiIMU) << ","
                         << "\n";
    }
}
/* B10: Removed as part of control code removal
void ControlDebugStates::WritePathProcessingStatesToFile(
    PreciseSeconds clock, PreciseSeconds image_timestamp,
    const std::vector<PreciseMeters>& input_path_x,
    const std::vector<PreciseMeters>& input_path_y,
    const std::vector<PreciseMeters>& processed_path_x,
    const std::vector<PreciseMeters>& processed_path_y,
    const std::vector<PreciseRadians>& processed_path_psi) {
    if (control_module_dir_exist_) {
        path_processing_debug_file_ << "time:\n"
                                    << std::to_string(clock) << "\n";
        path_processing_debug_file_ << "image_timestamp:\n"
                                    << std::to_string(image_timestamp) << "\n";
        path_processing_debug_file_ << "input_path_x:\n";
        for (PreciseMeters x_point : input_path_x) {
            path_processing_debug_file_ << x_point << ",";
        }
        path_processing_debug_file_ << "\n";
        path_processing_debug_file_ << "input_path_y:\n";
        for (PreciseMeters y_point : input_path_y) {
            path_processing_debug_file_ << y_point << ",";
        }
        path_processing_debug_file_ << "\n";
        path_processing_debug_file_ << "processed_path_x:\n";
        for (PreciseMeters x_point : processed_path_x) {
            path_processing_debug_file_ << x_point << ",";
        }
        path_processing_debug_file_ << "\n";
        path_processing_debug_file_ << "processed_path_y:\n";
        for (PreciseMeters y_point : processed_path_y) {
            path_processing_debug_file_ << y_point << ",";
        }
        path_processing_debug_file_ << "\n";
        path_processing_debug_file_ << "processed_path_psi:\n";
        for (PreciseRadians psi_point : processed_path_psi) {
            path_processing_debug_file_ << psi_point << ",";
        }
        path_processing_debug_file_ << "\n";
    }
}
*/

void ControlDebugStates::WriteToFile(std::string path) const {
    /*
    std::ofstream delta_file(path + "\\\\debug_delta.csv");
    std::ofstream ef_file(path + "\\\\debug_lateral_error.csv");
    std::ofstream vehicle_state_file(path + "\\\\debug_vehicle_states.csv");
    vehicle_state_file << "x"
                       << ","
                       << "y"
                       << ","
                       << "psi"
                       << ","
                       << "time[sec]"
                       << "\n";  //<<x,y,psi>, time>
    std::ofstream motion_planning_paths_file(
        path + "\\\\debug_motion_planning_paths.csv");

    int n = this->steering_num_updates_;
    for (int i = 0; i < n; i++) {
        delta_file << this->delta_command_[i].first << "," << this->delta_command_[i].second
                   << "\n";
        ef_file << this->lateral_error_[i].first << ","
                << this->lateral_error_[i].second << "\n";
        vehicle_state_file << this->vehicle_state_[i].first[0] << ","
                           << this->vehicle_state_[i].first[1] << ","
                           << this->vehicle_state_[i].first[2] << ","
                           << this->vehicle_state_[i].second
                           << "\n";  //<<x,y,psi>, time>
    }
    delta_file.close();
    ef_file.close();
    vehicle_state_file.close();

    n = this->motion_planning_num_updates_;
    /*
    assuming the variable "ControlAPI->apply_path_smoothing" is not changed,
    therefore either spline are applied or not applied durring debug session.
    so if this->motion_planning_path_x_[i].size() ==
    this->motion_planning_path_x_processed_[i].size() then splines are applied
    */
    /*
     bool spline_smoothing_is_applied =
         this->motion_planning_path_x_.size() ==
         this->motion_planning_path_x_processed_.size();
     for (int i = 0; i < n; i++) {
         motion_planning_paths_file_
             << "time"
             << "," << this->motion_planning_update_time_[i] << "\n";
         if (spline_smoothing_is_applied) {
             motion_planning_paths_file << "x_path_:_"
                                        << ",";
             for (int j = 0; j < this->motion_planning_path_x_[i].size(); j++) {
                 motion_planning_paths_file
                     << this->motion_planning_path_x_[i][j] << ",";
             }
             motion_planning_paths_file << "\n"
                                        << "y_path_:_"
                                        << ",";
             for (int j = 0; j < this->motion_planning_path_y_[i].size(); j++) {
                 motion_planning_paths_file
                     << this->motion_planning_path_y_[i][j] << ",";
             }
         }
         motion_planning_paths_file << "\n"
                                    << "x_path_processed:_"
                                    << ",";
         for (int j = 0; j < this->motion_planning_path_x_processed_[i].size();
              j++) {
             motion_planning_paths_file << this->motion_planning_path_x_[i][j]
                                        << ",";
         }
         motion_planning_paths_file << "\n"
                                    << "y_path_processed:_"
                                    << ",";
         for (int j = 0; j < this->motion_planning_path_y_processed_[i].size();
              j++) {
             motion_planning_paths_file << this->motion_planning_path_y_[i][j]
                                        << ",";
         }
         motion_planning_paths_file << "\n"
                                    << "psi_path_:_"
                                    << ",";
         for (int j = 0; j < this->motion_planning_path_psi_[i].size(); j++) {
             motion_planning_paths_file << this->motion_planning_path_psi_[i][j]
                                        << ",";
         }
     }
     motion_planning_paths_file.close();
     std::cout << "control debug states saved output to: " << path << std::endl;
     */
}
/* B10: Removed as part of control code removal
void ControlDebugStates::Write_file_for_visualization(
    std::filesystem::path visualization_data_path,
    std::string updated_file) {
    if (!std::filesystem::exists(visualization_data_path)) {
            std::cout << "Directory does not exist." << std::endl;
            (std::filesystem::create_directory(visualization_data_path));
            std::cout << visualization_data_path << " Created" << std::endl;
        }
    if (updated_file == "motion_planning_path") {
        std::filesystem::path ref_traj_file_path = visualization_data_path
            / "reference_traj.csv";
        if (std::filesystem::exists(ref_traj_file_path)) {
            std::remove(ref_traj_file_path.string().c_str());
        }
        // write the trajectory file
        // create a vector of pairs acting as a keyword (dictionary)
        std::vector<std::vector<double>> values;
        std::vector<std::string> headers;
        headers.push_back("x");
        values.push_back(motion_planning_path_x_processed_.back());
        headers.push_back("y");
        values.push_back(motion_planning_path_y_processed_.back());
        std::vector<std::pair<std::string, std::vector<double>>> data_for_vis =
            PackVectorsToNameValuePairs(headers, values);
        PrintToCSV(ref_traj_file_path, data_for_vis);
    }
    if (updated_file == "longitudinal") {
        std::filesystem::path longitudinal_file_path = visualization_data_path
            / "longitudinal.csv";
        if (std::filesystem::exists(longitudinal_file_path)) {
            std::remove(longitudinal_file_path.string().c_str());
        }
        // write the longitudinal file
        // create a vector of pairs acting as a keyword (dictionary)
        if (last_command_update_.size() > 0) {
            std::vector<std::vector<double>> values;
            std::vector<std::string> headers;

            headers.push_back("throttle_mode");
            values.push_back(
                {static_cast<double>(throttle_mode_.back().first)});

            headers.push_back("brake_mode");
            values.push_back({static_cast<double>(brake_mode_.back().first)});

            headers.push_back("kp_throttle");
            values.push_back({kp_throttle_.back().first});

            headers.push_back("ki_throttle");
            values.push_back({ki_throttle_.back().first});

            headers.push_back("kii_throttle");
            values.push_back({kii_throttle_.back().first});

            headers.push_back("kd_throttle");
            values.push_back({kd_throttle_.back().first});

            headers.push_back("kp_brake");
            values.push_back({kp_brake_.back().first});

            headers.push_back("ki_brake");
            values.push_back({ki_brake_.back().first});

            headers.push_back("kii_brake");
            values.push_back({kii_brake_.back().first});

            headers.push_back("kd_brake");
            values.push_back({kd_brake_.back().first});

            headers.push_back("vehicle_velocity");
            values.push_back({vehicle_velocity_.back().first});

            headers.push_back("vehicle_velocity_filtered");
            values.push_back({vehicle_velocity_filtered_.back().first});

            headers.push_back("reference_velocity");
            values.push_back({reference_velocity_.back().first});

            headers.push_back("reference_velocity_filtered");
            values.push_back({reference_velocity_filtered_.back().first});

            headers.push_back("velocity_error");
            values.push_back({velocity_error_.back().first});

            headers.push_back("previous_velocity_error");
            values.push_back({previous_velocity_error_.back().first});

            headers.push_back("velocity_error_integral_throttle");
            values.push_back({velocity_error_integral_throttle_.back().first});

            headers.push_back("velocity_error_double_integral_throttle");
            values.push_back(
                {velocity_error_double_integral_throttle_.back().first});

            headers.push_back("velocity_error_integral_brake");
            values.push_back({velocity_error_integral_brake_.back().first});

            headers.push_back("velocity_error_double_integral_brake");
            values.push_back(
                {velocity_error_double_integral_brake_.back().first});

            headers.push_back("velocity_error_integral_global");
            values.push_back({velocity_error_integral_global_.back().first});

            headers.push_back("velocity_error_derivative");
            values.push_back({velocity_error_derivative_.back().first});

            headers.push_back("p_term_throttle");
            values.push_back({p_term_throttle_.back().first});

            headers.push_back("i_term_throttle");
            values.push_back({i_term_throttle_.back().first});

            headers.push_back("ii_term_throttle");
            values.push_back({ii_term_throttle_.back().first});

            headers.push_back("d_term_throttle");
            values.push_back({d_term_throttle_.back().first});

            headers.push_back("p_term_brake");
            values.push_back({p_term_brake_.back().first});

            headers.push_back("i_term_brake");
            values.push_back({i_term_brake_.back().first});

            headers.push_back("ii_term_brake");
            values.push_back({ii_term_brake_.back().first});

            headers.push_back("d_term_brake");
            values.push_back({d_term_brake_.back().first});

            headers.push_back("last_update_throttle");
            values.push_back({last_update_throttle_.back().first});

            headers.push_back("last_update_brake");
            values.push_back({last_update_brake_.back().first});

            headers.push_back("last_command_update");
            values.push_back({last_command_update_.back().first});

            headers.push_back("dt");
            values.push_back({dt_.back().first});

            headers.push_back("clock");
            values.push_back({clock_.back().first});

            headers.push_back("raw_control_throttle");
            values.push_back({raw_control_throttle_.back().first});

            headers.push_back("raw_control_brake");
            values.push_back({raw_control_brake_.back().first});

            headers.push_back("control_throttle");
            values.push_back({control_throttle_.back().first});

            headers.push_back("control_brake");
            values.push_back({control_brake_.back().first});

            headers.push_back("halt_mode_additional_brake");
            values.push_back({halt_mode_additional_brake_.back().first});

            headers.push_back("command_throttle");
            values.push_back({command_throttle_.back().first});

            headers.push_back("command_brake");
            values.push_back({command_brake_.back().first});

            std::vector<std::pair<std::string, std::vector<double>>> 
            data_for_vis =
            PackVectorsToNameValuePairs(headers, values);
            PrintToCSV(longitudinal_file_path, data_for_vis);
        }
    }
    if (updated_file == "localization") {
        std::filesystem::path vehicle_state_file_path = visualization_data_path
            / "vehicle_state.csv";
        if (std::filesystem::exists(vehicle_state_file_path)) {
            std::remove(vehicle_state_file_path.string().c_str());
        }
        // write the vehicle state file
        std::vector<std::vector<double>> values;
        std::vector<std::string> headers;
        headers.push_back("x");
        values.push_back({vehicle_state_.back().first[0]});
        headers.push_back("y");
        values.push_back({vehicle_state_.back().first[1]});
        headers.push_back("psi");
        values.push_back({vehicle_state_.back().first[2]});
        std::vector<std::pair<std::string, std::vector<double>>> data_for_vis =
            PackVectorsToNameValuePairs(headers, values);
        PrintToCSV(vehicle_state_file_path, data_for_vis);
    }
    if (updated_file == "steering") {
        std::filesystem::path steering_file_path = visualization_data_path
            / "steering.csv";
        if (std::filesystem::exists(steering_file_path)) {
            std::remove(steering_file_path.string().c_str());
        }
        // write the steering file
        if (steering_measured_.size() > 0 &&
            delta_.size() > 0 && vehicle_heading_.size() > 0) {
                std::vector<std::vector<double>> values;
                std::vector<std::string> headers;
                headers.push_back("lateral_error");
                values.push_back({lateral_error_.back().first});

                headers.push_back("vehicle_heading");
                values.push_back({vehicle_heading_.back().first});

                headers.push_back("heading_reference_raw");
                values.push_back({heading_reference_raw_.back().first});
                
                headers.push_back("heading_reference_filtered");
                values.push_back({heading_reference_filtered_.back().first});

                headers.push_back("heading_error");
                values.push_back({heading_error_.back().first});

                headers.push_back("steering_measured");
                values.push_back({steering_measured_.back().first});

                headers.push_back("steering_cmd");
                values.push_back({delta_.back().first});

                headers.push_back("steering_wheel_cmd_raw");
                values.push_back({steering_wheel_command_raw_.back().first});

                headers.push_back("steering_wheel_cmd_filtered");
                values.push_back(
                    {steering_wheel_command_filtered_.back().first});

                headers.push_back("target_point_x");
                values.push_back({control_target_points_.back().first[0]});

                headers.push_back("target_point_y");
                values.push_back({control_target_points_.back().first[1]});

                std::vector<std::pair<std::string, std::vector<double>>>
                data_for_vis = PackVectorsToNameValuePairs(headers, values);
                PrintToCSV(steering_file_path, data_for_vis);
            }
    }
}
*/
