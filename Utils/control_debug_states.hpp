/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#pragma once

#include <stdint.h>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "units.hpp"

class ControlDebugStates {
    // attributes
 public:
    // Were originally C-types long int, Dor changed to int64_t
    // Perhaps can make int16 (didn't know value ranges for variables)
    int64_t motion_planning_num_updates_;
    int64_t steering_num_updates_;
    std::vector<PreciseSeconds> steering_update_time_;
    std::vector<PreciseSeconds> motion_planning_update_time_;
    std::vector<std::vector<PreciseMeters>> motion_planning_path_x_;
    std::vector<std::vector<PreciseMeters>> motion_planning_path_y_;
    std::vector<std::vector<PreciseMeters>> motion_planning_path_x_processed_;
    std::vector<std::vector<PreciseMeters>> motion_planning_path_y_processed_;
    std::vector<std::vector<PreciseRadians>> motion_planning_path_psi_;
    std::vector<std::vector<PreciseRadians>>
        motion_planning_path_psi_processed_;
    // std::vector<std::vector<PreciseMps>> vehicle_speed_;
    // std::vector<std::vector<PreciseRadians>> vehicle_heading_;
    std::vector<std::pair<PreciseMps, PreciseSeconds>>
        vehicle_speed_;  // <vehicle_heading_, time>
    std::vector<std::pair<PreciseRadians, PreciseSeconds>>
        heading_reference_raw_;  // <vehicle_heading_, time>
    std::vector<std::pair<PreciseRadians, PreciseSeconds>>
        heading_reference_filtered_;  // <vehicle_heading_, time>
    std::vector<std::pair<PreciseRadians, PreciseSeconds>>
        vehicle_heading_;  // <vehicle_heading_, time>
    std::vector<std::pair<std::vector<double>, double>>
        vehicle_state_;  // <<x,y,psi>, time>
    std::vector<std::pair<std::vector<PreciseMeters>, PreciseSeconds>>
        control_target_points_;  // <<x,y>, time>
    std::vector<std::pair<PreciseRadians, PreciseSeconds>>
        delta_;  // <steering command, time>
    std::vector<std::pair<PreciseRadians, PreciseSeconds>>
        steering_measured_;  // <steering, time>
    std::vector<std::pair<PreciseMeters, PreciseSeconds>>
        lateral_error_;
    std::vector<std::pair<PreciseRadians, PreciseSeconds>>
        heading_error_;
    std::vector<std::pair<PreciseRadians, PreciseSeconds>>
        steering_wheel_command_raw_;
    std::vector<std::pair<PreciseRadians, PreciseSeconds>>
        steering_wheel_command_filtered_;

    // Longitudinal states

    // Modes
    std::vector<std::pair<bool, PreciseSeconds>> throttle_mode_;
    std::vector<std::pair<bool, PreciseSeconds>> brake_mode_;
    // Gains
    std::vector<std::pair<double, PreciseSeconds>> kp_throttle_;
    std::vector<std::pair<double, PreciseSeconds>> ki_throttle_;
    std::vector<std::pair<double, PreciseSeconds>> kii_throttle_;
    std::vector<std::pair<double, PreciseSeconds>> kd_throttle_;
    std::vector<std::pair<double, PreciseSeconds>> kp_brake_;
    std::vector<std::pair<double, PreciseSeconds>> ki_brake_;
    std::vector<std::pair<double, PreciseSeconds>> kii_brake_;
    std::vector<std::pair<double, PreciseSeconds>> kd_brake_;
    // Reference velocity and velocity errors of different orders
    // NOTE: Controller's vehicle_velocity_ is updated via controller,
    // unlike vehicle_speed_, which is updated from localization obj
    // via UpdateSpeed(). Duplication for compatibility with Python code.
    std::vector<std::pair<PreciseMps, PreciseSeconds>> vehicle_velocity_;
    std::vector<std::pair<PreciseMps, PreciseSeconds>>
        vehicle_velocity_filtered_;
    std::vector<std::pair<PreciseMps, PreciseSeconds>> reference_velocity_;
    std::vector<std::pair<PreciseMps, PreciseSeconds>>
        reference_velocity_filtered_;
    std::vector<std::pair<PreciseMps, PreciseSeconds>> velocity_error_;
    std::vector<std::pair<PreciseMps, PreciseSeconds>> previous_velocity_error_;
    std::vector<std::pair<PreciseMeters, PreciseSeconds>>
        velocity_error_integral_throttle_;
    std::vector<std::pair<double, PreciseSeconds>>
        velocity_error_double_integral_throttle_;
    std::vector<std::pair<PreciseMeters, PreciseSeconds>>
        velocity_error_integral_brake_;
    std::vector<std::pair<double, PreciseSeconds>>
        velocity_error_double_integral_brake_;
    std::vector<std::pair<PreciseMeters, PreciseSeconds>>
        velocity_error_integral_global_;
    std::vector<std::pair<Mps2Precise, PreciseSeconds>>
        velocity_error_derivative_;
    // PI(I)D terms
    std::vector<std::pair<double, PreciseSeconds>> p_term_throttle_;
    std::vector<std::pair<double, PreciseSeconds>> i_term_throttle_;
    std::vector<std::pair<double, PreciseSeconds>> ii_term_throttle_;
    std::vector<std::pair<double, PreciseSeconds>> d_term_throttle_;
    std::vector<std::pair<double, PreciseSeconds>> p_term_brake_;
    std::vector<std::pair<double, PreciseSeconds>> i_term_brake_;
    std::vector<std::pair<double, PreciseSeconds>> ii_term_brake_;
    std::vector<std::pair<double, PreciseSeconds>> d_term_brake_;
    // Update times
    std::vector<std::pair<PreciseSeconds, PreciseSeconds>> last_update_brake_;
    std::vector<std::pair<PreciseSeconds, PreciseSeconds>>
    last_update_throttle_;
    std::vector<std::pair<PreciseSeconds, PreciseSeconds>> last_command_update_;
    std::vector<std::pair<PreciseSeconds, PreciseSeconds>> clock_;
    std::vector<std::pair<PreciseSeconds, PreciseSeconds>> dt_;
    // Raw control signals
    std::vector<std::pair<double, PreciseSeconds>> raw_control_throttle_;
    std::vector<std::pair<double, PreciseSeconds>> raw_control_brake_;
    // Processed control signals (rate limited etc., regardless of
    // throttle-brake switching desicions)
    std::vector<std::pair<Percentage, PreciseSeconds>> control_throttle_;
    std::vector<std::pair<Percentage, PreciseSeconds>> control_brake_;
    std::vector<std::pair<double, PreciseSeconds>> halt_mode_additional_brake_;
    // Final commands given to pedals, considering switching desicions
    std::vector<std::pair<Percentage, PreciseSeconds>> command_throttle_;
    std::vector<std::pair<Percentage, PreciseSeconds>> command_brake_;

    // methods
    std::mutex lock_;
    ControlDebugStates() {}
    ControlDebugStates(bool create_debug_dir, bool localization_mode,
                       std::filesystem::path control_module_dir);
    void WriteToFile(std::string path) const;
    void UpdateLongitudinalStates(
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
    );
    void WriteLongitudinalControlStatesToFile(
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
    );
    void WriteControlStatesToFile(PreciseSeconds clock,
                                  PreciseMeters target_point_x,
                                  PreciseMeters target_point_y,
                                  PreciseRadians reference_heading_raw,
                                  PreciseRadians reference_heading_filtered,
                                  PreciseRadians vehicle_heading,
                                  PreciseRadians heading_error,
                                  PreciseMeters lateral_error,
                                  PreciseRadians delta_i, 
                                  PreciseRadians steering_cmd_raw,// in wheel angle
                                  PreciseRadians steering_cmd_filtered);// in wheel angle
    void WriteLocalizationStatesToFile(PreciseSeconds clock,
                                       PreciseMps car_speed,
                                       PreciseRadians steering_angle,
                                       PreciseMeters vehicle_x,
                                       PreciseMeters vehicle_y,
                                       PreciseRadians vehicle_psi);
    void WriteAHRSStatesToFile(PreciseSeconds OS_clock,
                               PreciseSeconds IMU_clock,
                               PreciseRadians phi_hat,
                               PreciseRadians theta_hat,
                               PreciseRadians psi_hat, 
                               PreciseRadians phiIMU,
                               PreciseRadians thetaIMU,
                               PreciseRadians psiIMU
                               );
    void WritePathProcessingStatesToFile(
        PreciseSeconds clock, PreciseSeconds image_timestamp,const std::vector<PreciseMeters>& input_path_x,
        const std::vector<PreciseMeters>& input_path_y,
        const std::vector<PreciseMeters>& processed_path_x,
        const std::vector<PreciseMeters>& processed_path_y,
        const std::vector<PreciseRadians>& processed_path_psi);
    void Write_file_for_visualization(
        std::filesystem::path visualization_data_path,
                                      std::string updated_file);

 private:
    // std::ofstream delta_file_;
    // std::ofstream ef_file_;
    // std::ofstream vehicle_state_file_;
    std::ofstream path_processing_debug_file_;
    std::ofstream localization_debug_file_;
    std::ofstream longitudinal_control_debug_file_;
    std::ofstream steering_control_debug_file_;
    std::ofstream AHRS_debug_file_;
    bool control_module_dir_exist_;
    void CreateDebugFiles(std::filesystem::path path, bool localization_mode);
};
