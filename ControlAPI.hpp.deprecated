/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#pragma once

#include <stdio.h>  // to use NULL
#include <vector>
#include <utility>  // for use of pair
#include <cassert>
#include <fstream>
#include <tuple>
#include <string>
#include <nlohmann/json.hpp>
#include "Utils/Classes.hpp"
#include "Utils/units.hpp"
#include "Utils/Controllers.hpp"
#include "Utils/control_debug_states.hpp"
#include "Utils/Functions.hpp"
#include "Utils/AHRS.hpp"
#include "Utils/short_term_localization.hpp"
#include "ahrs_loc_handler.hpp"

using json = nlohmann::json;

const bool print_delay_compensation = false;
class ControlAPI {
 private:
    double current_system_time_;
    json vehicle_config_;
    json control_config_;
   //  StanleyController stanley_steering_controller_;
    std::shared_ptr<SteeringController> steering_controller_ptr_;
   //  LQRController LQR_steering_controller_;
    RateLimiter steering_cmd_rate_limiter_obj_;
    PreciseRadians steering_wheel_command_ = 0.0;
    PIDBasedLongitudinalController longitudinal_controller_;
    std::pair<Percentage, Percentage> longitudinal_command_;
    bool apply_path_smoothing_;
    bool debug_mode_;
    bool valid_motion_planning_path_;
    void CheckMotionPlanningPathValidity();
    ControlDebugStates debug_obj_;
    int ongoing_test_id_ = kInvalidTestId;
    std::shared_ptr<AHRSLocHandler> localization_handler;
    float driving_mode_ = -3.0;
    bool IsInTestState() const;
 public:
    ControlAPI(const json& vehicle_config, const json& control_config);
    ControlAPI(const std::string& vehicle_config_path = "vehicle_config.json",
                        const std::string& control_config_path = "control_config.json");
    bool MotionPlanningUpdate(const std::vector<PreciseMeters>& traj_x,
                              const std::vector<PreciseMeters>& traj_y,
                              PreciseSeconds system_time,
                              const std::string& frame = "NAV",
                              PreciseSeconds image_timestamp = 0, 
                              PreciseSeconds timestamp_for_delay_compensation = 0);
    // TODO(Dor): How to handle delay in longitudinal controller?
    bool ReferenceSpeedUpdate(PreciseMps traj_velocity,
                                 PreciseSeconds system_time);
    bool CalculateSteeringCommand(PreciseSeconds system_time);
    // Dor changed from non-const references to pointers
    void GetSteeringControllerStates(PreciseRadians* delta,
                                     PreciseMeters* ef) const;
    std::tuple<std::vector<double>, std::vector<double>>
    GetReferenceTrajectory() const;
    PreciseRadians GetDelta() const;
    PreciseMeters GetLateralError() const;
    PreciseRadians GetHeadingError() const;
    PreciseRadians GetHeadingReference() const;
    PreciseRadians GetSteeringCmd() const;
    // std::tuple<std::vector<double>,std::vector<double>> Get
    void UpdateControlDebugStates();
    void TerminateControlStates();
    void WriteDebugStates(std::string path) const;
    static const int kInvalidTestId = -1;
    void TestStart(int test_id = kInvalidTestId);
    void TestFinish();
    json GetControlConfiguration() const;
    bool CompensateForDelayBeforeMPC() const;
    PreciseSeconds GetMotionPlanningUpdateCallFrequency() const;
    // NOTE: CalculateLongitudinalCommand also calculates errors etc.,
    // i.e. assumes desired velocity is set and updates all values needed
    // to calculate the command, and finally calculates and sets command
    bool CalculateLongitudinalCommand(PreciseSeconds system_time);
   // Returns <throttle_control, brake_control>
   std::pair<Percentage, Percentage> GetLongitudinalCommand();
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
    GetLongitudinalControllerStates(PreciseSeconds system_time);
    
    void SetDrivingMode(float driving_mode);
    std::tuple<std::vector<std::vector<PreciseMeters>>, 
      PreciseSeconds> CompensateForDelay(
            vector<vector<PreciseMeters>> path_ego, 
            PreciseSeconds path_time_stamp, 
            std::string control_point="Front") const;
    std::tuple<std::vector<PreciseMeters>, std::vector<PreciseMeters>,
    std::vector<PreciseMeters>, std::vector<PreciseRadians>, bool>
    InterpulatePath(const std::vector<PreciseMeters> &x,
                    const std::vector<PreciseMeters> &y,
                    PreciseMeters ds) const;
    std::tuple<VectorXd, VectorXd,VectorXd, bool>
    InterpulatePath(const std::vector<PreciseMeters> &x,
                    const std::vector<PreciseMeters> &y,
                    const std::vector<PreciseMeters> &s_interp, 
                    std::string method="linear") const; // method='linear', 'splines' 
    // Wrapper function for linear interpolation
    std::tuple<std::vector<double>, bool>
    LinearInterpWrapper(const std::vector<double>& x,
                        const std::vector<double>& y,
                        const std::vector<double>& xi) const;
    std::tuple<std::vector<PreciseMeters>, 
           std::vector<PreciseMeters>, 
           std::vector<PreciseRadians>> 
    ConvertPathControlPoint(
                    const std::vector<PreciseMeters>& path_points_cp1_x,
                    const std::vector<PreciseMeters>& path_points_cp1_y,
                    PreciseMeters lr1, PreciseMeters lr2, PreciseMeters WB,
                    bool convert_to_cp2_frame = false) const;
    /* Update throttle pedal measurement, currently not doing anything */
    void UpdateThrottle(Percentage throttle, PreciseSeconds clock);
    /* Update throttle brake measurement, currently not doing anything. */
    void UpdateBrake(Percentage brake, PreciseSeconds clock);
};
