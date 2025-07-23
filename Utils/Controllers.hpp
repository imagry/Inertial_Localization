/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#pragma once
#include <vector>
#include <mutex>
#include "../Utils/units.hpp"
#include "../Utils/Classes.hpp"
#include <nlohmann/json.hpp>
using json = nlohmann::json;
class SteeringController{
 public:
    SteeringController(const json& control_config, const json& vehicle_config);
    virtual void CalcSteeringCommand(PreciseMeters vehicle_pos_x,
                                     PreciseMeters vehicle_pos_y,
                                     PreciseRadians vehicle_psi,
                                     PreciseMps vehicle_velocity,
                                     PreciseRadians current_steering_angle, 
                                     PreciseSeconds system_time) = 0;
    virtual void UpdatePath(const std::vector<PreciseMeters>& traj_x,
                            const std::vector<PreciseMeters>& traj_y) = 0;
    void Lock() const { lock_.lock(); };
    void Unlock() const { lock_.unlock(); };
    virtual PreciseRadians GetDelta() const { return delta_command_; };
    Trajectory traj_;
    PreciseMeters el_ = 0;
    PreciseRadians delta_command_ = 0;
    PreciseRadians e_psi_ = 0;
    PreciseRadians reference_psi_raw_ = 0;
    PreciseRadians reference_psi_filtered_ = 0;
    int target_index_ = 0;
    PreciseRadians vehicle_psi_ = 0;
    double lateral_error_deadband_ = 0.1;
    mutable std::mutex lock_;
    SecondOrderLPF reference_heading_filter_;
    SecondOrderLPF lateral_error_filter_;
    double reference_heading_gain_;
    bool filter_heading_reference_= false;
    bool filter_lateral_error_ = false;
    ContinuousAngleOnline reference_psi_cont_angle_obj_;
    ContinuousAngleOnline vehicle_heading_cont_angle_obj_;
};
class StanleyController : public SteeringController {
 public:
    // attributes
    double ks_;
    double eps_ = 0.1;
    GainScheduler heading_gain_scheduler_;
    bool apply_heading_error_gain_scheduling_ = false;
    bool adaptive_predictive_heading_ = false;
    std::vector<PreciseMeters> traj_length_;  // arc length per traj index
    int num_traj_samples_;
    int num_points_to_average_predictive_heading_=1;
    PreciseSeconds DT_for_adaptive_predictive_heading_ = 1.0;
    PreciseMps min_vel_predictive_heading_ = 1.0;
    // member functions
    StanleyController(const json& control_config, const json& vehicle_config);
    void CalcErrors(PreciseMeters vehicle_pos_x, PreciseMeters vehicle_pos_y,
                    PreciseRadians vehicle_psi, PreciseMps vehicle_velocity, PreciseSeconds system_time);
    void CalcSteeringCommand(PreciseMeters vehicle_pos_x,
                             PreciseMeters vehicle_pos_y,
                             PreciseRadians vehicle_psi,
                             PreciseMps vehicle_velocity,
                             PreciseRadians current_steering_angle, 
                             PreciseSeconds system_time) override;
    void UpdatePath(const std::vector<PreciseMeters>& traj_x,
                    const std::vector<PreciseMeters>& traj_y) override;
    void UpdatePath(const std::vector<PreciseMeters>& traj_x,
                    const std::vector<PreciseMeters>& traj_y,
                    const std::vector<PreciseRadians>& traj_psi);
    // PreciseRadians GetDelta() const override;
};
class LQRController : public SteeringController {
 public:
    // attributes
    int state_dim_ = 3;
    Eigen::MatrixXd Q_;
    double R_;
    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd X_;
    Eigen::MatrixXd K_;
    PreciseMeters WB_;
    PreciseSeconds dt_;
    PreciseRadians max_steering_angle_;
    PreciseSeconds tau_; // steering_response_time_constant_
    std::vector<PreciseMeters> traj_length_;  // arc length per traj index
    int num_traj_samples_;
    Eigen::Matrix3d saved_P_for_initial_condition_;
    int num_points_to_average_predictive_heading_=1;
    double eps_ = 0.01;
    double min_vel_ = 0.5;
    float max_iter_ = 1e4;
    LQRController(const json& control_config, const json& vehicle_config);
    void CalcErrors(PreciseMeters vehicle_pos_x, PreciseMeters vehicle_pos_y,
                    PreciseRadians vehicle_psi, PreciseSeconds system_time);
    void CalcProcessMatrices(PreciseMps v);
    void SolveLQRProblem();
    void CalcSteeringCommand(PreciseMeters vehicle_pos_x,
                             PreciseMeters vehicle_pos_y,
                             PreciseRadians vehicle_psi,
                             PreciseMps vehicle_velocity, 
                             PreciseRadians current_steering_angle,
                             PreciseSeconds system_time) override;
    void UpdatePath(const std::vector<PreciseMeters>& traj_x,
                    const std::vector<PreciseMeters>& traj_y) override;
    // PreciseRadians GetDelta() const override;
};
class PIDBasedLongitudinalController{
    
 private:
    // Controller constant parameters
    double epsilon_;
    double min_control_throttle_;  // Minimal value for throttle control signal
    double max_control_throttle_;  // Maximal value for throttle control signal
    // Maximal (absolute) throttle control signal derivative (for rate limiter)
    double max_control_derivative_throttle_;
    double min_control_brake_;  // Minimal value for brake control signal
    double max_control_brake_;  // Maximal value for brake control signal
    // Maximal (absolute) brake control signal derivative (for rate limiter)
    double max_control_derivative_brake_;


    // Default gains
    double kp_default_throttle_;  // Default proportional gain for throttle
    double ki_default_throttle_;  // Default integral gain for throttle
    double kii_default_throttle_;  // Default double-integral gain for throttle
    double kd_default_throttle_;  // Default derivative gain for throttle
    double kp_default_brake_;  // Default proportional gain for brake
    double ki_default_brake_;  // Default integral gain for brake
    double kii_default_brake_;  // Default double-integral gain for brake
    double kd_default_brake_;  // Default derivative gain for brake

    // Maximal allowed integral term for throttle PID
    Percentage max_integral_term_throttle_;
    // Maximal allowed integral term for brake PID
    Percentage max_integral_term_brake_;
    // Maximal allowed double-integral term for throttle PID
    Percentage max_double_integral_term_throttle_;
    // Maximal allowed double-integral term for brake PID
    Percentage max_double_integral_term_brake_;
    // Threshold global error integral value over which controller transits
    // from idle mode to throttle mode
    PreciseMeters threshold_throttle_mode_;
    // Threshold global error integral value under which controller transits
    // from idle mode to braking mode
    PreciseMeters threshold_brake_mode_;
    // Minimal allowed value of global error integral
    PreciseMeters min_integral_error_global_;
    // Maximal allowed value of global error integral
    PreciseMeters max_integral_error_global_;
    // Use Klegg anti-windup (zero integral on error sign change) flag
    bool anti_windup_;

    // Controller state variables

    // Flag for nthrottle mode, in which braking command is zero
    bool throttle_mode_;
    // Flag for braking mode, in which throttle command is zero
    bool brake_mode_;

    double kp_throttle_;  // Proportional gain for throttle PID controller
    double ki_throttle_;  // Integral gain for throttle PID controller
    double kii_throttle_;  // Double-integral gain for throttle PID controller
    double kd_throttle_;  // Derivative gain for throttle PID controller

    double kp_brake_;  // Proportional gain for brake PID controller
    double ki_brake_;  // Integral gain for brake PID controller
    double kii_brake_;  // Integral gain for brake PID controller
    double kd_brake_;  // Derivative gain for brake PID controller

    bool is_vehicle_velocity_initialized_;  // Flag for first speed measurement
    PreciseMps vehicle_velocity_;   // Vehicle's measured velocity
    PreciseMps vehicle_velocity_filtered_;   // Vehicle's measured velocity
    PreciseMps reference_velocity_;  // Desired vehicle velocity (from planner)
    PreciseMps reference_velocity_filtered_;  // Post-shaper target velocity
    PreciseMps velocity_error_;  // Error in velocity
    // Previous error in velocity, needed for derivative calculations
    PreciseMps previous_velocity_error_;
    // Integral of velocity error for throttle PID controller
    PreciseMeters velocity_error_integral_throttle_;
    // Double integral of velocity error for throttle PID controller
    double velocity_error_double_integral_throttle_;
    // Integral of velocity error for brake PID controller
    PreciseMeters velocity_error_integral_brake_;
    // Double integral of velocity error for brake PID controller
    double velocity_error_double_integral_brake_;
    Mps2Precise velocity_error_derivative_;  // Derivative of velocity error
    // Integral of velocity error for throttle-brake switching logic
    PreciseMeters velocity_error_integral_global_;
    // Threshold reference velocity in which braking mode is switched to
    // throttle mode when measured vehicle velocity is negative
    PreciseMps negative_velocity_brake_to_throttle_thresh_;
    // If halt mode enabled (via control config):
    // When vehicle speed is below halt_mode_vehicle_speed_ and target speed is
    // below halt_mode_target_speed_, a fed-forward braking value is added
    // to control_brake_ with a rate of halt_mode_additional_brake_rate_,
    // until control_brake reaches halt_mode_min_brake_control_.
    bool enable_halting_mode_;
    PreciseMps halt_mode_target_speed_;
    PreciseMps halt_mode_vehicle_speed_;
    Percentage halt_mode_additional_brake_rate_;
    // Assumed to be a brake control value in which vehicle is certainly static
    Percentage halt_mode_min_brake_control_;
    // Threshold vehicle speed for switching (sub-thresh) to "coasting mode"
    // In this mode, commands are more gentle
    double coasting_mode_threshold_speed_;
    // Proportional gain for throttle PID controller in coasting mode
    double kp_throttle_coasting_mode_;
    // Propotional gain for brake PID controller in coasting mode
    double kp_brake_coasting_mode_;


    double p_term_throttle_;   // Proportional control term for throttle PID
    double i_term_throttle_;   // Integral control term for throttle PID
    double ii_term_throttle_;   // Double-integral control term for throttle PID
    double d_term_throttle_;   // Derivative control term for throttle PID

    double p_term_brake_;   // Proportional control term for brake PID
    double i_term_brake_;   // Integral control term for brake PID
    double ii_term_brake_;   // Double-integral control term for brake PID
    double d_term_brake_;   // Derivative control term for brake PID

    // Latest update time for brake PID controller
    PreciseSeconds last_update_brake_;
    // Latest update time for throttle PID controller
    PreciseSeconds last_update_throttle_;
    // Latest update time for overall controller's command
    PreciseSeconds last_command_update_;
    // Controller's clock, updated when controller is called
    PreciseSeconds clock_;

    RateLimiter rate_limiter_throttle_;  // Rate limiter for throttle controller
    RateLimiter rate_limiter_brake_;  // Rate limiter for brake controller
    // Shaper for velocity measurements, to avoid discontinuity
    // Shaper order: 0 (disabled), 1 (limit acc), 2 (limit acc and jerk)
    int vehicle_velocity_shaper_order_;
    // First order shaper for vehicle velocity
    RateLimiter vehicle_velocity_shaper_first_order_;
    // Second order shaper for vehicle velocity
    SecondOrderLPF_WithRateAndAccLimit vehicle_velocity_shaper_second_order_;
    // Constants for vehicle velocity shaper
    double vehicle_velocity_shaper_freq_cutoff_;
    double vehicle_velocity_shaper_restraint_coefficient_;
    double vehicle_velocity_shaper_max_rate_;   // Max acceleration
    double vehicle_velocity_shaper_max_negative_rate_;
    double vehicle_velocity_shaper_max_acc_;  // Max jerk
    // Shaper for the received speed profile
    // Shaper order: 0 (disabled), 1 (limit acc), 2 (limit acc and jerk)
    int velocity_profile_shaper_order_;
    // TODO(Dor): might be redundant in production (profile already filtered)
    // Required for tests anyway
    // First order shaper for velocity profile
    RateLimiter velocity_profile_shaper_first_order_;
    // Second order shaper for velocity profile
    SecondOrderLPF_WithRateAndAccLimit velocity_profile_shaper_second_order_;
    // Constants for speed profile shaper
    double velocity_profile_shaper_freq_cutoff_;
    double velocity_profile_shaper_restraint_coefficient_;
    double velocity_profile_shaper_max_rate_;   // Max acceleration
    double velocity_profile_shaper_max_negative_rate_;
    double velocity_profile_shaper_max_acc_;  // Max jerk 

    PreciseSeconds dt_;  // Time interval since previous call to controller

    // Throttle control signal prior to rate limiting and saturation,
    // i.e., result of the  PID controller by the standard definition.
    double raw_control_throttle_;
    // Brake control signal prior to rate limiting and saturation,
    // i.e., result of the  PID controller by the standard definition.
    double raw_control_brake_;
    // Processed control signal of the throttle controller, regardless of
    // throttle-brake switching decisions.
    Percentage control_throttle_;
    // Processed control signal of the brake controller, regardless of
    // throttle-brake switching decisions.
    Percentage control_brake_;
    // Feedforward brake to be added to control_brake if halt mode enabled
    Percentage halt_mode_additional_brake_;
    // Final throttle command, considering switching decision.
    Percentage command_throttle_;
    // Final braking command, considering switching decision.
    Percentage command_brake_;

 public:

    // Constructor

    PIDBasedLongitudinalController(const json& control_config);

    // Object methods

    /* Updates clock and time-interval between calls to controller (dt) */
    void UpdateClock(PreciseSeconds clock);

    /* Calculates and updates control value, i.e. PID implementation. */
    void UpdatePIDController(PreciseSeconds clock);

    /* Sets current reference (target) velocity. */
    void SetReferenceVelocity(PreciseMps reference_velocity);

    /* Sets current vehicle velocity. */
    void SetVehicleVelocity(PreciseMps vehicle_velocity);
    
    /* Calculates and sets current and previous velocity error. */
    void CalcVelocityError();

    /* Calculates and sets current velocity error integral.
    Updates throttle error integral if in throttle mode.
    Updates brake error integral if in brake mode.
    Sets both integrators to zero if in idle mode.
    Updates global integrator regardless of mode. */
    void CalcVelocityErrorIntegral();

    /* Calculates and sets current velocity error double-integral.
    Updates throttle error double-integral if in throttle mode.
    Updates brake error double-integral if in brake mode.
    Sets both double-integrators to zero if in idle mode. */
    void CalcVelocityErrorDoubleIntegral();

    /* Calculates and sets current velocity error derivative. */
    void CalcVelocityErrorDerivative();

    /* Updates the modes the controller might be in -
    throttle mode and brake mode. When not in any of these modes,
    controller is in idle mode, integrating the error until either an upper
    or bottom threshold is met - causing a transition to throttle mode
    or braking mode correspondingly. */
    void UpdateThrottleBrakeSwitch(float driving_mode);

    /* Calculates an updated command (throttle and brake). */
    void CalcLongitudinalCommand(PreciseSeconds clock);

    /* Getter for longitudinal command <throttle command, brake command> */
    std::pair<Percentage, Percentage> GetLongitudinalCommand();

    /* Getter for all longitudinal controller's states */
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
    > GetControllerStates(PreciseSeconds time) const;
};
