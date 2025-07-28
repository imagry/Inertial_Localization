/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#include <vector>
#include <numeric>
#include <cmath>
#include <algorithm>
#include "Old_Controllers.hpp"// NOLINT
#include "Functions.hpp"// NOLINT
#include "Classes.hpp"// NOLINT
#include <nlohmann/json.hpp>

using json = nlohmann::json;

int Sign(double val) { return (0 < val) - (val < 0); }

SteeringController::SteeringController(const json& control_config, const json& vehicle_config)
                  : reference_heading_gain_(double(control_config["reference_heading_gain_Stanley"])),
                    filter_heading_reference_(bool(control_config["filter_reference_heading_Stanley"])),
                    filter_lateral_error_(bool(control_config["filter_lateral_error_Stanley"])),
                    reference_heading_filter_(SecondOrderLPF(0.0, 
                                    double(control_config["heading_filter_cutoff_freq_Stanley"]), 
                                    double(1.0), //dumping ratio
                                    0.0, // initial value 
                                    0.0, // initial derivative
                                    PreciseSeconds(0.5), double(1e-6))),
                    lateral_error_filter_(SecondOrderLPF(0.0, 
                                    double(control_config["lateral_error_filter_cutoff_freq_Stanley"]), 
                                    double(1.0), //dumping ratio
                                    0.0, // initial value 
                                    0.0, // initial derivative
                                    PreciseSeconds(0.5), double(1e-6))){}
StanleyController::StanleyController(const json& control_config, const json& vehicle_config)
                  : SteeringController(control_config, vehicle_config),
                    ks_(double(control_config["stanley_gain"])),
                    eps_(double(control_config["eps_Stanley"])),
                    num_points_to_average_predictive_heading_(
                        double(control_config["num_points_to_average_predictive_heading_Stanley"])),
                    adaptive_predictive_heading_(bool(control_config["adaptive_predictive_heading_Stanley"])),
                    DT_for_adaptive_predictive_heading_(double(control_config["DT_for_adaptive_predictive_heading_Stanley"])),
                    min_vel_predictive_heading_(double(control_config["min_vel_predictive_heading_Stanley"])),
                    apply_heading_error_gain_scheduling_(bool(control_config["apply_heading_error_gain_scheduling_Stanley"])), 
                    heading_gain_scheduler_(control_config["heading_error_gain_scheduling_speed_Stanley"]){}
void StanleyController::UpdatePath(const std::vector<PreciseMeters> &traj_x,
                                   const std::vector<PreciseMeters> &traj_y) {
    Trajectory temp_traj;
    temp_traj.x_coor = traj_x;
    temp_traj.y_coor = traj_y;
    // CalcCurveHeading(temp_traj.x_coor, temp_traj.y_coor, &(temp_traj.psi));
    std::vector<PreciseRadians> temp_traj_psi;
    CalcCurveHeading(temp_traj.x_coor, temp_traj.y_coor, &(temp_traj_psi));
    ContinuousAngle(temp_traj_psi, &(temp_traj.psi));
    CalcCurveLength(temp_traj.x_coor, temp_traj.y_coor, &(temp_traj.s));
    // lock the class object to exclusive access. Namely, don't perform
    // calculations during writing values.
    std::lock_guard<std::mutex> guard(lock_);
    /*in order to lock the object to the shortest period as possible,
    we don't copy values but rather chenge the pointer to the already written
    values on the temp variable*/
    traj_.Swap(temp_traj);
    num_traj_samples_ = traj_.x_coor.size();
    // when exiting the function, lock_ is released by destructor.
}
void StanleyController::UpdatePath(
    const std::vector<PreciseMeters> &traj_x,
    const std::vector<PreciseMeters> &traj_y,
    const std::vector<PreciseRadians> &traj_psi) {
    Trajectory temp_traj;
    temp_traj.x_coor = traj_x;
    temp_traj.y_coor = traj_y;
    temp_traj.psi = traj_psi;
    CalcCurveLength(temp_traj.x_coor, temp_traj.y_coor, &(temp_traj.s));
    // lock the class object to exclusive access. Namely, don't perform
    // calculations during writing values.
    std::lock_guard<std::mutex> guard(lock_);
    /*in order to lock the object to the shortest period as possible,
    we don't copy values but rather chenge the pointer to the already written
    values on the temp variable*/
    traj_.Swap(temp_traj);
    num_traj_samples_ = traj_.x_coor.size();
    // when exiting the function, lock_ is released by destructor.
}
void StanleyController::CalcErrors(PreciseMeters vehicle_pos_x,
                                   PreciseMeters vehicle_pos_y,
                                   PreciseRadians vehicle_psi, 
                                   PreciseMps vehicle_velocity,
                                   PreciseSeconds system_time) {
    /*lock object for exclusive access. Namely, dont writ a new reperence path
    while calculating the control signal*/
    std::lock_guard<std::mutex> guard(lock_);
    vehicle_psi_ = vehicle_heading_cont_angle_obj_.Update(vehicle_psi);
    target_index_ = traj_.ProjectPoint(vehicle_pos_x, vehicle_pos_y,
                                         vehicle_psi_, true);
    PreciseMeters dx = vehicle_pos_x - traj_.x_coor[target_index_];
    PreciseMeters dy = vehicle_pos_y - traj_.y_coor[target_index_];
    std::vector<PreciseMeters> front_axle_vec_rot_90{
        cos(vehicle_psi_ - M_PI / 2.0), sin(vehicle_psi_ - M_PI / 2.0)};
    std::vector<PreciseMeters> vec_target_2_vehicle{dx, dy};
    
    el_ = vec_target_2_vehicle[0] * front_axle_vec_rot_90[0] +
          vec_target_2_vehicle[1] * front_axle_vec_rot_90[1];
    if (filter_lateral_error_){
        el_ = lateral_error_filter_.Update(system_time, el_);
    }
    el_ = deadzone(el_, lateral_error_deadband_);
    int n_points_to_average = num_points_to_average_predictive_heading_; // set as default 
    if (adaptive_predictive_heading_){
        PreciseMps vehicle_velocity_for_predictive_heading = std::max(vehicle_velocity, min_vel_predictive_heading_);
        PreciseMeters predictive_heading_range = vehicle_velocity_for_predictive_heading * DT_for_adaptive_predictive_heading_;
        for (int i = target_index_; i < traj_.s.size(); ++i) {
            if (traj_.s[i] - traj_.s[target_index_] >= predictive_heading_range) {
                n_points_to_average = std::max(i - target_index_, 1);
            break;
            }
        }
    }
    if (n_points_to_average > 1){
        // notice that there's n-1 namples of psi in traj_.psi
        int num_path_points = traj_.psi.size();
        int end_index = std::min(target_index_ + n_points_to_average,
                                 num_path_points) - 1;
        std::vector<double> sub_vector(traj_.psi.begin() + target_index_, 
                                       traj_.psi.begin() + end_index + 1); // +1 to include the end index
        reference_psi_raw_ = std::accumulate(sub_vector.begin(), sub_vector.end(), 0.0) / 
                            sub_vector.size();

    } else {
        reference_psi_raw_ = traj_.psi[target_index_];
    }
    reference_psi_raw_ = reference_psi_cont_angle_obj_.Update(reference_psi_raw_);
    if (filter_heading_reference_){
        reference_psi_filtered_ = reference_heading_filter_.Update(system_time, 
                                                                reference_psi_raw_);
    } else {
        reference_psi_filtered_ = reference_psi_raw_;
    }
    e_psi_ = FoldAngles(reference_psi_filtered_ - vehicle_psi_);
}
void StanleyController::CalcSteeringCommand(PreciseMeters vehicle_pos_x,
                                            PreciseMeters vehicle_pos_y,
                                            PreciseRadians vehicle_psi,
                                            PreciseMps vehicle_velocity, 
                                            PreciseRadians current_steering_angle, 
                                            PreciseSeconds system_time) {
    CalcErrors(vehicle_pos_x, vehicle_pos_y, vehicle_psi, vehicle_velocity, system_time);
    if (apply_heading_error_gain_scheduling_){
        // assume scheduling in KPH 
        reference_heading_gain_ = heading_gain_scheduler_.calc_gain(vehicle_velocity * 3.6);
    }
    delta_command_ = reference_heading_gain_ * e_psi_ + atan2(ks_ * el_, vehicle_velocity + eps_);
}
// PreciseRadians StanleyController::GetDelta() const { return delta_command_; }

LQRController::LQRController(const json& control_config, const json& vehicle_config)
                  : SteeringController(control_config, vehicle_config),
                    state_dim_(3),
                    Q_(Eigen::MatrixXd::Zero(state_dim_,state_dim_)), 
                    A_(Eigen::MatrixXd::Zero(state_dim_,state_dim_)), 
                    B_(Eigen::MatrixXd::Zero(state_dim_,1)), 
                    X_(Eigen::MatrixXd::Zero(state_dim_,1)),
                    K_(Eigen::MatrixXd::Zero(1, state_dim_)),  
                    R_(double(control_config["LQR_config"]["steering_command_weight"])), 
                    WB_(PreciseMeters(vehicle_config["WB"])), 
                    dt_(PreciseSeconds(1.0 / double(control_config["nominal_IMU_freq"]))), 
                    max_steering_angle_(PreciseRadians(vehicle_config["MAX_STEER"])), 
                    tau_(PreciseSeconds(control_config["LQR_config"]["steering_response_time_constant"])),
                    saved_P_for_initial_condition_(Eigen::Matrix3d::Zero()), 
                    num_points_to_average_predictive_heading_(
                        double(control_config["LQR_config"]["num_points_to_average_predictive_heading_LQR"])),
                    eps_(double(control_config["LQR_config"]["eps_LQR"])),
                    max_iter_(int(control_config["LQR_config"]["max_itarations_LQR"]))
                    {
                    Q_.diagonal() << control_config["LQR_config"]["lateral_error_weight"], 
                                    control_config["LQR_config"]["heading_error_weight"], 
                                    control_config["LQR_config"]["steering_angle_weight"];
                    saved_P_for_initial_condition_ = Q_;
                    }
void LQRController::UpdatePath(const std::vector<PreciseMeters> &traj_x,
                                const std::vector<PreciseMeters> &traj_y) {
    Trajectory temp_traj;
    temp_traj.x_coor = traj_x;
    temp_traj.y_coor = traj_y;
    // CalcCurveHeading(temp_traj.x_coor, temp_traj.y_coor, &(temp_traj.psi));
    std::vector<PreciseRadians> temp_traj_psi;
    CalcCurveHeading(temp_traj.x_coor, temp_traj.y_coor, &(temp_traj_psi));
    ContinuousAngle(temp_traj_psi, &(temp_traj.psi));
    CalcCurveLength(temp_traj.x_coor, temp_traj.y_coor, &(temp_traj.s));
    // lock the class object to exclusive access. Namely, don't perform
    // calculations during writing values.
    std::lock_guard<std::mutex> guard(lock_);
    /*in order to lock the object to the shortest period as possible,
    we don't copy values but rather chenge the pointer to the already written
    values on the temp variable*/
    traj_.Swap(temp_traj);
    num_traj_samples_ = traj_.x_coor.size();
    // when exiting the function, lock_ is released by destructor.
}
void LQRController::CalcErrors(PreciseMeters vehicle_pos_x,
                               PreciseMeters vehicle_pos_y,
                               PreciseRadians vehicle_psi, 
                               PreciseSeconds system_time) {
    /*lock object for exclusive access. Namely, dont writ a new reperence path
    while calculating the control signal*/
    std::lock_guard<std::mutex> guard(lock_);
    vehicle_psi_ = vehicle_heading_cont_angle_obj_.Update(vehicle_psi);
    target_index_ = traj_.ProjectPoint(vehicle_pos_x, vehicle_pos_y,
                                         vehicle_psi_, true);
    PreciseMeters dx = traj_.x_coor[target_index_] - vehicle_pos_x;
    PreciseMeters dy = traj_.y_coor[target_index_] - vehicle_pos_y;
    /* y ego frame axis in w frame in a NED configuration (z pointing down y is right)*/
    std::vector<PreciseMeters> ego_frame_y_axis_in_w_frame{
        cos(vehicle_psi_ + M_PI / 2.0), sin(vehicle_psi_ + M_PI / 2.0)};
    std::vector<PreciseMeters> vec_vehicle_2_target{dx, dy};
    
    el_ =  (vec_vehicle_2_target[0] * ego_frame_y_axis_in_w_frame[0] +
          vec_vehicle_2_target[1] * ego_frame_y_axis_in_w_frame[1]);
    el_ = deadzone(el_, lateral_error_deadband_);
    /*
    sign: in the stanley controller, steering to the direction of el minimizes el, 
        in LQR steering is to the negative direction of el
    */
    if (num_points_to_average_predictive_heading_ > 1){
        int num_path_points = traj_.x_coor.size();
        int end_index = std::min(target_index_ + num_points_to_average_predictive_heading_,
                                 num_path_points) - 1;
        std::vector<double> sub_vector(traj_.psi.begin() + target_index_, 
                                       traj_.psi.begin() + end_index);
        reference_psi_raw_ = std::accumulate(sub_vector.begin(), sub_vector.end(), 0.0) / 
                            sub_vector.size();
    } else {
        reference_psi_raw_ = traj_.psi[target_index_];
    }
    reference_psi_raw_ = reference_psi_cont_angle_obj_.Update(reference_psi_raw_);
    if (filter_heading_reference_){
        reference_psi_filtered_ = reference_heading_filter_.Update(system_time, 
                                                                reference_psi_raw_);
    } else {
        reference_psi_filtered_ = reference_psi_raw_;
    }
    e_psi_ = FoldAngles(reference_psi_filtered_ - vehicle_psi_);
}
void LQRController::CalcProcessMatrices(PreciseMps v){
    A_ << 1, v * dt_, v * dt_,
          0, 1       , v * dt_ / WB_,
          0, 0       , 1 - dt_ / tau_;
    B_ << 0,
          0,
          dt_ / tau_;
}
void LQRController::SolveLQRProblem(){
    Eigen::MatrixXd A = A_;
    Eigen::MatrixXd B = B_;
    Eigen::MatrixXd Q = Q_;
    double R = R_;
    double tolerance = eps_;
    int max_num_iteration = max_iter_;

    // Ensure matrix dimensions are compatible
    assert(A.rows() == A.cols() && B.rows() == A.rows() && Q.rows() == Q.cols() && Q.rows() == A.cols() );

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(Q.rows(), 1);
    Eigen::MatrixXd AT = A.transpose();
    Eigen::MatrixXd BT = B.transpose();
    Eigen::MatrixXd MT = M.transpose();

    Eigen::MatrixXd P = saved_P_for_initial_condition_;
    int num_iteration = 0;
    double diff = std::numeric_limits<double>::infinity();
    bool print_during_optimization = false;

    if (print_during_optimization) {
        std::cout << "\n=========== LQR solve ==============" << std::endl;
    }

    while (num_iteration < max_num_iteration && diff > tolerance) {
        num_iteration++;
        /*
        we know that R is 1 X1 becaus the system has a single input therefore
        (R + BT * P * B).inverse() = 1 / (R + BT * P * B)
        */  
        double R_pluse_BT_times_P_times_B_inv = 1 / (R + (BT * P * B)(0, 0));
        Eigen::MatrixXd P_next = AT * P * A - (AT * P * B + M) * R_pluse_BT_times_P_times_B_inv * (BT * P * A + MT) + Q;

        // Check the difference between P and P_next
        diff = (P_next - P).cwiseAbs().maxCoeff();
        if (print_during_optimization) {
            std::cout << "iteration [" << num_iteration << "]: diff = " << diff << ", P_next = " << P_next << std::endl;
        }
        P = P_next;
    }

    if (num_iteration >= max_num_iteration) {
        PreciseSeconds now = WhatsTheTimeSeconds();
    } 

    saved_P_for_initial_condition_ = P;
    double R_pluse_BT_times_P_times_B_inv = 1 / (R + (BT * P * B)(0, 0));
    K_ = R_pluse_BT_times_P_times_B_inv * (BT * P * A + MT);
}
void LQRController::CalcSteeringCommand(PreciseMeters vehicle_pos_x,
                             PreciseMeters vehicle_pos_y,
                             PreciseRadians vehicle_psi,
                             PreciseMps vehicle_velocity,
                             PreciseRadians current_steering_angle,
                             PreciseSeconds system_time){
                                CalcErrors(vehicle_pos_x, vehicle_pos_y, vehicle_psi, system_time);
                                X_ << -el_, -e_psi_, current_steering_angle;
                                CalcProcessMatrices(std::max(vehicle_velocity, min_vel_));
                                SolveLQRProblem();
                                // Perform the matrix multiplication and convert the result to a double
                                delta_command_ = (-K_ * X_).coeff(0, 0);
                                if (false){
                                    PreciseMeters el_debug= 0.0;
                                    PreciseRadians e_psi__debug= 0.2;
                                    Eigen::MatrixXd X_debug(3, 1);
                                    X_debug<<-el_debug, -e_psi__debug, current_steering_angle;
                                    PreciseRadians delta_command_debug = (-K_ * X_debug).coeff(0, 0);
                                    std::cout << "K:\n" << K_ << std::endl;
                                    std::cout << "X_debug:\n" << X_debug << std::endl;
                                    std::cout << "el_debug:\n" << el_debug << std::endl;
                                    std::cout << "e_psi__debug:\n" << e_psi__debug << std::endl;
                                    std::cout << "delta_command_debug:\n" << delta_command_debug << std::endl;
                                }
                                delta_command_ = LimitValue(delta_command_, - max_steering_angle_, max_steering_angle_);
                             }

PIDBasedLongitudinalController::PIDBasedLongitudinalController(
    const json& control_config) :
    clock_(0.0),
        min_control_throttle_(static_cast<double>(control_config["Longitudinal_controller"]["min_control_throttle"])),
        min_control_brake_(static_cast<double>(control_config["Longitudinal_controller"]["min_control_brake"])),
        max_control_throttle_(static_cast<double>(control_config["Longitudinal_controller"]["max_control_throttle"])),
        max_control_brake_(static_cast<double>(control_config["Longitudinal_controller"]["max_control_brake"])),
        max_control_derivative_throttle_(static_cast<double>(control_config["Longitudinal_controller"]["max_control_derivative_throttle"])),
        max_control_derivative_brake_(static_cast<double>(control_config["Longitudinal_controller"]["max_control_derivative_brake"])),
        kp_default_throttle_(static_cast<double>(control_config["Longitudinal_controller"]["kp_default_throttle"])),
        ki_default_throttle_(static_cast<double>(control_config["Longitudinal_controller"]["ki_default_throttle"])),
        kii_default_throttle_(static_cast<double>(control_config["Longitudinal_controller"]["kii_default_throttle"])),
        kd_default_throttle_(static_cast<double>(control_config["Longitudinal_controller"]["kd_default_throttle"])),
        kp_default_brake_(static_cast<double>(control_config["Longitudinal_controller"]["kp_default_brake"])),
        ki_default_brake_(static_cast<double>(control_config["Longitudinal_controller"]["ki_default_brake"])),
        kii_default_brake_(static_cast<double>(control_config["Longitudinal_controller"]["kii_default_brake"])),
        kd_default_brake_(static_cast<double>(control_config["Longitudinal_controller"]["kd_default_brake"])),
        max_integral_term_throttle_(static_cast<double>(control_config["Longitudinal_controller"]["max_integral_term_throttle"])),
        max_double_integral_term_throttle_(static_cast<double>(control_config["Longitudinal_controller"]["max_double_integral_term_throttle"])),
        max_integral_term_brake_(static_cast<double>(control_config["Longitudinal_controller"]["max_integral_term_brake"])),
        max_double_integral_term_brake_(static_cast<double>(control_config["Longitudinal_controller"]["max_double_integral_term_brake"])),
        min_integral_error_global_(static_cast<double>(control_config["Longitudinal_controller"]["min_integral_error_global"])),
        max_integral_error_global_(static_cast<double>(control_config["Longitudinal_controller"]["max_integral_error_global"])),
        threshold_throttle_mode_(static_cast<double>(control_config["Longitudinal_controller"]["threshold_throttle_mode"])),
        threshold_brake_mode_(static_cast<double>(control_config["Longitudinal_controller"]["threshold_brake_mode"])),
        anti_windup_(static_cast<bool>(control_config["Longitudinal_controller"]["anti_windup"])),
        epsilon_(static_cast<double>(control_config["Longitudinal_controller"]["epsilon"])),
        negative_velocity_brake_to_throttle_thresh_(static_cast<double>(control_config["Longitudinal_controller"]["negative_velocity_brake_to_throttle_thresh"])),
        enable_halting_mode_(static_cast<bool>(control_config["Longitudinal_controller"]["enable_halting_mode"])),
        halt_mode_target_speed_(static_cast<double>(control_config["Longitudinal_controller"]["halt_mode_target_speed"])),
        halt_mode_vehicle_speed_(static_cast<double>(control_config["Longitudinal_controller"]["halt_mode_vehicle_speed"])),
        halt_mode_additional_brake_rate_(static_cast<double>(control_config["Longitudinal_controller"]["halt_mode_additional_brake_rate"])),
        halt_mode_min_brake_control_(static_cast<double>(control_config["Longitudinal_controller"]["halt_mode_min_brake_control"])),
        coasting_mode_threshold_speed_(static_cast<double>(control_config["Longitudinal_controller"]["coasting_mode_threshold_speed"])),
        kp_throttle_coasting_mode_(static_cast<double>(control_config["Longitudinal_controller"]["kp_throttle_coasting_mode"])),
        kp_brake_coasting_mode_(static_cast<double>(control_config["Longitudinal_controller"]["kp_brake_coasting_mode"])),
        // Initialize controller state variables
        throttle_mode_(false),
        brake_mode_(true),
        kp_throttle_(static_cast<double>(control_config["Longitudinal_controller"]["kp_default_throttle"])),
        ki_throttle_(static_cast<double>(control_config["Longitudinal_controller"]["ki_default_throttle"])),
        kii_throttle_(static_cast<double>(control_config["Longitudinal_controller"]["kii_default_throttle"])),
        kd_throttle_(static_cast<double>(control_config["Longitudinal_controller"]["kd_default_throttle"])),
        kp_brake_(static_cast<double>(control_config["Longitudinal_controller"]["kp_default_brake"])),
        ki_brake_(static_cast<double>(control_config["Longitudinal_controller"]["ki_default_brake"])),
        kii_brake_(static_cast<double>(control_config["Longitudinal_controller"]["kii_default_brake"])),
        kd_brake_(static_cast<double>(control_config["Longitudinal_controller"]["kd_default_brake"])),
        is_vehicle_velocity_initialized_(false),
        vehicle_velocity_(0.0),
        vehicle_velocity_filtered_(0.0),
        reference_velocity_(0.0),
        reference_velocity_filtered_(0.0),
        velocity_error_(0.0),
        previous_velocity_error_(0.0),
        velocity_error_integral_throttle_(0.0),
        velocity_error_double_integral_throttle_(0.0),
        velocity_error_integral_brake_(0.0),
        velocity_error_double_integral_brake_(0.0),
        velocity_error_derivative_(0.0),
        velocity_error_integral_global_(0.0),
        p_term_throttle_(0.0),
        i_term_throttle_(0.0),
        ii_term_throttle_(0.0),
        d_term_throttle_(0.0),
        p_term_brake_(0.0),
        i_term_brake_(0.0),
        ii_term_brake_(0.0),
        d_term_brake_(0.0),
        last_update_brake_(0.0),
        last_update_throttle_(0.0),
        last_command_update_(0.0),
        rate_limiter_throttle_(RateLimiter(
            static_cast<double>(control_config["Longitudinal_controller"]["max_control_derivative_throttle"]),
            0.0,
            0.0)),
        rate_limiter_brake_(RateLimiter(
            static_cast<double>(control_config["Longitudinal_controller"]["max_control_derivative_brake"]),
            0.0,
            0.0)),
        vehicle_velocity_shaper_order_(static_cast<double>(control_config["Longitudinal_controller"]["vehicle_velocity_shaper_order"])),
        vehicle_velocity_shaper_first_order_(RateLimiter(
            static_cast<double>(control_config["Longitudinal_controller"]["vehicle_velocity_shaper_max_rate"]),
            0.0,
            0.0,
            2.0,
            1e-6)),
        vehicle_velocity_shaper_second_order_(
                SecondOrderLPF_WithRateAndAccLimit(
                0.0,
                static_cast<double>(control_config["Longitudinal_controller"]["vehicle_velocity_shaper_freq_cutoff"]),
                static_cast<double>(control_config["Longitudinal_controller"]["vehicle_velocity_shaper_restraint_coefficient"]),
                static_cast<double>(control_config["Longitudinal_controller"]["vehicle_velocity_shaper_max_rate"]),
                static_cast<double>(control_config["Longitudinal_controller"]["vehicle_velocity_shaper_max_negative_rate"]),
                static_cast<double>(control_config["Longitudinal_controller"]["vehicle_velocity_shaper_max_acc"]),
                2.0,
                1e-6)
            ),
        velocity_profile_shaper_order_(static_cast<double>(control_config["Longitudinal_controller"]["velocity_profile_shaper_order"])),
        velocity_profile_shaper_first_order_(RateLimiter(
            static_cast<double>(control_config["Longitudinal_controller"]["velocity_profile_shaper_max_rate"]),
            0.0,
            0.0,
            2.0,
            1e-6)),
        velocity_profile_shaper_second_order_(
            SecondOrderLPF_WithRateAndAccLimit(
                0.0,
                static_cast<double>(control_config["Longitudinal_controller"]["velocity_profile_shaper_freq_cutoff"]),
                static_cast<double>(control_config["Longitudinal_controller"]["velocity_profile_shaper_restraint_coefficient"]),
                static_cast<double>(control_config["Longitudinal_controller"]["velocity_profile_shaper_max_rate"]),
                static_cast<double>(control_config["Longitudinal_controller"]["velocity_profile_shaper_max_negative_rate"]),
                static_cast<double>(control_config["Longitudinal_controller"]["velocity_profile_shaper_max_acc"]),
                2.0,
                1e-6)
            ),
        dt_(0.0),
        raw_control_throttle_(0.0),
        raw_control_brake_(0.0),
        control_throttle_(0.0),
        control_brake_(0.0),
        halt_mode_additional_brake_(0.0),
        command_throttle_(0.0),
        command_brake_(100.0) {}

void PIDBasedLongitudinalController::UpdatePIDController(
    PreciseSeconds clock) {
    // Temp variable for control value before limiting to min and max values
    Percentage unlimited_control;
    // Throttle update
    if (throttle_mode_ == true)
    {
        /* Calculate PID terms */
        p_term_throttle_ = kp_throttle_ * velocity_error_;
        // NOTE: velocity_error_integral_ already limited, no need to check
        i_term_throttle_ = ki_throttle_ * velocity_error_integral_throttle_;
        ii_term_throttle_ =
            kii_throttle_ * velocity_error_double_integral_throttle_;
        d_term_throttle_ = kd_throttle_ * velocity_error_derivative_;

        /* Set control signal to value & rate limited sum of PID terms */
        raw_control_throttle_ = p_term_throttle_ + i_term_throttle_
            + ii_term_throttle_ + d_term_throttle_;

        // If rate limiter enabled
        if (max_control_derivative_throttle_ >= 0.0) {
            unlimited_control = rate_limiter_throttle_.Update(
                clock, raw_control_throttle_);
        } else {
            unlimited_control = raw_control_throttle_;
        }

        // Limit actual control command to min and max values
        control_throttle_ = std::max(
            std::min(unlimited_control, max_control_throttle_),
            min_control_throttle_);
        last_update_throttle_ = clock;

    } else if (brake_mode_ == true){    // Brake update
        /* Calculate PID terms */
        // Note: in case of negative vehicle velocity, need to flip sign of
        // proportional and derivative terms.
        // Integral terms are not flipped, but the error accumulated in the
        // first integrator is flipped. Thus the integral terms remain
        // continuous when vehicle velocity changes sign
        p_term_brake_ = copysign(1.0, vehicle_velocity_) * kp_brake_ *
            velocity_error_;
        // NOTE: velocity_error_integral_ already limited, no need to check
        i_term_brake_ = ki_brake_ * velocity_error_integral_brake_;
        ii_term_brake_ = kii_brake_ * velocity_error_double_integral_brake_;
        d_term_brake_ = copysign(1.0, vehicle_velocity_) * kd_brake_ *
            velocity_error_derivative_;

        /* Set control signal to value & rate limited sum of PID terms */
        raw_control_brake_ = p_term_brake_ + i_term_brake_ + ii_term_brake_
            + d_term_brake_;

        // If rate limiter enabled
        if (max_control_derivative_brake_ >= 0.0) {
            unlimited_control = rate_limiter_brake_.Update(
                clock, raw_control_brake_);
        } else {
            unlimited_control = raw_control_brake_;
        }

        // Limit actual control command to min and max values
        control_brake_ = std::max(
            std::min(unlimited_control, max_control_brake_),
            min_control_brake_);
        last_update_brake_ = clock;
    }
}

void PIDBasedLongitudinalController::SetVehicleVelocity(
    PreciseMps vehicle_velocity) {
    // Assumed NaN values are filtered preemptively
    vehicle_velocity_ = vehicle_velocity;
    // Filter raw vehicle velocity with shaper
    if (vehicle_velocity_shaper_order_ == 1) {
        if (!is_vehicle_velocity_initialized_ && dt_ != 0.0) {
            vehicle_velocity_shaper_first_order_.Reset(
                clock_,
                vehicle_velocity);
            vehicle_velocity_filtered_ = vehicle_velocity;
            is_vehicle_velocity_initialized_ = true;
        } else {
            vehicle_velocity_filtered_ =
            vehicle_velocity_shaper_first_order_.Update(
                clock_,
                vehicle_velocity);
        }
    } else if (vehicle_velocity_shaper_order_ == 2) {
        if (!is_vehicle_velocity_initialized_ || dt_ > 1.0) {
            vehicle_velocity_shaper_second_order_.Reset(
                clock_,
                vehicle_velocity,
                0.0);
            vehicle_velocity_filtered_ = vehicle_velocity;
            is_vehicle_velocity_initialized_ = true;
        } else {
            vehicle_velocity_filtered_ =
                vehicle_velocity_shaper_second_order_.Update(
                    clock_,
                    vehicle_velocity,
                    vehicle_velocity,
                    0.0).first;
        }
    } else {
        vehicle_velocity_filtered_ = vehicle_velocity_;
        if (!is_vehicle_velocity_initialized_  && dt_ != 0.0) {
                is_vehicle_velocity_initialized_ = true;
        }
    }
}

void PIDBasedLongitudinalController::SetReferenceVelocity(
    PreciseMps reference_velocity) {
    // Assumed NaN values are filtered preemptively
    reference_velocity_ = reference_velocity;
    // Filter raw (i.e. planner's) reference velocity with shaper
    if (velocity_profile_shaper_order_ == 1) {
        reference_velocity_filtered_ =
            vehicle_velocity_shaper_first_order_.Update(
                clock_,
                reference_velocity);
    } else if (velocity_profile_shaper_order_ == 2) {
        reference_velocity_filtered_ =
            velocity_profile_shaper_second_order_.Update(
                clock_,
                reference_velocity,
                reference_velocity,
                0.0).first;
    } else {
        reference_velocity_filtered_ = reference_velocity;
    }
}

void PIDBasedLongitudinalController::CalcVelocityError() {
    // Set previous velocity error (for derivative calculation later)
    previous_velocity_error_ = velocity_error_;
    // Set new velocity error
    velocity_error_ = reference_velocity_filtered_ - vehicle_velocity_filtered_;
}

void PIDBasedLongitudinalController::CalcVelocityErrorIntegral() {
    Percentage unlimited_integral;
    int integral_sign;

    // Update throttle error integral
    if (throttle_mode_ == true) {
        unlimited_integral = velocity_error_integral_throttle_ +
        velocity_error_ * dt_;
        integral_sign = Sign(unlimited_integral);
        if (velocity_error_ < 0.0 && anti_windup_ && vehicle_velocity_ > 0.0) {
            velocity_error_integral_throttle_ = 0.0;
        } else {
            velocity_error_integral_throttle_ = integral_sign * std::min(
            integral_sign * unlimited_integral,
            max_integral_term_throttle_ / std::max(ki_throttle_, epsilon_));
        }
    } else if (brake_mode_ == true) {
        // Update brake error integral
        // If vehicle velocity is negative, subtract instead of add
        unlimited_integral = velocity_error_integral_brake_ +
        copysign(1.0, vehicle_velocity_) * velocity_error_ * dt_;
        integral_sign = Sign(unlimited_integral);
        if (velocity_error_ > 0.0 && anti_windup_ && vehicle_velocity_ > 0.0) {
            velocity_error_integral_throttle_ = 0.0;
        } else {
            velocity_error_integral_brake_ = integral_sign * std::min(
            integral_sign * unlimited_integral,
            max_integral_term_brake_ / std::max(ki_brake_, epsilon_));
        }
    } else {   // Idle mode, update global integral
        unlimited_integral = velocity_error_integral_global_ +
            velocity_error_ * dt_;
        velocity_error_integral_global_ = std::min(
            std::max(unlimited_integral, min_integral_error_global_),
            max_integral_error_global_);
    }
}

void PIDBasedLongitudinalController::CalcVelocityErrorDoubleIntegral() {
    Percentage unlimited_double_integral;
    int double_integral_sign;

    // Update throttle error double integral
    if (throttle_mode_ == true) {
        unlimited_double_integral = velocity_error_double_integral_throttle_ +
        velocity_error_integral_throttle_ * dt_;
        double_integral_sign = Sign(unlimited_double_integral);
        if (velocity_error_ < 0.0 && anti_windup_ && vehicle_velocity_ > 0.0) {
            velocity_error_integral_throttle_ = 0.0;
        } else {
            velocity_error_double_integral_throttle_ = double_integral_sign *
            std::min(
            double_integral_sign * unlimited_double_integral,
            max_double_integral_term_throttle_ / std::max(kii_throttle_, epsilon_));
        }
    } else if (brake_mode_ == true) {
    // Update brake error integral
        unlimited_double_integral = velocity_error_double_integral_brake_ +
        velocity_error_integral_brake_ * dt_;
        double_integral_sign = Sign(unlimited_double_integral);
        if (velocity_error_ > 0.0 && anti_windup_ && vehicle_velocity_ > 0.0) {
            velocity_error_integral_throttle_ = 0.0;
        } else {
            velocity_error_double_integral_brake_ = double_integral_sign * 
            std::min(
            double_integral_sign * unlimited_double_integral,
            max_double_integral_term_brake_ / std::max(kii_brake_, epsilon_));
        }
    }
}

void PIDBasedLongitudinalController::CalcVelocityErrorDerivative() {
    velocity_error_derivative_ = (
        velocity_error_ - previous_velocity_error_) / std::max(dt_, epsilon_);
}

/* Implements throttle-brake switching logic. Updates the mode
the controller might be in - brake mode, throttle mode or none (idle mode). */
void PIDBasedLongitudinalController::UpdateThrottleBrakeSwitch(
    float driving_mode) {
    // NOTE: we set an integrator to zero when entering the corresponding mode.

    // If driving mode is not full or speed only, enter idle mode
    if (driving_mode != 6.0 && driving_mode != 7.0)
    {
        throttle_mode_ = false;
        brake_mode_ = false;
        // Make sure global integrator is set to zero
        velocity_error_integral_global_ = 0.0;
        // Reset error derivative and integrals of all orders to zero
        velocity_error_integral_throttle_ = 0.0;
        velocity_error_integral_brake_ = 0.0;
        velocity_error_double_integral_throttle_ = 0.0;
        velocity_error_double_integral_brake_ = 0.0;
        velocity_error_derivative_ = 0.0;
        // Reset all terms to zero
        p_term_throttle_ = 0.0;
        i_term_throttle_ = 0.0;
        ii_term_throttle_ = 0.0;
        d_term_throttle_ = 0.0;
        p_term_brake_ = 0.0;
        i_term_brake_ = 0.0;
        ii_term_brake_ = 0.0;
        d_term_brake_ = 0.0;
        // Reset controls
        raw_control_throttle_ = 0.0;
        control_throttle_ = 0.0;
        raw_control_brake_ = 0.0;
        control_brake_ = 0.0;
        return;
    }
    // If vehicle velocity is negative for some reason (pitch, numeric,
    // speed-estimation error, etc.)
    if (vehicle_velocity_ < 0.0) {
        // If target speed is 0 or slightly higher
        // (negative_velocity_brake_to_throttle_thresh_ defines the threshold)
        if (reference_velocity_filtered_ <=
            negative_velocity_brake_to_throttle_thresh_) {
            // If not in braking mode, enforce braking mode to halt
            if (!brake_mode_) {
                throttle_mode_ = false;
                brake_mode_ = true;
                // Make sure global integrator is set to zero
                velocity_error_integral_global_ = 0.0;
                // Reset error derivative and throttle integrals of all orders to zero
                velocity_error_integral_throttle_ = 0.0;
                velocity_error_double_integral_throttle_ = 0.0;
                velocity_error_derivative_ = 0.0;
                // Reset all throttle terms to zero
                p_term_throttle_ = 0.0;
                i_term_throttle_ = 0.0;
                ii_term_throttle_ = 0.0;
                d_term_throttle_ = 0.0;
                // Reset throttle control
                raw_control_throttle_ = 0.0;
                control_throttle_ = 0.0;
            }
            // NOTE: If target speed is 0 and in braking mode,
            // remain in braking mode (don't switch mode, just return here)
            return;

        // If target speed is positive and not in throttle mode,
        // enforce throttle mode
        } else if (!throttle_mode_) {
            throttle_mode_ = true;
            brake_mode_ = false;
            // Make sure global integrator is set to zero
            velocity_error_integral_global_ = 0.0;
            // Reset error derivative and brake integrals of all orders to zero
            velocity_error_integral_brake_ = 0.0;
            velocity_error_double_integral_brake_ = 0.0;
            velocity_error_derivative_ = 0.0;
            // Reset all brake terms to zero
            p_term_brake_ = 0.0;
            i_term_brake_ = 0.0;
            ii_term_brake_ = 0.0;
            d_term_brake_ = 0.0;
            // Reset brake control
            raw_control_brake_ = 0.0;
            control_brake_ = 0.0;
        }
        // If target speed is positive and in throttle mode,
        // maintain throttle mode and just return here (don't switch mode)
        return;
    }
    // If not in negative velocity, and in coasting mode,
    // set to coasting mode gains
    if (vehicle_velocity_ < coasting_mode_threshold_speed_) {
        kp_throttle_ = kp_throttle_coasting_mode_;
        kp_brake_ = kp_brake_coasting_mode_;
    } else {
        // If not in coasting mode, set to default gains
        kp_throttle_ = kp_default_throttle_;
        kp_brake_ = kp_default_brake_;
    }
    // If current active control command is zero, enter idle mode
    if ((throttle_mode_ == true && control_throttle_ == 0.0) ||
        (brake_mode_ == true && control_brake_ == 0.0))
        {
            throttle_mode_ = false;
            brake_mode_ = false;
            // Make sure global integrator is set to zero
            velocity_error_integral_global_ = 0.0;
            // Reset error derivative and integrals of all orders to zero
            velocity_error_integral_throttle_ = 0.0;
            velocity_error_integral_brake_ = 0.0;
            velocity_error_double_integral_throttle_ = 0.0;
            velocity_error_double_integral_brake_ = 0.0;
            velocity_error_derivative_ = 0.0;
            // Reset all terms to zero
            p_term_throttle_ = 0.0;
            i_term_throttle_ = 0.0;
            ii_term_throttle_ = 0.0;
            d_term_throttle_ = 0.0;
            p_term_brake_ = 0.0;
            i_term_brake_ = 0.0;
            ii_term_brake_ = 0.0;
            d_term_brake_ = 0.0;
            // Reset controls
            raw_control_throttle_ = 0.0;
            control_throttle_ = 0.0;
            raw_control_brake_ = 0.0;
            control_brake_ = 0.0;

        } else if (throttle_mode_ == false && brake_mode_ == false) {
            // If currently in idle mode

            // If global integral reached upper threshold, change to throttle
            if (velocity_error_integral_global_ > threshold_throttle_mode_) {
                throttle_mode_ = true;
                // Set throttle integrator to zero, to start accumulating
                velocity_error_integral_throttle_ = 0.0;
                // Reset global integrator
                velocity_error_integral_global_ = 0.0;
            } else if (
                velocity_error_integral_global_ < threshold_brake_mode_) {
                brake_mode_ = true;
                // Set brake integrator to zero, to start accumulating
                velocity_error_integral_brake_ = 0.0;
                // Reset global integrator
                velocity_error_integral_global_ = 0.0;
                }
        }
}

// Calculate actual command given to pedals
void PIDBasedLongitudinalController::CalcLongitudinalCommand(
    PreciseSeconds clock) {

    // If not in throttle mode, throttle command is zero
    if (throttle_mode_ == false) {
        command_throttle_ = 0.0;
        // If in brake mode, give braking command
        if (brake_mode_ == true) {
            // If halt mode enabled and car & target speeds close to zero,
            // accumulate additive braking
            if (enable_halting_mode_ &&
                reference_velocity_filtered_ < halt_mode_target_speed_ &&
                abs(vehicle_velocity_filtered_) < halt_mode_vehicle_speed_) {
                    halt_mode_additional_brake_ +=
                        halt_mode_additional_brake_rate_ * dt_;
                    control_brake_ = std::max(
                        control_brake_ - halt_mode_additional_brake_,
                        halt_mode_min_brake_control_);
                } else {
                    // If leaving halt mode, i.e. halt mode additional brake
                    // is non-zero, reset halt mode additional brake and
                    // braking PID to zero, and switch to idle
                    if (halt_mode_additional_brake_ != 0.0) {
                        halt_mode_additional_brake_ = 0.0;
                        // Reset all terms to zero
                        p_term_brake_ = 0.0;
                        i_term_brake_ = 0.0;
                        ii_term_brake_ = 0.0;
                        d_term_brake_ = 0.0;
                        raw_control_brake_ = 0.0;
                        control_brake_ = 0.0;
                        velocity_error_integral_global_ = 0.0;
                        brake_mode_ = false;
                    }
                }
            // NOTE: Brake control signal is negative, command is positive
            command_brake_ = -100.0 * control_brake_;
        } else {    // If in idle mode, don't use brakes either
            command_brake_ = 0.0;
            halt_mode_additional_brake_ = 0.0;
        }
    } else {    // If in throttle mode, use throttle and not brakes
        command_brake_ = 0.0;
        command_throttle_ = 100.0 * control_throttle_;
        halt_mode_additional_brake_ = 0.0;
    }
    last_command_update_ = clock;
}

void PIDBasedLongitudinalController::UpdateClock(
    PreciseSeconds clock) {
    dt_ = clock - clock_;
    clock_ = clock;
}

std::pair<Percentage, Percentage> 
PIDBasedLongitudinalController::GetLongitudinalCommand() {
    return std::make_pair(command_throttle_, command_brake_);
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
PIDBasedLongitudinalController::GetControllerStates(PreciseSeconds time) const {
    return std::make_tuple(
        time,
        throttle_mode_,
        brake_mode_,
        kp_throttle_,
        ki_throttle_,
        kii_throttle_,
        kd_throttle_,
        kp_brake_,
        ki_brake_,
        kii_brake_,
        kd_brake_,
        vehicle_velocity_,
        vehicle_velocity_filtered_,
        reference_velocity_,
        reference_velocity_filtered_,
        velocity_error_,
        previous_velocity_error_,
        velocity_error_integral_throttle_,
        velocity_error_double_integral_throttle_,
        velocity_error_integral_brake_,
        velocity_error_double_integral_brake_,
        velocity_error_integral_global_,
        velocity_error_derivative_,
        p_term_throttle_,
        i_term_throttle_,
        ii_term_throttle_,
        d_term_throttle_,
        p_term_brake_,
        i_term_brake_,
        ii_term_brake_,
        d_term_brake_,
        last_update_brake_,
        last_update_throttle_,
        last_command_update_,
        dt_,
        clock_,
        raw_control_throttle_,
        raw_control_brake_,
        control_throttle_,
        control_brake_,
        halt_mode_additional_brake_,
        command_throttle_,
        command_brake_
    );
}
