/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <cstdio>
#include <cstdlib>
#include "Classes.hpp"// NOLINT
#include "Functions.hpp"// NOLINT
#include "DataHandling.hpp"// NOLINT

void Trajectory::Init(const std::vector<PreciseMeters>& x,
                      const std::vector<PreciseMeters>& y) {
    x_coor = x;
    y_coor = y;
}
int Trajectory::ProjectPoint(PreciseMeters x, PreciseMeters y,
                             PreciseRadians psi,
                             bool exclude_points_behind_vehicle)
                             const {
    int target_index;
    std::vector<PreciseMeters> dx;
    std::vector<PreciseMeters> dy;
    std::vector<PreciseMeters> dist;
    int num_traj_samples = x_coor.size();
    if (exclude_points_behind_vehicle) {
        // create an Eigen array  of size nX2
        Eigen::Matrix<double, Eigen::Dynamic, 2> trajectory_points_nav;
        trajectory_points_nav =
            Eigen::Matrix<double, Eigen::Dynamic, 2>::Zero(num_traj_samples, 2);
        for (int i = 0; i < num_traj_samples; i++) {
            trajectory_points_nav(i, 0) = x_coor[i];
            trajectory_points_nav(i, 1) = y_coor[i];
        }
        // calculate affine transformation from NAV to EGO
        Eigen::Matrix3d T_nav_2_ego = AffineTransformation2D(x, y, psi);
        Eigen::Matrix<double, Eigen::Dynamic, 2> trajectory_points_ego =
            ProjectPoints2D(T_nav_2_ego, trajectory_points_nav);
        std::vector<int> idx_vec;
        for (int i = 0; i < num_traj_samples; i++) {
            if (trajectory_points_ego(i, 0) > 0) {idx_vec.push_back(i);}
        }
        for (int j = 0; j < idx_vec.size(); j++) {
            dist.push_back(sqrt(pow(trajectory_points_ego(idx_vec[j], 0), 2) +
                            pow(trajectory_points_ego(idx_vec[j], 1), 2)));
        }
        int target_index_in_idx_vec = std::min_element(dist.begin(),
                                                    dist.end()) - dist.begin();
        if (target_index_in_idx_vec >= idx_vec.size()) {
            std::cout << "Trajectory::ProjectPoint:" << 
            "target_index_in_idx_vec >= idx_vec.size"<< "\n"
            << "target_index_in_idx_vec = " << target_index_in_idx_vec << "\n"
            << "idx_vec.size() = " << idx_vec.size() << "\n";
            return -1;
        }
        target_index = idx_vec[target_index_in_idx_vec];
    } else { 
        for (int i = 0; i < num_traj_samples; i++) {
            dx.push_back(x - x_coor[i]);
            dy.push_back(y - y_coor[i]);
            dist.push_back(sqrt(pow(dx[i], 2) + pow(dy[i], 2)));
        }
        target_index = std::min_element(dist.begin(), dist.end())
                         - dist.begin();
    }
    return target_index;
}
void Trajectory::Swap(Trajectory& other) {
    x_coor.swap(other.x_coor);
    y_coor.swap(other.y_coor);
    psi.swap(other.psi);
    s.swap(other.s);
}
Buffer_any::Buffer_any(int max_size)
:
    maximal_size_(max_size)
{}
void Buffer_any::Update(std::any new_value) {
    values_.push_back(new_value);
    if (values_.size() > maximal_size_) {
        values_.pop_front();
    }
}
std::any Buffer_any::GetValue(int idx) {
    if (idx > this->Size()) {
        throw std::invalid_argument("Buffer::GetValue:  idx > buffer.size");
    }
    return values_.at(idx);
}
std::any Buffer_any::GetFirst() {
    return values_.front();
}
std::any Buffer_any::GetLast() {
    if (this->Size() == 0) {
        throw std::invalid_argument("Buffer::GetLast:  buffer.size is zero");
    }
    return values_.back();
}
// defined for buffers containing doubles (clock)
std::tuple<int, double> Buffer_any::ClosestValue(double target) {
    std::vector<double> delta_vec;
    for (int i=0; i < this->Size(); i++) {
        delta_vec.push_back(
            abs(std::any_cast<double>(this->GetValue(i)) - target));
    }
    int idx = std::min_element(
        delta_vec.begin(), delta_vec.end()) - delta_vec.begin();
    return std::make_tuple(idx, std::any_cast<double>(values_[idx]));
}
int Buffer_any::Size() {
    return values_.size();
}
Delay_any::Delay_any(PreciseSeconds max_time_delay,
          PreciseSeconds nominal_dt,
          std::any default_value):
          stored_values_(Buffer_any(max_time_delay / nominal_dt)),
          clock_(Buffer_any(max_time_delay / nominal_dt)),
          max_time_delay_(max_time_delay),
          default_value_(default_value)
          {}
void Delay_any::Update(PreciseSeconds clock, std::any value) {
    clock_.Update(clock);
    stored_values_.Update(value);
}
std::any Delay_any::GetDelayedValue(PreciseSeconds delayed_clock) {
    if (clock_.Size() == 0) {
        return default_value_;
    } else if (
        std::any_cast<PreciseSeconds>(clock_.GetFirst()) > delayed_clock) {
        return default_value_;
    } else {
        std::tuple<int, double> temp = clock_.ClosestValue(delayed_clock);
        int idx = std::get<0>(temp);
        return stored_values_.GetValue(idx);
    }
}
PreciseSeconds Delay_any::GetLatestTime() {
    return std::any_cast<PreciseSeconds>(clock_.GetLast());
}
int Delay_any::Size() {
    return clock_.Size();
}
void Delay_any::Test() {
    std::vector<PreciseSeconds> t = Arange(0, 10, 0.001);
    std::vector<double> signal;
    std::vector<double> delayed_signal;
    for (const PreciseSeconds &ti : t) {
        signal.push_back(sin(2 * M_PI * 0.2 * ti) + sin(
            2 * M_PI * 0.3 * ti + 0.3));
    }
    PreciseSeconds nominal_delay = 0.1;
    Delay_any delay_obj = Delay_any(0.3, 0.001, static_cast<double>(0.0));

    for (int i = 0; i < t.size(); ++i) {
        delay_obj.Update(t[i], signal[i]);
        delayed_signal.push_back(
            std::any_cast<double>(delay_obj.GetDelayedValue(
                t[i] - nominal_delay)));
    }
    std::filesystem::path current_path = std::filesystem::current_path();
    std::filesystem::path temp_results_dir_path =
                current_path / ".." / "data/temp_results";
    std::filesystem::path delay_test_results_dir =
                temp_results_dir_path / "delay_test_results_dir";
    if (!std::filesystem::exists(delay_test_results_dir)) {
                std::cout << "Directory does not exist." << std::endl;
                (std::filesystem::create_directory(delay_test_results_dir));
                std::cout << temp_results_dir_path << " Created" << std::endl;
            }
    {
        std::vector<std::string> headers{"x", "y"};
        std::vector<std::vector<double>> data;
        data.push_back(t);
        data.push_back(signal);
        std::vector<std::pair<std::string, std::vector<double>>>
            dict_for_csv = PackVectorsToNameValuePairs(headers, data);
        PrintToCSV(delay_test_results_dir / "signal", dict_for_csv, true);
    }
    {
        std::vector<std::string> headers{"x", "y"};
        std::vector<std::vector<double>> data;
        data.push_back(t);
        data.push_back(delayed_signal);
        std::vector<std::pair<std::string, std::vector<double>>>
            dict_for_csv = PackVectorsToNameValuePairs(headers, data);
        PrintToCSV(
            delay_test_results_dir / "delayed_signal", dict_for_csv, true);
    }
    std::string system_cmd =
        "python ../Tests/python/plot_inputs.py --path " +
        (delay_test_results_dir / "signal").string() + " --path2 " +
            (delay_test_results_dir / "delayed_signal").string();
    std::cout << system_cmd << "\n";
    system(system_cmd.c_str());
}
SecondOrderLPF::SecondOrderLPF(
    PreciseSeconds clock, double freq_cutoff, double dumping_ratio,
    double initial_value, double initial_derivative, double max_dt,
    double eps) {
    double wc = freq_cutoff * 2 * M_PI;  // cut off frequency in rad/sec
    double zeta = dumping_ratio;  // dumping ratio
    wc_LPO1_ =
        2 * zeta * wc;  // cut off frequency in rad/sec of the 1st order filter
    K_ = (wc * wc) / wc_LPO1_;
    max_dt_ = max_dt;
    eps_ = eps;
    Reset(clock, initial_value, initial_derivative);
}

void SecondOrderLPF::Reset(PreciseSeconds clock,
                                               double initial_value,
                                               double initial_derivative) {
    last_update_time_ = clock;
    integrator_1_ = initial_derivative;
    integrator_2_ = initial_value;
}

double  SecondOrderLPF::Update(PreciseSeconds clock, double uk) {
    PreciseSeconds dt = clock - last_update_time_;
    if (dt == 0.0) {
        std::cout << "Rate limiter duplicate update" << std::endl;
        return integrator_2_;
    }
    if (dt > max_dt_ || dt < 0.0) {
        Reset(clock, integrator_2_, integrator_1_);
        return integrator_2_;
    }

    last_update_time_ = clock;
    double input_to_LPO1 = K_ * (uk - integrator_2_);

    // 1st order LPF 
    double input_to_integrator_LPO1 = wc_LPO1_ * (input_to_LPO1 - integrator_1_);
    integrator_1_ += input_to_integrator_LPO1 * dt;
    // LPFO1 output is fed to 2nd integrator
    integrator_2_ += integrator_1_ * dt;
    return integrator_2_;
}
void SecondOrderLPF::StepTest() {
    std::vector<PreciseSeconds> t1 = Arange(
        (PreciseSeconds)0, (PreciseSeconds)10, (PreciseSeconds)0.001);
    std::vector<PreciseSeconds> t2 = Arange(
        (PreciseSeconds)50, (PreciseSeconds)60, (PreciseSeconds)0.001);
    std::vector<PreciseSeconds> t;
    t.insert(t.end(), t1.begin(), t1.end());
    t.insert(t.end(), t2.begin(), t2.end());
    double step_time;
    double step_value;
    SecondOrderLPF f = SecondOrderLPF(t[0], 0.1, 1.0, 0.0, 0.0);
    std::vector<double> u;
    std::vector<double> y;
    for (const PreciseSeconds &ti : t) {
        if (ti < 1.0) {
            step_value = 0.0;
            } else if (ti < 10) {
            step_value = 1.0;
            } else if (ti < 51) {
            step_value = 0.3;
            } else {
            step_value = -2.0;
            }
        u.push_back(step_value);
        y.push_back(f.Update(ti, u.back()));
    }
    // PrintCurrentPath();
    std::filesystem::path file_path = std::filesystem::current_path() /
        ".." / "data" / "temp_results"/ "step_test.csv";
    // std::cout << file_path.string() << std::endl;
    // ListFilesInDirectory(file_path.parent_path());
    std::vector<std::vector<double>> values;
    std::vector<std::string> headers;
    headers.push_back("t");
    values.push_back(t);
    headers.push_back("u");
    values.push_back(u);
    headers.push_back("y");
    values.push_back(y);
    std::vector<std::pair<std::string, std::vector<double>>> data_for_vis =
        PackVectorsToNameValuePairs(headers, values);
    PrintToCSV(file_path, data_for_vis);
    std::string system_cmd =
        "python ../Tests/python/plot_inputs.py --path "
        + file_path.string()
        + " --x t t --y u y";
    system(system_cmd.c_str());
}

SecondOrderLPF_WithRateAndAccLimit::SecondOrderLPF_WithRateAndAccLimit(
    PreciseSeconds clock, double freq_cutoff, double restraint_coefficient,
    double max_rate, double max_negative_rate, double max_acc,
    double initial_value, double initial_derivative, double max_dt,
    double eps) {
    double wc = freq_cutoff * 2 * M_PI;  // cut off frequency in rad/sec
    double zeta = restraint_coefficient;
    wc_LPO1_ =
        2 * zeta * wc;  // cut off frequency in rad/sec of the 1st order filter
    K_ = (wc * wc) / wc_LPO1_;
    std::cout << "freq_cutoff: " << freq_cutoff << ", wc: " << wc << ", zeta: " << zeta << ", wc_LPO1_:" << wc_LPO1_ << ", K_: " << K_ << "\n";
    max_dt_ = max_dt;
    eps_ = eps;
    max_rate_ = max_rate;
    max_negative_rate_ = max_negative_rate;
    max_acc_ = max_acc;
    Reset(clock, initial_value, initial_derivative);
}

void SecondOrderLPF_WithRateAndAccLimit::Reset(PreciseSeconds clock,
                                               double initial_value,
                                               double initial_derivative) {
    last_update_time_ = clock;
    integrator_1_ = initial_derivative;
    integrator_2_ = initial_value;
}

std::pair<double /* velocity */, double /* acceleration */>
SecondOrderLPF_WithRateAndAccLimit::Update(PreciseSeconds clock, double uk,
                                           double reference_value,
                                           double reference_derivative) {
    if (clock - last_update_time_ > max_dt_) {
        Reset(clock, reference_value, reference_derivative);
        return {integrator_2_, integrator_1_};
    }

    PreciseSeconds dt = clock - last_update_time_;
    last_update_time_ = clock;
    double input_to_LPO1 = K_ * (uk - integrator_2_);

    // if 1st integrator is out of range, bringing it to range with higher
    // priority
    if (integrator_1_ < -max_negative_rate_ - max_acc_ * dt) {
        integrator_1_ += max_acc_ * dt;
    } else if (integrator_1_ > max_rate_ + max_acc_ * dt) {
        integrator_1_ -= max_acc_ * dt;
    } else {
        // 1st order LPF with limited integrator
        double input_to_integrator_LPO1 =
            wc_LPO1_ * (input_to_LPO1 - integrator_1_);
        double input_to_integrator_LPO1_limited =
            std::clamp(input_to_integrator_LPO1, -max_acc_, max_acc_);

        integrator_1_ += input_to_integrator_LPO1_limited * dt;
        integrator_1_ =
            std::clamp(integrator_1_, -max_negative_rate_, max_rate_);
    }

    // LPFO1 output is fed to 2nd integrator
    integrator_2_ += integrator_1_ * dt;
    return {integrator_2_, integrator_1_};
}
