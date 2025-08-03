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

BufferAny::BufferAny(int max_size)
:
    maximal_size_(max_size)
{}
void BufferAny::Update(std::any new_value) {
    values_.push_back(new_value);
    if (values_.size() > maximal_size_) {
        values_.pop_front();
    }
}
std::any BufferAny::GetValue(int idx) {
    if (idx > this->Size()) {
        throw std::invalid_argument("Buffer::GetValue:  idx > buffer.size");
    }
    return values_.at(idx);
}
std::any BufferAny::GetFirst() {
    return values_.front();
}
std::any BufferAny::GetLast() {
    if (this->Size() == 0) {
        throw std::invalid_argument("Buffer::GetLast:  buffer.size is zero");
    }
    return values_.back();
}
// defined for buffers containing doubles (clock)
std::tuple<int, double> BufferAny::ClosestValue(double target) {
    std::vector<double> delta_vec;
    for (int i=0; i < this->Size(); i++) {
        delta_vec.push_back(
            abs(std::any_cast<double>(this->GetValue(i)) - target));
    }
    int idx = std::min_element(
        delta_vec.begin(), delta_vec.end()) - delta_vec.begin();
    return std::make_tuple(idx, std::any_cast<double>(values_[idx]));
}
int BufferAny::Size() {
    return values_.size();
}
DelayAny::DelayAny(PreciseSeconds max_time_delay,
          PreciseSeconds nominal_dt,
          std::any default_value):
          stored_values_(BufferAny(max_time_delay / nominal_dt)),
          clock_(BufferAny(max_time_delay / nominal_dt)),
          max_time_delay_(max_time_delay),
          default_value_(default_value)
          {}
void DelayAny::Update(PreciseSeconds clock, std::any value) {
    clock_.Update(clock);
    stored_values_.Update(value);
}
std::any DelayAny::GetDelayedValue(PreciseSeconds delayed_clock) {
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
PreciseSeconds DelayAny::GetLatestTime() {
    return std::any_cast<PreciseSeconds>(clock_.GetLast());
}
int DelayAny::Size() {
    return clock_.Size();
}
void DelayAny::Test() {
    std::vector<PreciseSeconds> t = Arange(0, 10, 0.001);
    std::vector<double> signal;
    std::vector<double> delayed_signal;
    for (const PreciseSeconds &ti : t) {
        signal.push_back(sin(2 * M_PI * 0.2 * ti) + sin(
            2 * M_PI * 0.3 * ti + 0.3));
    }
    PreciseSeconds nominal_delay = 0.1;
    DelayAny delay_obj = DelayAny(0.3, 0.001, static_cast<double>(0.0));

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

