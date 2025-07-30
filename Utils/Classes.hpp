/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#pragma once
#include <iostream>
#include <vector>
#include <deque>
#include <any>
#include <filesystem>
#include <tuple>
#include <utility>
#include <string>
#include <fstream>
#include <mutex>
#include <algorithm>
#include "../Utils/Functions.hpp"
#include "../Utils/units.hpp"
#include "../Utils/DataHandling.hpp"

struct Buffer_any {
    std::deque<std::any> values_;
    int maximal_size_;
    explicit Buffer_any(int max_size);
    void Update(std::any new_value);
    std::any GetValue(int idx);
    std::any GetFirst();
    std::any GetLast();
    // defined for buffers containing doubles (clock)
    std::tuple<int, double> ClosestValue(double target);
    int Size();
};
class Delay_any {
 private:
    Buffer_any stored_values_;
    Buffer_any clock_;
    PreciseSeconds max_time_delay_;
    std::any default_value_;
 public:
    Delay_any(PreciseSeconds max_time_delay,
          PreciseSeconds nominal_dt,
          std::any default_value);
    void Update(PreciseSeconds clock, std::any value);
    std::any GetDelayedValue(PreciseSeconds delayed_clock);
    PreciseSeconds GetLatestTime();
    int Size();
    void Test();
};

class RateLimiter {
 private:
    double maximal_rate_;
    double output_;
    double eps_;
    PreciseSeconds max_dt_;
    PreciseSeconds last_update_time_;

 public:
    RateLimiter(double maximal_rate, PreciseSeconds clock,
                 double initial_value = 0.0, PreciseSeconds max_dt = 2.0,
                 double eps = 1e-6):
        maximal_rate_(maximal_rate),
        output_(initial_value),
        eps_(eps),
        max_dt_(max_dt),
        last_update_time_(clock) {}
    void Reset(PreciseSeconds clock, double initial_value) {
        output_ = initial_value;
        last_update_time_ = clock;
    }
    double Update(PreciseSeconds clock, double input) {
        PreciseSeconds dt = clock - last_update_time_;
        if (dt > max_dt_ || dt < 0.0) {
            Reset(clock, output_);
            return output_;
        }
        last_update_time_ = clock;
        double dy = input - output_;
        // avoid devision by zero
        if (std::abs(dy) < eps_) {
            return output_;
            }
        double sign = dy/abs(dy);
        if (abs(dy) / dt < maximal_rate_) {
            output_ += dy;
            } else {
            output_ += sign * maximal_rate_ * dt;
            }
        return output_;
    }
    double Update(PreciseSeconds clock, double input, double reference_value) {
        PreciseSeconds dt = clock - last_update_time_;
        if (dt > max_dt_ || dt < 0.0) {
            Reset(clock, reference_value);
            return output_;
        }
        last_update_time_ = clock;
        double dy = input - output_;
        // avoid devision by zero
        if (std::abs(dy) < eps_) {
            return output_;
            }
        double sign = dy/abs(dy);
        if (abs(dy) / dt < maximal_rate_) {
            output_ += dy;
            } else {
            output_ += sign * maximal_rate_ * dt;
            }
        return output_;
    }
    void StepTest() {
        std::vector<PreciseSeconds> t1 = Arange(
            (PreciseSeconds)0, (PreciseSeconds)10, (PreciseSeconds)0.001);
        std::vector<PreciseSeconds> t2 = Arange(
            (PreciseSeconds)50, (PreciseSeconds)60, (PreciseSeconds)0.001);
        std::vector<PreciseSeconds> t;
        t.insert(t.end(), t1.begin(), t1.end());
        t.insert(t.end(), t2.begin(), t2.end());
        double step_time;
        double step_value;
        RateLimiter f = RateLimiter(0.8, t[0], 0.0, 0.05);
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
    void SinTest() {
        std::vector<PreciseSeconds> t = Arange(
            (PreciseSeconds)0, (PreciseSeconds)10, (PreciseSeconds)0.001);
        double freq = 0.2 * 2 * M_PI;
        double amplitude = 1;
        RateLimiter f = RateLimiter(1.0, t[0], 0.0, 0.05);
        std::vector<double> u;
        std::vector<double> y;
        for (const PreciseSeconds &ti : t) {
            u.push_back(amplitude * sin(freq * ti));
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
};
class FirstOrderLPF {
 private:
    PreciseSeconds dt_;
    double wc_;
    double yk_minus_1_;
    double yk_;
    double uk_minus_1_;
    double alfa_;
    double beta_;
    double output_;
    double eps_;
    PreciseSeconds max_dt_;
    PreciseSeconds last_update_time_;

 public:
    FirstOrderLPF(PreciseSeconds clock, double freq_cutoff, 
                  double freq_sampling, double initial_value = 0.0, 
                  PreciseSeconds max_dt = 0.05, double eps = 1e-6):
        /* 
        implement Tustin integration lowpass filter
        https://github.com/botprof/first-order-low-pass-filter/blob/main/first-order-lpf.ipynb
        freq is inserted in Hz and converted to rad/sec
        */
        wc_(2 * M_PI * freq_cutoff),//cut off frequency in rad/sec
        dt_(1/freq_sampling),
        yk_minus_1_(initial_value),
        yk_(initial_value),
        uk_minus_1_(0.0),
        alfa_((2 - wc_ * dt_ )/ (2 + wc_ * dt_)),
        beta_(wc_ * dt_ / (2 + wc_ * dt_)),
        max_dt_(max_dt),
        last_update_time_(clock),
        eps_(eps){}
    void Reset(PreciseSeconds clock, double initial_value) {
        output_ = initial_value;
        last_update_time_ = clock;
    }
    void InitializeClock(PreciseSeconds clock) {
        last_update_time_ = clock;
    }
    double Update(PreciseSeconds clock, double input) {
        PreciseSeconds dt = clock - last_update_time_;
        if (dt > max_dt_ || dt < 0.0) {
            InitializeClock(clock);
            return yk_;
        }
        last_update_time_ = clock;
        alfa_ = (2 - wc_ * dt_) / (2 + wc_ * dt_);
        beta_ = wc_ * dt_ / (2 + wc_ * dt_);
        yk_ = alfa_ * yk_minus_1_ + beta_ * (input + uk_minus_1_);
        yk_minus_1_ = yk_;
        uk_minus_1_ = input;
        return yk_;
    }
    void StepTest() {
        std::vector<PreciseSeconds> t1 = Arange(
            (PreciseSeconds)0, (PreciseSeconds)10, (PreciseSeconds)0.001);
        std::vector<PreciseSeconds> t2 = Arange(
            (PreciseSeconds)50, (PreciseSeconds)60, (PreciseSeconds)0.001);
        std::vector<PreciseSeconds> t;
        t.insert(t.end(), t1.begin(), t1.end());
        t.insert(t.end(), t2.begin(), t2.end());
        double step_time;
        double step_value;
        FirstOrderLPF f = FirstOrderLPF(t[0], 0.1, 100, 0.0, 0.05, 1e-6);
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
    void SinTest() {
        std::vector<PreciseSeconds> t = Arange(
            (PreciseSeconds)0, (PreciseSeconds)10, (PreciseSeconds)0.001);
        double freq = 2 * 2 * M_PI;
        double amplitude = 1;
        FirstOrderLPF f = FirstOrderLPF(t[0], 0.1, 100, 0.0, 0.05, 1e-6);
        std::vector<double> u;
        std::vector<double> y;
        for (const PreciseSeconds &ti : t) {
            u.push_back(amplitude * sin(freq * ti));
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
};
class SecondOrderLPF {
    public:
       SecondOrderLPF() {}
       SecondOrderLPF(PreciseSeconds clock, double freq_cutoff,
                                          double dumping_ratio,
                                          double initial_value,
                                          double initial_derivative,
                                          double max_dt = 0.05, double eps = 1e-6);
       void Reset(PreciseSeconds clock, double initial_value,
                  double initial_derivative);
       double Update(PreciseSeconds clock, double uk);
       void StepTest();
   
    private:
       double wc_LPO1_;  // cut off frequency in rad/sec of the 1st order filter
       double K_;
       double max_dt_;
       PreciseSeconds last_update_time_;
       double eps_;
       double integrator_1_;
       double integrator_2_;
       void InitializeClock(PreciseSeconds clock) { last_update_time_ = clock; }
};
class ContinuousAngleOnline{
    private:
        int counter_ = 0;
        PreciseRadians u_old_ = 0;
        bool initialized_ = false;
    public:
        ContinuousAngleOnline() {}
        PreciseRadians Update(PreciseRadians u) {
            if (!initialized_) {
                u_old_ = u;
                initialized_ = true;
                return u;
            }
            if (u - u_old_ > M_PI) {
                counter_ -= 1;
            } else if (u - u_old_ < -M_PI) {
                counter_ += 1;
            }
            u_old_ = u;
            return u + counter_ * 2 * M_PI;
            
        }
};

