/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#pragma once

#include <deque>
#include <iostream>

#include "DataHandling.hpp"
#include "Functions.hpp"
#include "units.hpp"

template <typename T_buffer>
struct Buffer {
    std::deque<T_buffer> values_;
    int maximal_size_;
    explicit Buffer(int max_size):
        maximal_size_(max_size) {}
    void Update(T_buffer new_value) {
        values_.push_back(new_value);
        if (values_.size() > maximal_size_) {
            values_.pop_front();
            }
        }
    T_buffer GetValue(int idx) const {
        if (idx >= this->Size()) {
            throw std::invalid_argument(
                "Buffer::GetValue:  idx > buffer.size");
            }
        return values_.at(idx);
        }
    T_buffer GetFirst() const {
       return values_.front();
        }
    T_buffer GetLast() const {
        if (this->Size() == 0) {
            throw std::invalid_argument(
                "Buffer::GetLast:  buffer.size is zero");
            }
        return values_.back();
        }
    // defined for buffers containing doubles (clock)
    std::tuple<int, T_buffer> ClosestValue(T_buffer target) const {
        std::vector<T_buffer> delta_vec;
        for (int i=0; i < this->Size(); i++) {
            delta_vec.push_back(
                abs(this->GetValue(i) - target));
            }
        int idx = std::min_element(delta_vec.begin(),
            delta_vec.end()) - delta_vec.begin();
        return std::make_tuple(idx, values_[idx]);
        }
    int Size() const {
        return values_.size();
        }
    void Clear() {
        values_.clear();
        }
    void Test() {
        Buffer bf = Buffer(5);
        bf.Update(static_cast<double>(0));
        bf.Update(static_cast<double>(1));
        bf.Update(static_cast<double>(2));
        bf.Update(static_cast<double>(3));
        bf.Update(static_cast<double>(4));
        int n = bf.values_.size();
        std::cout << "\n" << std::endl;
        for (int i = 0; i < n; i++) {
            double val = static_cast<double>(bf.values_.at(i));
            std::cout << "bf.values_[" << i << "] = " << val << std::endl;
        }
        bf.Update(static_cast<double>(5));
        n = bf.values_.size();
        std::cout << "\n" << std::endl;
        for (int i = 0; i < n; i++) {
            double val = static_cast<double>(bf.values_.at(i));
            std::cout << "bf.values_[" << i << "] = " << val << std::endl;
        }
        std::cout << "\n" << std::endl;
        std::cout << "first = " << bf.GetFirst() << std::endl;
        std::cout << "\n" << std::endl;
        std::cout << "last = " << bf.GetLast() << std::endl;
        std::cout << "\n" << std::endl;
        std::cout << " bf.values_[2] = " << bf.GetValue(2)<< std::endl;
        std::cout << "\n" << std::endl;
        std::cout << "Buffer size before clear: " << bf.values_.size() << std::endl;
        bf.Clear();
        std::cout << "Buffer size after clear: " << bf.values_.size() << std::endl;
    }
};
template <typename T_delay>
class Delay {
 private:
    Buffer<T_delay> stored_values_;
    Buffer<PreciseSeconds> clock_;
    PreciseSeconds max_time_delay_ = 60.0;
    T_delay default_value_;
    mutable std::mutex lock_;

 public:
    Delay(PreciseSeconds max_time_delay,
          PreciseSeconds nominal_dt,
          T_delay default_value):
            stored_values_(Buffer<T_delay>(max_time_delay / nominal_dt)),
            clock_(Buffer<PreciseSeconds>(max_time_delay / nominal_dt)),
            max_time_delay_(max_time_delay),
            default_value_(default_value) {}
    void Update(PreciseSeconds clock, T_delay value) {
        std::lock_guard<std::mutex> guard(lock_);
        clock_.Update(clock);
        stored_values_.Update(value);
    }
    T_delay GetDelayedValue(PreciseSeconds delayed_clock) const {
        std::lock_guard<std::mutex> guard(lock_);
        // std::cout << "Clock size: " << clock_.Size() << std::endl;
        if (clock_.Size() == 0) {
            // std::cout<< "returned default value" << std::endl;
            return default_value_;
        // } else if (clock_.GetFirst() > delayed_clock) {
        //     std::cout<< "returned default value" << std::endl;
        //     return default_value_;
        } else {
            // std::cout << "delayed_clock = " << std::to_string(delayed_clock) << std::endl;
            // std::cout << "first clock in buffer = " << std::to_string(clock_.GetFirst()) << std::endl;
            std::tuple<int, PreciseSeconds>
                temp = clock_.ClosestValue(delayed_clock);
            // std::cout << "idx = " << std::get<0>(temp) << std::endl;
            // std::cout << "closest clock = " << std::to_string(std::get<1>(temp)) << std::endl;
            int idx = std::get<0>(temp);
            return stored_values_.GetValue(idx);
        }
    }
    PreciseSeconds GetLatestTime() const {
        std::lock_guard<std::mutex> guard(lock_);
        // std::cout << "GetLatestTime" << std::endl;
        // std::cout << "clock size: " << clock_.Size() << std::endl;
        if (clock_.Size() == 0){
            // std::cout<< "returned default time" << std::endl;
            return 0.0;
        } else {
            return clock_.GetLast();
        }
    }
    int Size() const {
        std::lock_guard<std::mutex> guard(lock_);
        return clock_.Size();
    }
    void Clear() {
        std::lock_guard<std::mutex> guard(lock_);
        stored_values_.Clear();
        clock_.Clear();
    }
    void Test() {
        std::vector<PreciseSeconds> t = Arange(0, 10, 0.001);
        std::vector<double> signal;
        std::vector<double> delayed_signal;
        for (const PreciseSeconds &ti : t) {
            signal.push_back(
                sin(2 * M_PI * 0.2 * ti) + sin(2 * M_PI * 0.3 * ti + 0.3));
        }
        PreciseSeconds nominal_delay = 0.1;
        Delay delay_obj = Delay<double>(0.3, 0.001, static_cast<double>(0.0));
        for (int i=0; i < t.size(); ++i) {
            delay_obj.Update(t[i], signal[i]);
            delayed_signal.push_back(
                delay_obj.GetDelayedValue(t[i] - nominal_delay));
        }
        std::filesystem::path current_path = std::filesystem::current_path();
        std::filesystem::path temp_results_dir_path =
                    current_path / ".." / "data/temp_results";
        std::filesystem::path delay_test_results_dir =
                    temp_results_dir_path / "delay_test_results_dir";
        if (!std::filesystem::exists(delay_test_results_dir)) {
                    std::cout << "Directory does not exist." << std::endl;
                    (std::filesystem::create_directory(
                        delay_test_results_dir));
                    std::cout << temp_results_dir_path <<
                        " Created" << std::endl;
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
};
