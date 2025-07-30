/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/

#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>
#include <fstream>
#include <any>
#include <utility>
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "test_setup.hpp"// NOLINT
#include "../Utils/Functions.hpp"// NOLINT
#include "../Utils/Classes.hpp"// NOLINT
#include "../Utils/DataHandling.hpp"// NOLINT
#include "../Utils/short_term_localization.hpp"// NOLINT
#include "../wrapper/control_api_wrapper.h"// NOLINT

// using namespace Eigen;
using json = nlohmann::json;

void TestEigen() {
    Eigen::Matrix2d mat1;
    mat1 << 1, 2, 3, 4;
    Eigen::Matrix2d mat2;
    mat2 << 2, 3, 1, 4;

    // Perform some operations.
    Eigen::Matrix2d sum = mat1 + mat2;
    Eigen::Matrix2d diff = mat1 - mat2;
    Eigen::Matrix2d prod = mat1 * mat2;

    // Print the results.
    std::cout << "Sum:\n" << sum << std::endl;
    std::cout << "Difference:\n" << diff << std::endl;
    std::cout << "Product:\n" << prod << std::endl;
}
void TestReadCSV() {
    std::filesystem::path data_path_relative = "../data";
    std::filesystem::path data_path_absolute =
        std::filesystem::absolute(data_path_relative);
    std::filesystem::path vehicle_states_file_path =
        data_path_absolute / "vehicle_states.csv";
    std::vector<std::pair<std::string, std::vector<double>>> vehicle_states =
        ReadCSV(vehicle_states_file_path);
    std::cout << "file: " + vehicle_states_file_path.string() +
                     " read succesfully"
              << std::endl;
}
void TestShortTermLocalization() {
    std::filesystem::path data_path_relative = "../data/2023-06-13-T11_48_22";
    std::filesystem::path data_path_absolute =
        std::filesystem::absolute(data_path_relative);
    std::filesystem::path speed_file_path = data_path_absolute / "speed.csv";
    std::filesystem::path imu_file_path = data_path_absolute / "imu.csv";
    std::vector<std::pair<std::string, std::vector<double>>> speed_data =
        ReadCSV(speed_file_path);
    std::vector<std::pair<std::string, std::vector<double>>> imu_data =
        ReadCSV(imu_file_path);
    std::vector<double> speed_update_time = speed_data[0].second;
    std::vector<double> speed_values = speed_data[1].second;
    std::vector<double> IMU_update_time = imu_data[0].second;
    std::vector<double> IMU_acc_x = imu_data[1].second;
    std::vector<double> IMU_acc_y = imu_data[2].second;
    std::vector<double> IMU_acc_z = imu_data[3].second;
    std::vector<double> IMU_acc_b_x = imu_data[4].second;
    std::vector<double> IMU_acc_b_y = imu_data[5].second;
    std::vector<double> IMU_acc_b_z = imu_data[6].second;
    std::vector<double> IMU_pitch = imu_data[7].second;
    std::vector<double> IMU_roll = imu_data[8].second;
    std::vector<double> IMU_yaw = imu_data[9].second;
    std::vector<double> IMU_gyro_x = imu_data[10].second;
    std::vector<double> IMU_gyro_y = imu_data[11].second;
    std::vector<double> IMU_gyro_z = imu_data[12].second;
    std::vector<double> IMU_gyro_b_x = imu_data[13].second;
    std::vector<double> IMU_gyro_b_y = imu_data[14].second;
    std::vector<double> IMU_gyro_b_z = imu_data[15].second;
    std::vector<double> IMU_mag_x = imu_data[16].second;
    std::vector<double> IMU_mag_y = imu_data[17].second;
    std::vector<double> IMU_mag_z = imu_data[18].second;
    // initialize
    double t0 = std::min(speed_update_time[0], IMU_update_time[0]);
    speed_update_time = AddScalarToVector(speed_update_time, -t0);
    IMU_update_time = AddScalarToVector(IMU_update_time, -t0);
    int speed_idx = 0;
    int IMU_idx = 0;
    double clock = 0;
    std::string next_event;
    if (speed_update_time[speed_idx] < IMU_update_time[IMU_idx]) {
        next_event = "speed";
    } else {
        next_event = "IMU";
    }
    bool stop_cond = (speed_idx + 1 >= speed_update_time.size()) ||
                     (IMU_idx + 1 >= IMU_update_time.size());
    std::vector<double> P_est_x;
    std::vector<double> P_est_y;
    P_est_x.push_back(0.0);
    P_est_y.push_back(0.0);
    json vehicle_config = json::parse(std::ifstream("../vehicle_config.json"));
    json control_config = json::parse(std::ifstream("../control_config.json"));
    ShortTermLocalization short_term_localization_obj =
        ShortTermLocalization(control_config["vehicle_heading_estimation_mode"],
                                control_config["vehicle_speed_estimation_mode"],
                                vehicle_config["WB"]);
    // short_term_localization_obj.ResetVehicleState(clock);
    std::vector<double> P_est_time;
    P_est_time.push_back(clock);
    while (!stop_cond) {
        if (next_event == "speed") {
            clock = speed_update_time[speed_idx];
            short_term_localization_obj.UpdateSpeed(speed_values[speed_idx] /
                                                    3.6);
            speed_idx++;
        } else {
            clock = IMU_update_time[IMU_idx];
            short_term_localization_obj.UpdateIMU(
                IMU_update_time[IMU_idx], IMU_acc_x[IMU_idx],
                IMU_acc_y[IMU_idx], IMU_acc_z[IMU_idx], IMU_acc_b_x[IMU_idx],
                IMU_acc_b_y[IMU_idx], IMU_acc_b_z[IMU_idx],
                Deg2Rad(IMU_pitch[IMU_idx]), Deg2Rad(IMU_roll[IMU_idx]),
                Deg2Rad(IMU_yaw[IMU_idx]), IMU_gyro_x[IMU_idx],
                IMU_gyro_y[IMU_idx], IMU_gyro_z[IMU_idx], IMU_gyro_b_x[IMU_idx],
                IMU_gyro_b_y[IMU_idx], IMU_gyro_b_z[IMU_idx],
                IMU_mag_x[IMU_idx], IMU_mag_y[IMU_idx], IMU_mag_z[IMU_idx]);
            IMU_idx++;
        }
        if (speed_update_time[speed_idx] < IMU_update_time[IMU_idx]) {
            next_event = "speed";
        } else {
            next_event = "IMU";
        }
        if (!std::isnan(short_term_localization_obj.State().psi_) &&
            !std::isnan(short_term_localization_obj.State().speed_)) {
            short_term_localization_obj.UpdatePosition(clock);
            P_est_x.push_back(short_term_localization_obj.State().pos_[0]);
            P_est_y.push_back(short_term_localization_obj.State().pos_[1]);
            P_est_time.push_back(clock);
        }
        stop_cond = (speed_idx + 1 >= speed_update_time.size()) ||
                    (IMU_idx + 1 >= IMU_update_time.size());
    }
    std::vector<std::vector<double>> values;
    std::vector<std::string> headers;
    headers.push_back("P_est_time");
    values.push_back(P_est_time);
    headers.push_back("P_est_x");
    values.push_back(P_est_x);
    headers.push_back("P_est_y");
    values.push_back(P_est_y);
    std::vector<std::pair<std::string, std::vector<double>>> data_to_file =
        PackVectorsToNameValuePairs(headers, values);
    std::filesystem::path output_file_path =
        "../data/localization_test_output_cpp.csv";
    output_file_path = std::filesystem::absolute(output_file_path);
    PrintToCSV(output_file_path, data_to_file);
}
void TestShortTermLocalization2() {
    std::filesystem::path data_path_relative = "../data/2023-06-13-T11_48_22";
    std::filesystem::path data_path_absolute =
        std::filesystem::absolute(data_path_relative);
    std::filesystem::path speed_file_path = data_path_absolute / "speed.csv";
    std::filesystem::path imu_file_path = data_path_absolute / "imu.csv";
    std::vector<std::pair<std::string, std::vector<double>>> speed_data =
        ReadCSV(speed_file_path);
    std::vector<std::pair<std::string, std::vector<double>>> imu_data =
        ReadCSV(imu_file_path);
    std::vector<double> speed_update_time = speed_data[0].second;
    std::vector<double> speed_values = speed_data[1].second;
    std::vector<double> IMU_update_time = imu_data[0].second;
    std::vector<double> IMU_acc_x = imu_data[1].second;
    std::vector<double> IMU_acc_y = imu_data[2].second;
    std::vector<double> IMU_acc_z = imu_data[3].second;
    std::vector<double> IMU_acc_b_x = imu_data[4].second;
    std::vector<double> IMU_acc_b_y = imu_data[5].second;
    std::vector<double> IMU_acc_b_z = imu_data[6].second;
    std::vector<double> IMU_pitch = imu_data[7].second;
    std::vector<double> IMU_roll = imu_data[8].second;
    std::vector<double> IMU_yaw = imu_data[9].second;
    std::vector<double> IMU_gyro_x = imu_data[10].second;
    std::vector<double> IMU_gyro_y = imu_data[11].second;
    std::vector<double> IMU_gyro_z = imu_data[12].second;
    std::vector<double> IMU_gyro_b_x = imu_data[13].second;
    std::vector<double> IMU_gyro_b_y = imu_data[14].second;
    std::vector<double> IMU_gyro_b_z = imu_data[15].second;
    std::vector<double> IMU_mag_x = imu_data[16].second;
    std::vector<double> IMU_mag_y = imu_data[17].second;
    std::vector<double> IMU_mag_z = imu_data[18].second;
    // initialize
    double t0 = std::min(speed_update_time[0], IMU_update_time[0]);
    speed_update_time = AddScalarToVector(speed_update_time, -t0);
    IMU_update_time = AddScalarToVector(IMU_update_time, -t0);
    int speed_idx = 0;
    int IMU_idx = 0;
    double clock = 0;
    std::string next_event;
    if (speed_update_time[speed_idx] < IMU_update_time[IMU_idx]) {
        next_event = "speed";
    } else {
        next_event = "IMU";
    }
    bool stop_cond = (speed_idx + 1 >= speed_update_time.size()) ||
                     (IMU_idx + 1 >= IMU_update_time.size());
    std::vector<double> P_est_x;
    std::vector<double> P_est_y;
    P_est_x.push_back(0.0);
    P_est_y.push_back(0.0);
    json vehicle_config = json::parse(std::ifstream("../vehicle_config.json"));
    json control_config = json::parse(std::ifstream("../control_config.json"));
    ShortTermLocalization short_term_localization_obj =
        ShortTermLocalization(control_config["vehicle_heading_estimation_mode"],
                                control_config["vehicle_speed_estimation_mode"],
                                vehicle_config["WB"]);
    // short_term_localization_obj.ResetVehicleState(clock);
    std::vector<double> P_est_time;
    P_est_time.push_back(clock);
    while (!stop_cond) {
        if (next_event == "speed") {
            clock = speed_update_time[speed_idx];
            short_term_localization_obj.UpdateSpeed(speed_values[speed_idx] /
                                                    3.6);
            speed_idx++;
        } else {
            clock = IMU_update_time[IMU_idx];
            ImuSample sample;
            sample.time_stamp = IMU_update_time[IMU_idx];
            sample.acc_.x = IMU_acc_x[IMU_idx];
            sample.acc_.y = IMU_acc_y[IMU_idx];
            sample.acc_.z = IMU_acc_z[IMU_idx];
            sample.acc_b_.x = IMU_acc_b_x[IMU_idx];
            sample.acc_b_.y = IMU_acc_b_y[IMU_idx];
            sample.acc_b_.z = IMU_acc_b_z[IMU_idx];
            sample.pitch_ = Deg2Rad(IMU_pitch[IMU_idx]);
            sample.roll_ = Deg2Rad(IMU_roll[IMU_idx]);
            sample.yaw_ = Deg2Rad(IMU_yaw[IMU_idx]);
            sample.gyro_.x = IMU_gyro_x[IMU_idx];
            sample.gyro_.y = IMU_gyro_y[IMU_idx];
            sample.gyro_.z = IMU_gyro_z[IMU_idx];
            sample.gyro_b_.x = IMU_gyro_b_x[IMU_idx];
            sample.gyro_b_.y = IMU_gyro_b_y[IMU_idx];
            sample.gyro_b_.z = IMU_gyro_b_z[IMU_idx];
            sample.mag_.x = IMU_mag_x[IMU_idx];
            sample.mag_.y = IMU_mag_y[IMU_idx];
            sample.mag_.z =
                IMU_mag_z[IMU_idx];  // called on every IMU measurement
            short_term_localization_obj.UpdateIMU(sample);
            IMU_idx++;
        }
        if (speed_update_time[speed_idx] < IMU_update_time[IMU_idx]) {
            next_event = "speed";
        } else {
            next_event = "IMU";
        }
        if (!std::isnan(short_term_localization_obj.State().psi_) &&
            !std::isnan(short_term_localization_obj.State().speed_)) {
            short_term_localization_obj.UpdatePosition(clock);
            P_est_x.push_back(short_term_localization_obj.State().pos_[0]);
            P_est_y.push_back(short_term_localization_obj.State().pos_[1]);
            P_est_time.push_back(clock);
        }
        stop_cond = (speed_idx + 1 >= speed_update_time.size()) ||
                    (IMU_idx + 1 >= IMU_update_time.size());
    }
    std::vector<std::vector<double>> values;
    std::vector<std::string> headers;
    headers.push_back("P_est_time");
    values.push_back(P_est_time);
    headers.push_back("P_est_x");
    values.push_back(P_est_x);
    headers.push_back("P_est_y");
    values.push_back(P_est_y);
    std::vector<std::pair<std::string, std::vector<double>>> data_to_file =
        PackVectorsToNameValuePairs(headers, values);
    std::filesystem::path output_file_path =
        "../data/localization_test_output_cpp.csv";
    output_file_path = std::filesystem::absolute(output_file_path);
    PrintToCSV(output_file_path, data_to_file);
}
void TestNlohman() {
    // create a JSON object
    json j;
    j["pi"] = 3.141;
    j["pi_ip"] = 3.1411413;

    // print the JSON object
    std::cout << j << std::endl;
    std::ifstream f("../vehicle_config.json");
    json data = json::parse(f);
    std::cout << data << std::endl;
    std::cout << "steering_ratio = " << data["steering_ratio"] << std::endl;
    double SR = data["steering_ratio"];
    std::cout << "steering_ratio X 4 = " << std::to_string(4.0 * SR)
              << std::endl;
    std::ifstream f2("../control_config.json");
    json data2 = json::parse(f2);
    std::cout << data2 << std::endl;
    json LQR_config = data2["LQR_config"];
    std::cout << "lateral_error_weight = " << LQR_config["lateral_error_weight"] << std::endl;
}
void TestPlotInputs() {
    system(
        "python ../Tests/python/plot_inputs.py --path "
        "../Tests/python/input.csv");
}
void TestRateLimiter() {
    RateLimiter f = RateLimiter(0.1, 0.1, 0.1);
    f.StepTest();
    f.SinTest();
}
void Test2ndOrderLPF() {
    SecondOrderLPF f = SecondOrderLPF(0.0, 1.0, 0.01, 0.0, 0.5, 1e-6);
    f.StepTest();
}
void TestLPF() {
    FirstOrderLPF f = FirstOrderLPF(0.0, 1.0, 0.01, 0.0, 0.5, 1e-6);
    f.StepTest();
    f.SinTest();
}
void TestLinInterp(){
    std::vector<double> x = {0, 1, 2, 3};
    std::vector<double> y = {0, 2, 4, 6};
    std::vector<double> xi = {0.0, 0.5, 1.5, 20.5};

    auto [yi, success] = LinearInterp(x, y, xi);
    if (!success) {
        std::cerr << "[warning] TestLinInterp: Interpolation failed.\n";
        return;
    }
    std::cout << "Interpolated values:\n";
    for (size_t i = 0; i < xi.size(); ++i)
            std::cout << "f(" << xi[i] << ") = " << yi[i] << "\n"; 
}
