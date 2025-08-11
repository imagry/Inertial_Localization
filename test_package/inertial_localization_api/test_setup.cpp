/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/

#include "test_setup.h"                                              // NOLINT
#include "inertial_localization_api/utils/classes.h"                 // NOLINT
#include "inertial_localization_api/utils/data_handling.h"           // NOLINT
#include "inertial_localization_api/utils/functions.h"               // NOLINT
#include "inertial_localization_api/utils/sensor_types.h"            // NOLINT
#include "inertial_localization_api/utils/short_term_localization.h" // NOLINT
#include "inertial_localization_api/wrapper/localization_api_wrapper.h" // NOLINT
#include <Eigen/Dense>
#include <algorithm>
#include <any>
#include <chrono>
#include <fstream>
#include <iostream>
#include <json/json.h>
#include <string>
#include <utility>
#include <vector>

// using namespace Eigen;
using inertial_localization_api::ImuSample;
using inertial_localization_api::ShortTermLocalization;
Json::Value ParseJsonFile(const std::string &filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Error opening file: " + filename);
  }

  Json::Value root;
  Json::Reader reader;
  if (!reader.parse(file, root)) {
    throw std::runtime_error("Error parsing file: " + filename);
  }

  return root;
}

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
  Json::Value vehicle_config = ParseJsonFile("../vehicle_config.json");
  Json::Value localization_config =
      ParseJsonFile("../localization_config.json");
  ShortTermLocalization short_term_localization_obj = ShortTermLocalization(
      localization_config["vehicle_heading_estimation_mode"].asString(),
      localization_config["vehicle_speed_estimation_mode"].asString(),
      vehicle_config["WB"].asDouble());
  // short_term_localization_obj.ResetVehicleState(clock);
  std::vector<double> P_est_time;
  P_est_time.push_back(clock);
  while (!stop_cond) {
    if (next_event == "speed") {
      clock = speed_update_time[speed_idx];
      short_term_localization_obj.UpdateSpeed(speed_values[speed_idx] / 3.6);
      speed_idx++;
    } else {
      clock = IMU_update_time[IMU_idx];
      short_term_localization_obj.UpdateImu(
          {IMU_update_time[IMU_idx], IMU_acc_x[IMU_idx], IMU_acc_y[IMU_idx],
           IMU_acc_z[IMU_idx], IMU_acc_b_x[IMU_idx], IMU_acc_b_y[IMU_idx],
           IMU_acc_b_z[IMU_idx], Deg2Rad(IMU_pitch[IMU_idx]),
           Deg2Rad(IMU_roll[IMU_idx]), Deg2Rad(IMU_yaw[IMU_idx]),
           IMU_gyro_x[IMU_idx], IMU_gyro_y[IMU_idx], IMU_gyro_z[IMU_idx],
           IMU_gyro_b_x[IMU_idx], IMU_gyro_b_y[IMU_idx], IMU_gyro_b_z[IMU_idx],
           IMU_mag_x[IMU_idx], IMU_mag_y[IMU_idx], IMU_mag_z[IMU_idx]});
      IMU_idx++;
    }
    if (speed_update_time[speed_idx] < IMU_update_time[IMU_idx]) {
      next_event = "speed";
    } else {
      next_event = "IMU";
    }
    if (!std::isnan(short_term_localization_obj.State().psi) &&
        !std::isnan(short_term_localization_obj.State().speed)) {
      short_term_localization_obj.UpdateFrontAxlePosition(clock);
      P_est_x.push_back(short_term_localization_obj.State().pos[0]);
      P_est_y.push_back(short_term_localization_obj.State().pos[1]);
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
  Json::Value vehicle_config = ParseJsonFile("../vehicle_config.json");
  Json::Value localization_config =
      ParseJsonFile("../localization_config.json");
  ShortTermLocalization short_term_localization_obj = ShortTermLocalization(
      localization_config["vehicle_heading_estimation_mode"].asString(),
      localization_config["vehicle_speed_estimation_mode"].asString(),
      vehicle_config["WB"].asDouble());
  // short_term_localization_obj.ResetVehicleState(clock);
  std::vector<double> P_est_time;
  P_est_time.push_back(clock);
  while (!stop_cond) {
    if (next_event == "speed") {
      clock = speed_update_time[speed_idx];
      short_term_localization_obj.UpdateSpeed(speed_values[speed_idx] / 3.6);
      speed_idx++;
    } else {
      clock = IMU_update_time[IMU_idx];
      ImuSample sample;
      sample.time_stamp = IMU_update_time[IMU_idx];
      sample.acc.x = IMU_acc_x[IMU_idx];
      sample.acc.y = IMU_acc_y[IMU_idx];
      sample.acc.z = IMU_acc_z[IMU_idx];
      sample.acc_b.x = IMU_acc_b_x[IMU_idx];
      sample.acc_b.y = IMU_acc_b_y[IMU_idx];
      sample.acc_b.z = IMU_acc_b_z[IMU_idx];
      sample.pitch = Deg2Rad(IMU_pitch[IMU_idx]);
      sample.roll = Deg2Rad(IMU_roll[IMU_idx]);
      sample.yaw = Deg2Rad(IMU_yaw[IMU_idx]);
      sample.gyro.x = IMU_gyro_x[IMU_idx];
      sample.gyro.y = IMU_gyro_y[IMU_idx];
      sample.gyro.z = IMU_gyro_z[IMU_idx];
      sample.gyro_b.x = IMU_gyro_b_x[IMU_idx];
      sample.gyro_b.y = IMU_gyro_b_y[IMU_idx];
      sample.gyro_b.z = IMU_gyro_b_z[IMU_idx];
      sample.mag.x = IMU_mag_x[IMU_idx];
      sample.mag.y = IMU_mag_y[IMU_idx];
      sample.mag.z = IMU_mag_z[IMU_idx]; // called on every IMU measurement
      short_term_localization_obj.UpdateImu(sample);
      IMU_idx++;
    }
    if (speed_update_time[speed_idx] < IMU_update_time[IMU_idx]) {
      next_event = "speed";
    } else {
      next_event = "IMU";
    }
    if (!std::isnan(short_term_localization_obj.State().psi) &&
        !std::isnan(short_term_localization_obj.State().speed)) {
      short_term_localization_obj.UpdateFrontAxlePosition(clock);
      P_est_x.push_back(short_term_localization_obj.State().pos[0]);
      P_est_y.push_back(short_term_localization_obj.State().pos[1]);
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
void TestPlotInputs() {
  system("python ../tests/python/plot_inputs.py --path "
         "../tests/python/input.csv");
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
