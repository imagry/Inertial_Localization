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
// B10: Removed as part of control code removal
// #include "../ControlAPI.hpp"// NOLINT
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
void TestEverything() {
    std::string msg = "------------------------------- ";
    PrintToTerminal(msg);
    msg = "Program Start";
    PrintToTerminal(msg);
    msg = "-------------------------------------";
    PrintToTerminal(msg);
    msg = "Tests/test_setup.hello_world() says: ";
    PrintToTerminal(msg);
    msg = "-------------------------------------";
    PrintToTerminal(msg);
    msg = "Tests/test_setup.test_Utils() says: ";
    PrintToTerminal(msg);
    msg = "-------------------------------------";
    PrintToTerminal(msg);
    msg = "Tests/test_setup.TestEigen() says: ";
    PrintToTerminal(msg);
    TestEigen();
    msg = "-------------------------------------";
    PrintToTerminal(msg);
    msg = "great succes !!!";
    PrintToTerminal(msg);
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
void TestSplines3() {
    std::filesystem::path data_path_relative = "../data";
    std::filesystem::path data_path_absolute =
        std::filesystem::absolute(data_path_relative);
    std::filesystem::path input_points_path =
        data_path_absolute / "motion_planning_single_path.csv";
    std::filesystem::path spline_python_points_path =
        data_path_absolute / "splines_python.csv";
    std::filesystem::path spline_cpp_points_path =
        data_path_absolute / "splines_cpp.csv";
    std::string system_cmd =
        "python ../Tests/python/create_splines.py --path " +
        input_points_path.string() + " --plot_res " + " --out_path " +
        spline_python_points_path.string();
    system(system_cmd.c_str());
    // ListFilesInDirectory(data_path_relative);
    std::vector<std::pair<std::string, std::vector<double>>> input_points =
        ReadCSV(input_points_path);
    std::vector<double> input_x = input_points[0].second;
    std::vector<double> input_y = input_points[1].second;
    
    auto [s_interp, x_interp, y_interp] = SplineInterpulationEigen(
        input_x, input_y, 0.1);
    std::vector<std::string> headers{"x", "y"};
    std::vector<std::vector<double>> vecs_to_save;
    vecs_to_save.push_back(x_interp);
    vecs_to_save.push_back(y_interp);
    std::vector<std::pair<std::string, std::vector<double>>> data_to_save =
        PackVectorsToNameValuePairs(headers, vecs_to_save);
    PrintToCSV(spline_cpp_points_path, data_to_save);
    std::cout << "traverse_spline done" << std::endl;
    system_cmd = "python ../Tests/python/plot_inputs.py --path " +
                 spline_python_points_path.string() + " --path2 " +
                 spline_cpp_points_path.string();
    system(system_cmd.c_str());
}
void TestAffineTransformation() {
    double CS2_origin_x = 2;
    double CS2_origin_y = 1;
    double psi = M_PI / 3;
    Eigen::Matrix3d T = AffineTransformation2D(CS2_origin_x, CS2_origin_y, psi);
    std::cout << "T = \n" << T << std::endl;
    Eigen::Matrix3d T_inv = InvAffineTransformation2D(T);
    std::cout << "T_inv = \n" << T_inv << std::endl;
}
void TestProject2dPoints() {
    double CS2_origin_x = 2;
    double CS2_origin_y = 1;
    double psi = M_PI / 2;
    Eigen::Matrix<double, 2, 2> p;
    p.row(0) << 3.0, 4.0;
    p.row(1) << 5.0, 6.0;
    std::cout << "p = \n" << p << std::endl;
    Eigen::Matrix3d T = AffineTransformation2D(CS2_origin_x, CS2_origin_y, psi);
    Eigen::Matrix<double, Eigen::Dynamic, 2> p_in_CS2 = ProjectPoints2D(T, p);
    std::cout << "p_in_CS2 = \n" << p_in_CS2 << std::endl;
}
void TestTimeTypes() {
    const std::chrono::time_point<std::chrono::system_clock> now =
        std::chrono::system_clock::now();
    const std::time_t t0 = std::chrono::system_clock::to_time_t(now);
    std::cout << "t0 = " << std::put_time(std::localtime(&t0), "%F %T.\n")
              << std::flush;
}
// Removed as part of B10 task (Control code removal)
/*
void TestControlAPI_init() {
    ControlAPI control_api = ControlAPI(std::string("../vehicle_config.json"),
                                        std::string("../control_config.json"));
    std::cout << "control_api initialized" << std::endl;
}
void TestControlAPI_init2(){
    std::shared_ptr<ControlAPI> control_api = aidriver::control_api::GetControlAPIInstance(std::map<std::string, std::string>(), 
                                                                                "../vehicle_config.json",
                                                                                "../control_config.json");
    std::cout << "control_api initialized" << std::endl;                               
}
void TestLQR_FromControlAPI() {
    std::shared_ptr<ControlAPI> control_api = aidriver::control_api::GetControlAPIInstance(std::map<std::string, std::string>(), 
                                                                                "../vehicle_config.json",
                                                                                "../control_config.json");
    
    auto localization_handler_obj = aidriver::control_api::GetAHRSLocHandlerInstance(std::map<std::string, std::string>(), std::string("../vehicle_config.json"),
                                                                                        std::string("../control_config.json"));
    PreciseMeters x_v = 0.0;
    PreciseMeters y_v = 1.0;
    PreciseRadians psi_v = 0.0;
    PreciseMps vel = 10 / 3.6;
    PreciseSeconds sys_clock = 0.0;
    PreciseRadians steering_wheel_angle = 0.0;
    localization_handler_obj->ResetVehicleState(sys_clock, {x_v, y_v, psi_v, vel});
    localization_handler_obj->UpdateSteeringWheel(steering_wheel_angle,sys_clock);
    std::vector<PreciseMeters> traj_x{0.0, 1.0, 2.0, 3.0, 4.0};
    std::vector<PreciseMeters> traj_y{0.0, 0.0, 0.0, 0.0, 0.0};
    control_api->MotionPlanningUpdate(traj_x, traj_y, sys_clock, string("NAV"), sys_clock);
    auto start = std::chrono::high_resolution_clock::now();
    control_api->CalculateSteeringCommand(sys_clock);
    auto end = std::chrono::high_resolution_clock::now();
    // Print steering command
    std::cout << "delta:\n" << control_api->GetDelta() << std::endl;
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Elapsed time: " << elapsed.count() << " seconds" << std::endl;
}
*/
void TestBuffer() {
    Buffer_any bf = Buffer_any(5);
    bf.Update(static_cast<double>(0));
    bf.Update(static_cast<double>(1));
    bf.Update(static_cast<double>(2));
    bf.Update(static_cast<double>(3));
    bf.Update(static_cast<double>(4));
    int n = bf.values_.size();
    std::cout << "\n" << std::endl;
    for (int i = 0; i < n; i++) {
        double val = std::any_cast<double>(bf.values_.at(i));
        std::cout << "bf.values_[" << i << "] = " << val << std::endl;
    }
    bf.Update(static_cast<double>(5));
    n = bf.values_.size();
    std::cout << "\n" << std::endl;
    for (int i = 0; i < n; i++) {
        double val = std::any_cast<double>(bf.values_.at(i));
        std::cout << "bf.values_[" << i << "] = " << val << std::endl;
    }
    std::cout << "\n" << std::endl;
    std::cout << "first = " << std::any_cast<double>(
        bf.GetFirst()) << std::endl;
    std::cout << "\n" << std::endl;
    std::cout << "last = " << std::any_cast<double>(bf.GetLast()) << std::endl;
    std::cout << "\n" << std::endl;
    std::cout << " bf.values_[2] = " << std::any_cast<double>(
        bf.GetValue(2)) << std::endl;
    std::cout << "\n" << std::endl;
}
void TestBufferTemplate() {
    Buffer buf = Buffer<int>(10);
    buf.Test();
}
void TestDelay() {
    // initialize a Delay object with whatever values
    Delay_any delay_obj = Delay_any(1, 1, 1);
    delay_obj.Test();
}
void TestDelayTemplate() {
    // initialize a Delay object with whatever values
    Delay delay_obj = Delay<double>(1, 1, 1);
    delay_obj.Test();
}
void TestTemplateClass() {
    TemplateTestClass template_class_obj = TemplateTestClass(
        static_cast<double>(2.0));
    template_class_obj.Print_a();
    TemplateTestClass template_class_obj2 = TemplateTestClass(
        static_cast<int>(2.0));
    template_class_obj2.Print_a();
    TemplateTestClass template_class_obj3 = TemplateTestClass(
        std::string("sdgfg"));
    template_class_obj3.Print_a();
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
PreciseMeters CalculatePathLength(std::vector<double> planner_waypoints_x, 
                                    std::vector<double> planner_waypoints_y) {
    /*calculate the acumulated distance to last point in the path
     assuming path is given in EGO*/
    PreciseMeters Ego_pos_x = 0.0;
    PreciseMeters Ego_pos_y = 0.0;
    PreciseMeters path_len_ = 0.0;
    for (uint16_t i = 0; i < planner_waypoints_x.size(); i++) {
        if (planner_waypoints_x[i] > 0) {
            path_len_ += std::sqrt(std::pow(planner_waypoints_x[i] - Ego_pos_x, 2) + 
                         std::pow(planner_waypoints_y[i] - Ego_pos_y, 2));
            Ego_pos_x = planner_waypoints_x[i];
            Ego_pos_y = planner_waypoints_y[i];
        }
    };
    return path_len_;
}
void TestPathInterp(){
    std::vector<double> input_x = {0, 0.100472, 0.201123, 0.301777, 0.402434, 0.503093, 0.603753, 0.704416, 0.80508, 0.905746, 1.00641, 1.10708, 1.20775, 1.30842, 1.40909, 1.50976, 1.61044, 1.71111, 1.81178, 1.91246, 2.01313, 2.11381, 2.21448, 2.31516, 2.41583, 2.51651, 2.61718, 2.71786, 2.81854, 2.91921, 3.01989, 3.12056, 3.22124, 3.32191, 3.42259, 3.52326, 3.62393, 3.72461, 3.82528, 3.92596, 4.02663, 4.1273, 4.22797, 4.32865, 4.42932, 4.52999, 4.63066, 4.73133, 4.832, 4.93267, 5.03334, 5.13401, 5.23468, 5.33534, 5.43601};
    std::vector<double> input_y = {0, -0.00692822, -0.00697963, -0.00711737, -0.00733921, -0.00764289, -0.00802614, -0.00848677, -0.0090229, -0.00963238, -0.0103132, -0.0110635, -0.0118815, -0.0127652, -0.013713, -0.0147231, -0.0157939, -0.0169238, -0.018111, -0.0193542, -0.0206517, -0.0220022, -0.0234042, -0.0248562, -0.0263571, -0.0279055, -0.0295, -0.0311396, -0.0328229, -0.0345488, -0.0363161, -0.0381238, -0.0399707, -0.0418558, -0.043778, -0.0457365, -0.0477301, -0.049758, -0.0518192, -0.0539128, -0.056038, -0.0581939, -0.0603796, -0.0625945, -0.0648377, -0.0671084, -0.069406, -0.0717296, -0.0740786, -0.0764523, -0.0788501, -0.0812713, -0.0837152, -0.0861813, -0.0886689};
    const int kN = 10; // number of interpolation points
    double speed_for_optimization_ = 5.0; // m/s
    PreciseSeconds kDt = 0.25;
    PreciseMeters pred_path_len = speed_for_optimization_ * kDt * kN;
    PreciseMeters path_len_ = CalculatePathLength(input_x, input_y);
    PreciseSeconds dt_for_optimization_ = kDt;
    if (pred_path_len >= path_len_) {
        std::cout << "path_len_ = " << path_len_ << ", pred_path_len = " << pred_path_len << std::endl;
        pred_path_len = path_len_;
        dt_for_optimization_ = pred_path_len / speed_for_optimization_ / kN;
    }
    std::vector<PreciseMeters> s_interp(kN + 1, 0.0);
    if (false){ 
        s_interp = {0, 0.543701, 1.0874, 1.6311, 2.1748, 2.7185, 3.26221, 3.80591, 4.34961, 4.89331, 5.43701};
    } else{
        double ds = speed_for_optimization_ * dt_for_optimization_;
        std::cout << "ds = " << ds << std::endl;
        std::cout << "dt_for_optimization_ = " << dt_for_optimization_ << std::endl;
        for (int i = 0; i <= kN; i++){
            s_interp[i] = i * ds;
        }
        s_interp.back() = s_interp.back() + 5e-7;
    }
    std::cout << "s_interp values:\n";
    for (const auto& val : s_interp) {
        std::cout << val << " ";
    }
    std::cout << std::endl;
    auto [x_interp, y_interp, success] = LinearPathInterpulation(input_x, input_y, s_interp);
    if (!success) {
        std::cerr << "[warning] TestPathinterp: Interpolation failed.\n";
        return;
    }
    std::cout << "Interpolated x values:\n";
    for (const auto& val : x_interp) {
        std::cout << val << " ";
    }
    std::cout << "\nInterpolated y values:\n";
    for (const auto& val : y_interp) {
        std::cout << val << " ";
    }
}
void Test_function_convert_path_control_points(){
    // 1. Read path from CSV
    std::string input_csv_path = "../data/temp_results/example_path.csv"; // <-- set your path here
    auto csv_data = ReadCSV(input_csv_path);
    if (csv_data.size() < 2) {
        std::cerr << "CSV file must have at least two columns (x, y)" << std::endl;
        return;
    }
    std::vector<PreciseMeters> path_x(csv_data[0].second.begin(), csv_data[0].second.end());
    std::vector<PreciseMeters> path_y(csv_data[1].second.begin(), csv_data[1].second.end());
    // 2. Set parameters (example values, adjust as needed)
    PreciseMeters WB = 3.75;
    PreciseMeters lr_target = 0.5 * WB;
    PreciseMeters lr_front = WB;
    PreciseMeters lr_rear = 0.0;

    // 3. Call the function for rear and front axle
    auto [rear_x, rear_y, rear_psi] = convert_path_control_points(path_x, path_y, lr_target, lr_rear, WB, false);
    auto [front_x, front_y, front_psi] = convert_path_control_points(path_x, path_y, lr_target, lr_front, WB, false);

    // 4. Write results to CSV for plotting
    std::vector<std::string> headers = {"path_x", "path_y", "rear_x", "rear_y", "front_x", "front_y"};
    std::vector<std::vector<double>> columns;
    columns.push_back(std::vector<double>(path_x.begin(), path_x.end()));
    columns.push_back(std::vector<double>(path_y.begin(), path_y.end()));
    columns.push_back(std::vector<double>(rear_x.begin(), rear_x.end()));
    columns.push_back(std::vector<double>(rear_y.begin(), rear_y.end()));
    columns.push_back(std::vector<double>(front_x.begin(), front_x.end()));
    columns.push_back(std::vector<double>(front_y.begin(), front_y.end()));

    std::vector<std::pair<std::string, std::vector<double>>> data_to_save;
    for (size_t i = 0; i < headers.size(); ++i) {
        data_to_save.emplace_back(headers[i], columns[i]);
    }
    std::string output_csv_path = "../data/convert_path_control_points_output.csv";
    PrintToCSV(output_csv_path, data_to_save, true);

    // 5. Call the Python plotting script
    std::string plot_cmd = "python3 ../Tests/python/plot_inputs.py --path " + output_csv_path +
                           " --x path_x path_x path_x --y path_y rear_y front_y";
    std::cout << "Running: " << plot_cmd << std::endl;
    int ret = system(plot_cmd.c_str());
    if (ret != 0) {
        std::cerr << "Plotting script failed!" << std::endl;
    }

}
