/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/

#include <string>
#include <vector>
#include "ControllerTests.hpp"// NOLINT
#include "Functions.hpp"// NOLINT
#include "Controllers.hpp"// NOLINT
#include "units.hpp"// NOLINT
#include "DataHandling.hpp"// NOLINT
#include <nlohmann/json.hpp>
using json = nlohmann::json;

void TestLQRControllerSingleSample() {
    std::vector<PreciseMeters> traj_x{0.0, 1.0, 2.0};
    std::vector<PreciseMeters> traj_y{0.0, 0.0, 0.0};
    std::vector<PreciseRadians> traj_psi{0.0, 0.0, 0.0};
    double SR = 1.0;
    std::string control_config_path = "../control_config.json";
    std::string vehicle_config_path = "../vehicle_config.json";
    // Print the current working directory for debugging
    std::cout << "Current working directory: " << std::filesystem::current_path() << std::endl;
    ListFilesInDirectory("../");//std::filesystem::current_path()
    if (std::filesystem::exists(control_config_path)) {
        std::cout << "File exists: " << control_config_path << std::endl;
    } else {
        std::cout << "File does not exist: " << control_config_path << std::endl;
    }
    if (std::filesystem::exists(vehicle_config_path)) {
        std::cout << "File exists: " << vehicle_config_path << std::endl;
    } else {
        std::cout << "File does not exist: " << vehicle_config_path << std::endl;
    }
    json control_config = json::parse(std::ifstream(control_config_path));
    json vehicle_config = json::parse(std::ifstream(vehicle_config_path));
    LQRController LC = LQRController(control_config, vehicle_config);
    LC.UpdatePath(traj_x, traj_y);
    PreciseMeters x_v = 0.0;
    PreciseMeters y_v = 0.0;
    PreciseRadians psi_v = -0.2;
    PreciseMps vel = 0.0;//0 / 3.6;
    PreciseRadians current_steering = 0.24;
    auto start = std::chrono::high_resolution_clock::now();
    LC.CalcSteeringCommand(x_v, y_v, psi_v, vel, current_steering, 0.0);
    auto end = std::chrono::high_resolution_clock::now();
    // Print the matrix to the terminal
    // std::cout << "A:\n" << LC.A_ << std::endl;
    // std::cout << "Q:\n" << LC.Q_ << std::endl;
    // std::cout << "R:\n" << LC.R_ << std::endl;
    // std::cout << "B:\n" << LC.B_ << std::endl;
    // std::cout << "P:\n" << LC.saved_P_for_initial_condition_ << std::endl;
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "K:\n" << LC.K_ << std::endl;
    std::cout << "X:\n" << LC.X_ << std::endl;
    std::cout << "delta:\n" << LC.delta_command_ << std::endl;
    std::cout << "Elapsed time: " << elapsed.count() << " seconds" << std::endl;
}
