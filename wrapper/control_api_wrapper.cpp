/* Copyright (C) 2021 Imagry. All Rights Reserved.
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential.
 */

#pragma once

#include "control_api_wrapper.h"

#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace aidriver::control_api {

// B10: Removed as part of control code removal
// static std::shared_ptr<ControlAPI> CreateControlAPIInstance(
//     const std::map<std::string, std::string>& external_config,
//     const std::string& vehicle_config_path,
//     const std::string& control_config_path);

static std::shared_ptr<AHRSLocHandler> CreateAHRSLocHandlerInstance(
    const std::map<std::string, std::string>& external_config,
    const std::string& vehicle_config_path,
    const std::string& control_config_path);

static std::shared_ptr<AHRSLocHandler> CreateAHRSLocHandlerInstance(
    const std::map<std::string, std::string>& external_config,
    const json& vehicle_config_path,
    const json& control_config_path);

// merge JSON config with external string dictionary.
// The string dictionary takes precedence.
static json merged_configs(
    const std::map<std::string, std::string>& external_config,
    const std::string& internal_config_path) {
    json config(json::parse(std::ifstream(internal_config_path)));
    const std::string dlc_prefix = "dlc_";

    for (const auto& item : external_config) {
        if (item.first.rfind(dlc_prefix, 0) == 0) { // Check if key starts with dlc_prefix
            std::string key_without_prefix = item.first.substr(dlc_prefix.size());
            if (config["Longitudinal_controller"].contains(key_without_prefix)) {
                if (config["Longitudinal_controller"][key_without_prefix].is_number_integer()) {
                    config["Longitudinal_controller"][key_without_prefix] = std::stoi(item.second);
                } else if (config["Longitudinal_controller"][key_without_prefix].is_number_float()) {
                    config["Longitudinal_controller"][key_without_prefix] = std::stof(item.second);
                } else if (config["Longitudinal_controller"][key_without_prefix].is_string()) {
                    config["Longitudinal_controller"][key_without_prefix] = item.second;
                }
            }
        } else {
            if (!config.contains(item.first)) { continue; }
            if (config[item.first].is_number_integer()) {
                config[item.first] = std::stoi(item.second);
            } else if (config[item.first].is_number_float()) {
                config[item.first] = std::stof(item.second);
            } else if (config[item.first].is_string()) {
                config[item.first] = item.second;
            }
        }
    }

    return config;
}

static json merged_configs(
    const std::map<std::string, std::string>& external_config,
    const json& internal_config) {
    json config(internal_config);
    const std::string dlc_prefix = "dlc_";

    for (const auto& item : external_config) {
        if (item.first.rfind(dlc_prefix, 0) == 0) { // Check if key starts with dlc_prefix
            std::string key_without_prefix = item.first.substr(dlc_prefix.size());
            if (config["Longitudinal_controller"].contains(key_without_prefix)) {
                if (config["Longitudinal_controller"][key_without_prefix].is_number_integer()) {
                    config["Longitudinal_controller"][key_without_prefix] = std::stoi(item.second);
                } else if (config["Longitudinal_controller"][key_without_prefix].is_number_float()) {
                    config["Longitudinal_controller"][key_without_prefix] = std::stof(item.second);
                } else if (config["Longitudinal_controller"][key_without_prefix].is_string()) {
                    config["Longitudinal_controller"][key_without_prefix] = item.second;
                }
            }
        } else {
            if (!config.contains(item.first)) { continue; }
            if (config[item.first].is_number_integer()) {
                config[item.first] = std::stoi(item.second);
            } else if (config[item.first].is_number_float()) {
                config[item.first] = std::stof(item.second);
            } else if (config[item.first].is_string()) {
                config[item.first] = item.second;
            }
        }
    }
    return config;
}

// B10: Removed as part of control code removal
// std::shared_ptr<ControlAPI> GetControlAPIInstance(
//     const std::map<std::string, std::string>& external_config,
//     const std::string& vehicle_config_path,
//     const std::string& control_config_path) {
//     static std::shared_ptr<ControlAPI> control_api_instance =
//         CreateControlAPIInstance(external_config, vehicle_config_path, control_config_path);
// 
//     return control_api_instance;
// }

// B10: Removed as part of control code removal
// std::shared_ptr<ControlAPI> CreateControlAPIInstance(
//     const std::map<std::string, std::string>& external_config,
//     const std::string& vehicle_config_path,
//     const std::string& control_config_path) {
//     json vehicle_config = merged_configs(external_config, vehicle_config_path);
//     // json control_config(json::parse(std::ifstream(control_config_path)));
//     json control_config = merged_configs(external_config, control_config_path);
//     std::cout << "===== control_api is created with the following configs =====" << std::endl;
//     std::cout << "control_config: " << control_config << std::endl;
//     std::cout << "vehicle_config: " << vehicle_config << std::endl;
//     return std::make_shared<ControlAPI>(
//         vehicle_config, control_config);
// }

// std::map<std::string, std::string>


std::shared_ptr<AHRSLocHandler> GetAHRSLocHandlerInstance(
    const std::map<std::string, std::string>& external_config,
    const std::string& vehicle_config_path,
    const std::string& control_config_path) {
    json vehicle_config = merged_configs(external_config, vehicle_config_path);
    json control_config = merged_configs(external_config, control_config_path);
    std::shared_ptr<AHRSLocHandler> ahrs_handler_instance = 
        GetAHRSLocHandlerInstance(external_config, vehicle_config, control_config);
    return ahrs_handler_instance;
}

std::shared_ptr<AHRSLocHandler> CreateAHRSLocHandlerInstance(
    const std::map<std::string, std::string>& external_config,
    const std::string& vehicle_config_path,
    const std::string& control_config_path) {
    json vehicle_config = merged_configs(external_config, vehicle_config_path);
    json control_config = merged_configs(external_config, control_config_path);
    // json control_config(json::parse(std::ifstream(control_config_path)));
    std::cout << "===== localization handler is created with the following path configs =====" << std::endl;
    std::cout << "control_config: " << control_config << std::endl;
    std::cout << "vehicle_config: " << vehicle_config << std::endl;
    auto ahrs_handler_instance = std::make_shared<AHRSLocHandler>(
        vehicle_config, control_config);

    const std::vector<double> kDefaultVehicleState{0, 0, 0, 0};

    ahrs_handler_instance->ResetVehicleState(-1.0, kDefaultVehicleState);
    return ahrs_handler_instance;
}

static std::shared_ptr<AHRSLocHandler> CreateAHRSLocHandlerInstance(
    const std::map<std::string, std::string>& external_config,
    const json& vehicle_config,
    const json& control_config) {
        json vehicle_config_local = merged_configs(external_config, vehicle_config);
        json control_config_local = merged_configs(external_config, control_config);
        std::cout << "===== localization handler is created with the following json configs =====" << std::endl;
        std::cout << "control_config: " << control_config << std::endl;
        std::cout << "vehicle_config: " << vehicle_config << std::endl;
        auto ahrs_handler_instance = std::make_shared<AHRSLocHandler>(
        vehicle_config_local, control_config_local);

    const std::vector<double> kDefaultVehicleState{0, 0, 0, 0};

    ahrs_handler_instance->ResetVehicleState(-1.0, kDefaultVehicleState);
    return ahrs_handler_instance;
}

std::shared_ptr<AHRSLocHandler> GetAHRSLocHandlerInstance(
    const std::map<std::string, std::string>& external_config,
    const json& vehicle_config,
    const json& control_config) {
    static std::shared_ptr<AHRSLocHandler> ahrs_handler_instance =
        CreateAHRSLocHandlerInstance(external_config, vehicle_config, control_config);

    return ahrs_handler_instance;
}

}  // namespace aidriver::control_api
