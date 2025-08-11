/* Copyright (C) 2021 Imagry. All Rights Reserved.
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential.
 */

#include "inertial_localization_api/wrapper/localization_api_wrapper.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <json/json.h>

using namespace inertial_localization_api;

namespace aidriver::control_api {

static std::shared_ptr<AhrsLocHandler> CreateAHRSLocHandlerInstance(
    const std::map<std::string, std::string> &external_config,
    const std::string &vehicle_config_path,
    const std::string &control_config_path);

static std::shared_ptr<AhrsLocHandler> CreateAHRSLocHandlerInstance(
    const std::map<std::string, std::string> &external_config,
    const Json::Value &vehicle_config_path,
    const Json::Value &control_config_path);

// merge JSON config with external string dictionary.
// The string dictionary takes precedence.
static Json::Value
merged_configs(const std::map<std::string, std::string> &external_config,
               const std::string &internal_config_path) {
  Json::Value config;
  std::ifstream config_file(internal_config_path);
  Json::Reader reader;
  reader.parse(config_file, config);

  for (const auto &item : external_config) {
    if (!config.isMember(item.first)) {
      continue;
    }
    if (config[item.first].isIntegral()) {
      config[item.first] = std::stoi(item.second);
    } else if (config[item.first].isDouble()) {
      config[item.first] = std::stof(item.second);
    } else if (config[item.first].isString()) {
      config[item.first] = item.second;
    }
  }

  return config;
}

static Json::Value
merged_configs(const std::map<std::string, std::string> &external_config,
               const Json::Value &internal_config) {
  Json::Value config(internal_config);
  for (const auto &item : external_config) {
    if (!config.isMember(item.first)) {
      continue;
    }
    if (config[item.first].isIntegral()) {
      config[item.first] = std::stoi(item.second);
    } else if (config[item.first].isDouble()) {
      config[item.first] = std::stof(item.second);
    } else if (config[item.first].isString()) {
      config[item.first] = item.second;
    }
  }
  return config;
}

std::shared_ptr<AhrsLocHandler> GetAHRSLocHandlerInstance(
    const std::map<std::string, std::string> &external_config,
    const std::string &vehicle_config_path,
    const std::string &control_config_path) {
  Json::Value vehicle_config =
      merged_configs(external_config, vehicle_config_path);
  Json::Value localization_config =
      merged_configs(external_config, control_config_path);
  std::shared_ptr<AhrsLocHandler> ahrs_handler_instance =
      GetAHRSLocHandlerInstance(external_config, vehicle_config,
                                localization_config);
  return ahrs_handler_instance;
}

std::shared_ptr<AhrsLocHandler> CreateAHRSLocHandlerInstance(
    const std::map<std::string, std::string> &external_config,
    const std::string &vehicle_config_path,
    const std::string &control_config_path) {
  Json::Value vehicle_config =
      merged_configs(external_config, vehicle_config_path);
  Json::Value localization_config =
      merged_configs(external_config, control_config_path);
  // Json::Value
  // localization_config(Json::Value::parse(std::ifstream(control_config_path)));
  std::cout << "===== localization handler is created with the following path "
               "configs ====="
            << std::endl;
  std::cout << "localization_config: " << localization_config << std::endl;
  std::cout << "vehicle_config: " << vehicle_config << std::endl;
  auto ahrs_handler_instance =
      std::make_shared<AhrsLocHandler>(vehicle_config, localization_config);

  const std::vector<double> kDefaultVehicleState{0, 0, 0, 0};

  ahrs_handler_instance->UpdateVehicleState(-1.0, kDefaultVehicleState);
  return ahrs_handler_instance;
}

static std::shared_ptr<AhrsLocHandler> CreateAHRSLocHandlerInstance(
    const std::map<std::string, std::string> &external_config,
    const Json::Value &vehicle_config, const Json::Value &localization_config) {
  Json::Value vehicle_config_local =
      merged_configs(external_config, vehicle_config);
  Json::Value control_config_local =
      merged_configs(external_config, localization_config);
  std::cout << "===== localization handler is created with the following json "
               "configs ====="
            << std::endl;
  std::cout << "localization_config: " << localization_config << std::endl;
  std::cout << "vehicle_config: " << vehicle_config << std::endl;
  auto ahrs_handler_instance = std::make_shared<AhrsLocHandler>(
      vehicle_config_local, control_config_local);

  const std::vector<double> kDefaultVehicleState{0, 0, 0, 0};

  ahrs_handler_instance->UpdateVehicleState(-1.0, kDefaultVehicleState);
  return ahrs_handler_instance;
}

std::shared_ptr<AhrsLocHandler> GetAHRSLocHandlerInstance(
    const std::map<std::string, std::string> &external_config,
    const Json::Value &vehicle_config, const Json::Value &localization_config) {
  static std::shared_ptr<AhrsLocHandler> ahrs_handler_instance =
      CreateAHRSLocHandlerInstance(external_config, vehicle_config,
                                   localization_config);

  return ahrs_handler_instance;
}

} // namespace aidriver::control_api
