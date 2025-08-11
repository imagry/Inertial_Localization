/* Copyright (C) 2021 Imagry. All Rights Reserved.
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential.
 */

#pragma once

#include <map>
#include <memory>
#include <string>

#include <json/json.h>

#include "inertial_localization_api/ahrs_loc_handler.h"

namespace aidriver::control_api {

static const auto kVehiclConfigPath =
    std::string("modules/control_api/VehicleControl/vehicle_config.json");
static const auto kControlConfigPath =
    std::string("modules/control_api/VehicleControl/localization_config.json");

std::shared_ptr<inertial_localization_api::AhrsLocHandler>
GetAHRSLocHandlerInstance(
    const std::map<std::string, std::string> &external_config =
        std::map<std::string, std::string>(),
    const std::string &vehicle_config_path = kVehiclConfigPath,
    const std::string &control_config_path = kControlConfigPath);

std::shared_ptr<inertial_localization_api::AhrsLocHandler>
GetAHRSLocHandlerInstance(
    const std::map<std::string, std::string> &external_config,
    const Json::Value &vehicle_config, const Json::Value &localization_config);

} // namespace aidriver::control_api
