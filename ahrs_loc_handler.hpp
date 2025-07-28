/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#pragma once

#include <mutex>
#include <vector>

#include <nlohmann/json.hpp>

#include "Utils/AHRS.hpp"
#include "Utils/control_debug_states.hpp"
#include "Utils/Delay.hpp"
#include "Utils/short_term_localization.hpp"
#include "Utils/SpeedEstimators.hpp"
using json = nlohmann::json;

class AHRSLocHandler {
    public:
    AHRSLocHandler(const json& vehicle_config, const json& control_config);
    AHRSLocHandler(const std::string& vehicle_config_path = "vehicle_config.json",
                        const std::string& control_config_path = "localization_config.json");
    const ShortTermLocalization& GetLoc() const { return localization_obj_; }
    const Delay<std::vector<double>>& GetDelay() const { return localization_delay_obj_; }
    bool UpdatePosition(PreciseSeconds clock);
    void UpdateIMU(const ImuSample& sample, PreciseSeconds clock);
    void UpdateSpeed(PreciseMps speed, PreciseSeconds clock);
    void UpdateRearRightSpeed(
        PreciseMps rear_right_speed,
        PreciseSeconds clock);
    void UpdateRearLeftSpeed(
        PreciseMps rear_left_speed,
        PreciseSeconds clock);
    void EstimateSpeed(PreciseSeconds clock);
    void UpdateSteeringWheel(PreciseRadians steering_wheel_angle,
                             PreciseSeconds clock);
    void UpdateHeading(PreciseRadians psi, PreciseSeconds clock);
    void UpdateVehicleState(PreciseSeconds clock,
                           const std::vector<double>& state);
    void ResetVehicleState(PreciseSeconds clock,
                            const std::vector<double>& state);
    std::vector<PreciseMeters> GetPosition() const;
    PreciseRadians GetVehicleHeading() const;
    std::string GetVehicleHeadingEstimationMode() const;
    

private:
    json vehicle_config_;
    json control_config_;

    ShortTermLocalization localization_obj_;
    Delay<std::vector<double>> localization_delay_obj_;
    AttitudeEstimator AHRS_obj_;
    std::unique_ptr<SpeedEstimator> speed_estimator_;
    std::mutex debug_obj_lock_;
    std::mutex UpdatePosition_lock_;
    bool debug_mode_;
    ControlDebugStates debug_obj_;
};
