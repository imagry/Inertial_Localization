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
#include "Utils/Sensors.hpp"
#include "Utils/localization_debug_states.hpp"
/* B10 deprication
#include "Utils/Delay.hpp"
*/
#include "Utils/short_term_localization.hpp"
#include "Utils/SpeedEstimators.hpp"
#include "Utils/StaticDynamicTest.hpp"
#include "Utils/GyroBiasStaticEstimator.hpp"
using json = nlohmann::json;

class AHRSLocHandler {
    public:
    AHRSLocHandler(const json& vehicle_config, const json& localization_config);
    AHRSLocHandler(const std::string& vehicle_config_path = "vehicle_config.json",
                        const std::string& localization_config_path = "localization_config.json");
    const ShortTermLocalization& GetLoc() const { return localization_obj_; }
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
    
    // Set or reset the vehicle state.
    void UpdateVehicleState(PreciseSeconds clock,
                           const std::vector<double>& state);
                           
    std::vector<PreciseMeters> GetPosition() const;
    PreciseRadians GetVehicleHeading() const;
    std::string GetVehicleHeadingEstimationMode() const;

    StaticDynamicTest::State GetStaticDynamicTestState() const { return static_dynamic_test_obj_.GetState(); }
    std::tuple<double, double, double, double> GetStaticDynamicTestSensorsFeatures() const { return static_dynamic_test_obj_.GetSensorsFeatures(); }

    Vec3d GetGyroBiases() const { return gyro_bias_static_estimator_.GetBiases(); }

 private:
    json vehicle_config_;
    json localization_config_;

    ShortTermLocalization localization_obj_;
    AttitudeEstimator AHRS_obj_;
    std::unique_ptr<SpeedEstimator> speed_estimator_;
    std::mutex debug_obj_lock_;
    std::mutex UpdatePosition_lock_;
    bool debug_mode_;
    LocalizationDebugStates debug_obj_;

    StaticDynamicTest static_dynamic_test_obj_;
    GyroBiasStaticEstimator gyro_bias_static_estimator_;
    
    // Helper method to initialize the appropriate speed estimator
    void InitializeSpeedEstimator();
};
