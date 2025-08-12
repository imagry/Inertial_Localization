#pragma once

#include "Classes.hpp"
#include "Sensors.hpp"
#include <vector>
#include <nlohmann/json.hpp>
#include <tuple>

class StaticDynamicTest {
public:
    enum State {
        NOT_INITIALIZED = 0,
        STATIC = 1,
        DYNAMIC = 2
    };

    StaticDynamicTest(int acc_buffer_size, int gyro_buffer_size, int car_speed_buffer_size, const nlohmann::json& config);
    StaticDynamicTest(const nlohmann::json& config);

    void UpdateIMU(const ImuSample& imu_sample);
    void UpdateCarSpeed(double car_speed);
    State GetState() const;
    std::tuple<double, double, double, double> GetSensorsFeatures() const; // (acc_std_, gyro_std_, speed_mean_, speed_max_)

private:
    void CalculateState();

    BufferAny acc_buffer_;
    BufferAny gyro_buffer_;
    BufferAny carspeed_buffer_;
    State state_;

    int acc_buffer_size_;
    int gyro_buffer_size_;
    int car_speed_buffer_size_;

    double acc_std_th_;
    double gyro_std_th_;
    double carspeed_mean_th_;
    double carspeed_max_th_;

    // Tracked features
    double acc_std_ = 0.0;
    double gyro_std_ = 0.0;
    double speed_mean_ = 0.0;
    double speed_max_ = 0.0;
};