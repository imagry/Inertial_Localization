#include "StaticDynamicTest.hpp"
#include "Functions.hpp"
#include "DataHandling.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>

StaticDynamicTest::StaticDynamicTest(int acc_buffer_size, int gyro_buffer_size, int car_speed_buffer_size, const nlohmann::json& config) :
    acc_buffer_size_(acc_buffer_size),
    gyro_buffer_size_(gyro_buffer_size),
    car_speed_buffer_size_(car_speed_buffer_size),
    acc_buffer_(acc_buffer_size),
    gyro_buffer_(gyro_buffer_size),
    carspeed_buffer_(car_speed_buffer_size),
    state_(NOT_INITIALIZED)
{
    acc_std_th_ = config["SD_test_acc_std_th"];
    gyro_std_th_ = config["SD_test_gyro_std_th"];
    carspeed_mean_th_ = config["SD_test_carspeed_mean_th"];
    carspeed_max_th_ = config["SD_test_carspeed_max_th"];
}

StaticDynamicTest::StaticDynamicTest(const nlohmann::json& config)
    : StaticDynamicTest(
        static_cast<int>(config["SD_test_acc_buffer_size"]),
        static_cast<int>(config["SD_test_gyro_buffer_size"]),
        static_cast<int>(config["SD_test_carspeed_buffer_size"]),
        config) {}

void StaticDynamicTest::UpdateIMU(const ImuSample& imu_sample) {
    acc_buffer_.Update(VectorNorm3D(imu_sample.acc_));
    gyro_buffer_.Update(VectorNorm3D(imu_sample.gyro_));
    CalculateState();
}

void StaticDynamicTest::UpdateCarSpeed(double car_speed) {
    carspeed_buffer_.Update(car_speed);
    CalculateState();
}

void StaticDynamicTest::CalculateState() {
    if (acc_buffer_.Size() < acc_buffer_size_ ||
        gyro_buffer_.Size() < gyro_buffer_size_ ||
        carspeed_buffer_.Size() < car_speed_buffer_size_) {
        state_ = NOT_INITIALIZED;
        return;
    }

    std::vector<double> acc_vec = acc_buffer_.GetBufferValues();
    std::vector<double> gyro_vec = gyro_buffer_.GetBufferValues();
    std::vector<double> speed_vec = carspeed_buffer_.GetBufferValues();

    acc_std_ = StandardDeviation(acc_vec);
    gyro_std_ = StandardDeviation(gyro_vec);
    speed_mean_ = Mean(speed_vec);
    speed_max_ = *std::max_element(speed_vec.begin(), speed_vec.end());

    if (acc_std_ < acc_std_th_ &&
        gyro_std_ < gyro_std_th_ &&
        speed_mean_ < carspeed_mean_th_ &&
        speed_max_ < carspeed_max_th_) {
        state_ = STATIC;
    } else {
        state_ = DYNAMIC;
    }
}

StaticDynamicTest::State StaticDynamicTest::GetState() const {
    return state_;
}

std::tuple<double, double, double, double> StaticDynamicTest::GetSensorsFeatures() const {
    return std::make_tuple(acc_std_, gyro_std_, speed_mean_, speed_max_);
}