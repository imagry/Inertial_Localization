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

    double acc_std = StandardDeviation(acc_vec);
    double gyro_std = StandardDeviation(gyro_vec);
    double speed_mean = Mean(speed_vec);
    double speed_max = *std::max_element(speed_vec.begin(), speed_vec.end());

    if (acc_std < acc_std_th_ &&
        gyro_std < gyro_std_th_ &&
        speed_mean < carspeed_mean_th_ &&
        speed_max < carspeed_max_th_) {
        state_ = STATIC;
    } else {
        state_ = DYNAMIC;
    }
}

StaticDynamicTest::State StaticDynamicTest::GetState() const {
    return state_;
}