#pragma once

#include <nlohmann/json.hpp>
#include "Sensors.hpp"
#include "units.hpp"
#include "Classes.hpp"
#include "Functions.hpp"

class GyroBiasStaticEstimator {
public:
    explicit GyroBiasStaticEstimator(const nlohmann::json& localization_config)
        : buffer_size_(static_cast<int>(localization_config["gyro_bias_estimation_buffer_size"])),
          gyroX_buffer_(buffer_size_),
          gyroY_buffer_(buffer_size_),
          gyroZ_buffer_(buffer_size_),
          nominal_dt_(1.0 / static_cast<double>(localization_config["nominal_IMU_freq"])),
          last_update_time_(-1.0) {
        biases_.x = 0.0; biases_.y = 0.0; biases_.z = 0.0;
    }

    void GyroUpdate(const Vec3d& gyros, PreciseSeconds sample_time) {
        if (last_update_time_ < 0.0) {
            // First update initializes last_update_time_
            last_update_time_ = sample_time;
        }
        PreciseSeconds dt = sample_time - last_update_time_;
        if (dt > 10.0 * nominal_dt_) {
            gyroX_buffer_.ResetBuffer();
            gyroY_buffer_.ResetBuffer();
            gyroZ_buffer_.ResetBuffer();
        }
        last_update_time_ = sample_time;

        gyroX_buffer_.Update(static_cast<double>(gyros.x));
        gyroY_buffer_.Update(static_cast<double>(gyros.y));
        gyroZ_buffer_.Update(static_cast<double>(gyros.z));

        if (gyroX_buffer_.Size() == buffer_size_ &&
            gyroY_buffer_.Size() == buffer_size_ &&
            gyroZ_buffer_.Size() == buffer_size_) {
            auto x_vals = gyroX_buffer_.GetBufferValues();
            auto y_vals = gyroY_buffer_.GetBufferValues();
            auto z_vals = gyroZ_buffer_.GetBufferValues();
            biases_.x = Mean(x_vals);
            biases_.y = Mean(y_vals);
            biases_.z = Mean(z_vals);
        }
    }

    Vec3d GetBiases() const { return biases_; }

private:
    Vec3d biases_;
    int buffer_size_;
    BufferAny gyroX_buffer_;
    BufferAny gyroY_buffer_;
    BufferAny gyroZ_buffer_;
    PreciseSeconds last_update_time_;
    PreciseSeconds nominal_dt_;
};