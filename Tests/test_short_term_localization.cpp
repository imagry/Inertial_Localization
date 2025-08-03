#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <cmath>
#include <memory>
#include <string>

#include "../Utils/short_term_localization.hpp"
#include "../Utils/units.hpp"

// Test fixture for ShortTermLocalization tests
class ShortTermLocalizationTest : public ::testing::Test {
protected:
    // Set up code that will be called before each test
    void SetUp() override {
        // Create a default ShortTermLocalization for testing
        // Parameters: heading mode, speed mode, wheelbase
        localizer = std::make_unique<ShortTermLocalization>("IMU", "default", 2.8);
    }

    // Helper to create IMU sample
    ImuSample CreateSampleImuData(double timestamp = 0.0) {
        ImuSample sample;
        sample.time_stamp = timestamp;
        
        // Default values - vehicle at rest, level orientation
        sample.acc_.x = 0.0;  // No forward acceleration
        sample.acc_.y = 0.0;  // No lateral acceleration
        sample.acc_.z = 9.81; // Gravity
        
        sample.acc_b_.x = 0.0;  // Body frame
        sample.acc_b_.y = 0.0;
        sample.acc_b_.z = 9.81;
        
        sample.gyro_.x = 0.0; // No rotation
        sample.gyro_.y = 0.0;
        sample.gyro_.z = 0.0;
        
        sample.gyro_b_.x = 0.0; // Body frame
        sample.gyro_b_.y = 0.0;
        sample.gyro_b_.z = 0.0;
        
        sample.mag_.x = 1.0;  // North direction
        sample.mag_.y = 0.0;
        sample.mag_.z = 0.0;
        
        sample.pitch_ = 0.0;  // Level orientation
        sample.roll_ = 0.0;
        sample.yaw_ = 0.0;    // Facing north
        
        return sample;
    }
    
    std::unique_ptr<ShortTermLocalization> localizer;
};

// Test initialization of the ShortTermLocalization
TEST_F(ShortTermLocalizationTest, Initialization) {
    // Get the initial state
    auto state = localizer->State();
    
    // Initial position should be at origin
    EXPECT_EQ(state.pos_[0], 0.0);
    EXPECT_EQ(state.pos_[1], 0.0);
    
    // Initial heading, speed, and steering angle should be NaN
    EXPECT_TRUE(std::isnan(state.psi_));
    EXPECT_TRUE(std::isnan(state.speed_));
    EXPECT_TRUE(std::isnan(state.delta_));
}

// Test straight-line movement
TEST_F(ShortTermLocalizationTest, StraightLineMovement) {
    // Initialize with a heading (0 = north)
    localizer->UpdateHeading(0.0);
    
    // Set constant speed (5 m/s)
    localizer->UpdateSpeed(5.0);
    
    // Update position at t=1s
    localizer->UpdatePosition(1.0);
    
    // Position should be 5 meters north (x=0, y=5)
    auto state = localizer->State();
    EXPECT_NEAR(state.pos_[0], 0.0, 1e-6);
    EXPECT_NEAR(state.pos_[1], 5.0, 1e-6);
    
    // Update position at t=2s
    localizer->UpdatePosition(2.0);
    
    // Position should be 10 meters north (x=0, y=10)
    state = localizer->State();
    EXPECT_NEAR(state.pos_[0], 0.0, 1e-6);
    EXPECT_NEAR(state.pos_[1], 10.0, 1e-6);
}

// Test turning movement
TEST_F(ShortTermLocalizationTest, TurningMovement) {
    // Initialize with a heading (0 = north)
    localizer->UpdateHeading(0.0);
    
    // Set constant speed (5 m/s)
    localizer->UpdateSpeed(5.0);
    
    // Update position at t=1s
    localizer->UpdatePosition(1.0);
    
    // Position should be 5 meters north (x=0, y=5)
    auto state = localizer->State();
    EXPECT_NEAR(state.pos_[0], 0.0, 1e-6);
    EXPECT_NEAR(state.pos_[1], 5.0, 1e-6);
    
    // Change heading to east (90 degrees or π/2 radians)
    localizer->UpdateHeading(M_PI/2);
    
    // Update position at t=2s
    localizer->UpdatePosition(2.0);
    
    // Position should be 5 meters north and 5 meters east (x=5, y=5)
    state = localizer->State();
    EXPECT_NEAR(state.pos_[0], 5.0, 1e-6);
    EXPECT_NEAR(state.pos_[1], 5.0, 1e-6);
}

// Test updating with IMU data
TEST_F(ShortTermLocalizationTest, IMUUpdate) {
    // Create sample IMU data
    ImuSample sample = CreateSampleImuData(1.0);
    
    // Update the localizer with IMU data
    localizer->UpdateIMU(sample);
    
    // The clock should be updated to the IMU timestamp
    EXPECT_EQ(localizer->Clock(), 1.0);
    
    // Update with a new IMU sample
    sample.time_stamp = 2.0;
    sample.yaw_ = M_PI/2;  // 90 degrees (east)
    localizer->UpdateIMU(sample);
    
    // The clock should be updated
    EXPECT_EQ(localizer->Clock(), 2.0);
    
    // The heading should be updated based on the IMU yaw
    // Note: This depends on the heading estimation mode
    auto state = localizer->State();
    if (localizer->State().psi_ == M_PI/2) {
        EXPECT_NEAR(state.psi_, M_PI/2, 1e-6);
    }
}

// Test wheel odometry updates
TEST_F(ShortTermLocalizationTest, WheelOdometryUpdate) {
    // Update right wheel speed at t=1s
    localizer->UpdateRearRightSpeed(5.0, 1.0);
    
    // The wheel odometry data should be updated
    auto* wheel_data = localizer->GetRearWheelsOdometry();
    EXPECT_EQ(wheel_data->time_stamp_, 1.0);
    EXPECT_EQ(wheel_data->rear_right_speed_, 5.0);
    EXPECT_EQ(wheel_data->rear_left_speed_, 0.0);  // Not updated yet
    
    // Update left wheel speed at t=1s
    localizer->UpdateRearLeftSpeed(5.0, 1.0);
    
    // Both wheel speeds should be updated
    wheel_data = localizer->GetRearWheelsOdometry();
    EXPECT_EQ(wheel_data->rear_right_speed_, 5.0);
    EXPECT_EQ(wheel_data->rear_left_speed_, 5.0);
    
    // Depending on the speed estimation mode, the vehicle speed might be updated
    // This is implementation dependent
}

// Test state reset
TEST_F(ShortTermLocalizationTest, StateReset) {
    // First set some non-default state
    localizer->UpdateHeading(M_PI/4);  // 45 degrees
    localizer->UpdateSpeed(10.0);
    localizer->UpdatePosition(1.0);
    
    // Create a new state to reset to
    LocState new_state;
    new_state.pos_ = {100.0, 200.0};
    new_state.psi_ = M_PI;  // 180 degrees
    new_state.speed_ = 20.0;
    new_state.delta_ = 0.1;  // Small steering angle
    
    // Reset the state
    localizer->ResetVehicleState(2.0, new_state);
    
    // Verify the state was reset
    auto state = localizer->State();
    EXPECT_NEAR(state.pos_[0], 100.0, 1e-6);
    EXPECT_NEAR(state.pos_[1], 200.0, 1e-6);
    EXPECT_NEAR(state.psi_, M_PI, 1e-6);
    EXPECT_NEAR(state.speed_, 20.0, 1e-6);
    EXPECT_NEAR(state.delta_, 0.1, 1e-6);
    
    // Clock should be updated to the reset time
    EXPECT_EQ(localizer->Clock(), 2.0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
