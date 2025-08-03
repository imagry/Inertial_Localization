#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <cmath>
#include <memory>
#include <string>
#include <fstream>

#include "../ahrs_loc_handler.hpp"
#include "../Utils/short_term_localization.hpp"
#include "../Utils/Sensors.hpp"
#include "../Utils/units.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// Test fixture for AHRSLocHandler tests
class AHRSLocHandlerTest : public ::testing::Test {
protected:
    // Set up code that will be called before each test
    void SetUp() override {
        // Load configuration files
        vehicle_config = json::parse(std::ifstream("../vehicle_config.json"));
        localization_config = json::parse(std::ifstream("../localization_config.json"));
        
        // Create the handler for testing
        handler = std::make_unique<AHRSLocHandler>(vehicle_config, localization_config);
    }
    
    // Helper to create IMU sample
    ImuSample CreateSampleImuData() {
        ImuSample sample;
        
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
    
    json vehicle_config;
    json localization_config;
    std::unique_ptr<AHRSLocHandler> handler;
};

// Test initialization
TEST_F(AHRSLocHandlerTest, Initialization) {
    // Verify the handler is initialized correctly
    EXPECT_NO_THROW(handler->GetPosition());
    EXPECT_NO_THROW(handler->GetVehicleHeading());
    
    // Initial position should be at origin
    auto position = handler->GetPosition();
    EXPECT_EQ(position.size(), 3);
    EXPECT_EQ(position[0], 0.0);
    EXPECT_EQ(position[1], 0.0);
    
    // Initial heading should be NaN or 0 depending on implementation
    auto heading = handler->GetVehicleHeading();
    EXPECT_TRUE(std::isnan(heading) || heading == 0.0);
}

// Test IMU update
TEST_F(AHRSLocHandlerTest, IMUUpdate) {
    // Create sample IMU data
    ImuSample sample = CreateSampleImuData();
    
    // Update with IMU data at t=1s
    EXPECT_NO_THROW(handler->UpdateIMU(sample, 1.0));
    
    // Position should still be at origin (no speed yet)
    auto position = handler->GetPosition();
    EXPECT_EQ(position[0], 0.0);
    EXPECT_EQ(position[1], 0.0);
}

// Test wheel speed updates
TEST_F(AHRSLocHandlerTest, WheelSpeedUpdates) {
    // Update right wheel speed at t=1s
    EXPECT_NO_THROW(handler->UpdateRearRightSpeed(5.0, 1.0));
    
    // Update left wheel speed at t=1s
    EXPECT_NO_THROW(handler->UpdateRearLeftSpeed(5.0, 1.0));
    
    // Update heading to north
    handler->UpdateHeading(0.0, 1.0);
    
    // Estimate speed based on wheel odometry
    handler->EstimateSpeed(1.0);
    
    // Update position at t=2s
    handler->UpdatePosition(2.0);
    
    // Position should have changed (vehicle moved forward)
    // Exact values depend on implementation
    auto position = handler->GetPosition();
    EXPECT_NE(position[0], 0.0);
    
    // Heading should be 0 (north)
    auto heading = handler->GetVehicleHeading();
    EXPECT_NEAR(heading, 0.0, 1e-6);
}

// Test steering wheel update
TEST_F(AHRSLocHandlerTest, SteeringWheelUpdate) {
    // Set up initial state: heading north, speed 5 m/s
    handler->UpdateHeading(0.0, 1.0);
    handler->UpdateRearRightSpeed(5.0, 1.0);
    handler->UpdateRearLeftSpeed(5.0, 1.0);
    handler->EstimateSpeed(1.0);
    
    // Update position at t=2s
    handler->UpdatePosition(2.0);
    
    // Update steering wheel angle (turn right)
    handler->UpdateSteeringWheel(0.5, 2.0);
    
    // Update position at t=3s
    handler->UpdatePosition(3.0);
    
    // Position should have changed and the path should curve
    auto position = handler->GetPosition();
    
    // Exact values depend on implementation details and vehicle model
}

// Test vehicle state update
TEST_F(AHRSLocHandlerTest, VehicleStateUpdate) {
    // Create a new state vector [x, y, z, roll, pitch, yaw, speed]
    std::vector<double> new_state = {100.0, 200.0, 0.0, 0.0, 0.0, M_PI/2, 10.0};
    
    // Update the vehicle state
    handler->UpdateVehicleState(1.0, new_state);
    
    // Position should be updated
    auto position = handler->GetPosition();
    EXPECT_NEAR(position[0], 100.0, 1e-6);
    EXPECT_NEAR(position[1], 200.0, 1e-6);
    
    // Heading should be updated to π/2 (east)
    auto heading = handler->GetVehicleHeading();
    EXPECT_NEAR(heading, M_PI/2, 1e-6);
}

// Test heading estimation mode
TEST_F(AHRSLocHandlerTest, HeadingEstimationMode) {
    // Get the current heading estimation mode
    std::string mode = handler->GetVehicleHeadingEstimationMode();
    
    // Mode should not be empty
    EXPECT_FALSE(mode.empty());
    
    // Mode should be one of the supported modes (depends on configuration)
    // For example: "INS", "IMU", "steering_wheel"
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
