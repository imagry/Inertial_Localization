#include <gtest/gtest.h>
#include <json/json.h>

#include <fstream>
#include <memory>
#include <string>

#include "inertial_localization_api/ahrs_loc_handler.h"
#include "inertial_localization_api/utils/sensor_types.h"
using inertial_localization_api::AhrsLocHandler;
// Test initialization from default configuration files
TEST(AHRSLocHandlerTest, InitializationFromDefaultFiles) {
  try {
    // Attempt to load the configuration files
    std::ifstream vehicle_file("../vehicle_config.json");
    ASSERT_TRUE(vehicle_file.is_open()) << "Failed to open vehicle_config.json";
    Json::Value vehicle_config;
    Json::Reader reader;
    ASSERT_TRUE(reader.parse(vehicle_file, vehicle_config));

    std::ifstream loc_file("../localization_config.json");
    ASSERT_TRUE(loc_file.is_open())
        << "Failed to open localization_config.json";
    Json::Value localization_config;
    ASSERT_TRUE(reader.parse(loc_file, localization_config));

    // Create the handler
    auto handler =
        std::make_unique<AhrsLocHandler>(vehicle_config, localization_config);
    ASSERT_NE(handler, nullptr) << "Handler was not created successfully";

    // Initial position should be at origin
    auto position = handler->GetPosition();
    EXPECT_EQ(position.size(), 2); // Position has X, Y components
    EXPECT_NEAR(position[0], 0.0, 1e-6);
    EXPECT_NEAR(position[1], 0.0, 1e-6);

    // Verify heading mode
    std::string mode = handler->GetVehicleHeadingEstimationMode();
    EXPECT_FALSE(mode.empty()) << "Heading estimation mode should not be empty";

    // Verify we can get vehicle heading without exception
    EXPECT_NO_THROW(handler->GetVehicleHeading());

  } catch (const std::exception &e) {
    FAIL() << "Exception while testing initialization: " << e.what();
  }
}

// Test vehicle state reset functionality
TEST(AHRSLocHandlerTest, VehicleStateReset) {
  try {
    // Load configuration files
    std::ifstream vehicle_file("../vehicle_config.json");
    ASSERT_TRUE(vehicle_file.is_open()) << "Failed to open vehicle_config.json";
    Json::Value vehicle_config;
    Json::Reader reader;
    ASSERT_TRUE(reader.parse(vehicle_file, vehicle_config));

    std::ifstream loc_file("../localization_config.json");
    ASSERT_TRUE(loc_file.is_open())
        << "Failed to open localization_config.json";
    Json::Value localization_config;
    ASSERT_TRUE(reader.parse(loc_file, localization_config));

    // Create the handler
    auto handler =
        std::make_unique<AhrsLocHandler>(vehicle_config, localization_config);

    // Define a new vehicle state
    const double new_x = 10.5;
    const double new_y = -5.25;
    const double new_heading = 0.75; // About 43 degrees
    const double new_delta = 0.1;    // Small steering angle

    std::vector<double> new_state = {new_x, new_y, new_heading, new_delta};

    // Current time (any value will do for testing)
    PreciseSeconds current_time = 12345.0;

    // Reset the vehicle state
    handler->UpdateVehicleState(current_time, new_state);

    // Verify the new position
    auto position = handler->GetPosition();
    EXPECT_NEAR(position[0], new_x, 1e-6);
    EXPECT_NEAR(position[1], new_y, 1e-6);

    // Verify the new heading
    auto heading = handler->GetVehicleHeading();
    EXPECT_NEAR(heading, new_heading, 1e-6);

  } catch (const std::exception &e) {
    FAIL() << "Exception while testing state reset: " << e.what();
  }
}

// Test that updating rear wheel speeds correctly updates the vehicle speed
TEST(AHRSLocHandlerTest, RearWheelSpeedUpdates) {
  try {
    // Load configuration files
    std::ifstream vehicle_file("../vehicle_config.json");
    ASSERT_TRUE(vehicle_file.is_open()) << "Failed to open vehicle_config.json";
    Json::Value vehicle_config;
    Json::Reader reader;
    ASSERT_TRUE(reader.parse(vehicle_file, vehicle_config));

    std::ifstream loc_file("../localization_config.json");
    ASSERT_TRUE(loc_file.is_open())
        << "Failed to open localization_config.json";
    Json::Value localization_config;
    ASSERT_TRUE(reader.parse(loc_file, localization_config));

    // Set the speed estimation mode to "rear_average" for this test
    localization_config["vehicle_speed_estimation_mode"] = "rear_average";

    // Create the handler with our modified config
    auto handler =
        std::make_unique<AhrsLocHandler>(vehicle_config, localization_config);

    // Define different speeds for left and right wheels
    const PreciseMps left_wheel_speed = 5.2;  // 5.2 m/s
    const PreciseMps right_wheel_speed = 4.8; // 4.8 m/s
    const PreciseMps expected_speed =
        (left_wheel_speed + right_wheel_speed) / 2.0; // Mean value: 5.0

    // Current time (any value will do for testing)
    PreciseSeconds current_time = 12345.0;

    // First set a known vehicle state to avoid NaN in heading
    const double initial_x = 0.0;
    const double initial_y = 0.0;
    const double initial_heading = 0.0; // Straight ahead
    const double initial_delta = 0.0;   // No steering

    std::vector<double> initial_state = {initial_x, initial_y, initial_heading,
                                         initial_delta};

    // Set the initial state
    handler->UpdateVehicleState(current_time, initial_state);

    // Update the wheel speeds
    handler->UpdateRearLeftSpeed(left_wheel_speed, current_time);
    handler->UpdateRearRightSpeed(right_wheel_speed, current_time);

    // Need to estimate speed since that's what propagates the wheel speed
    // values
    handler->EstimateSpeed(current_time);

    // Get the internal state from the ShortTermLocalization object
    auto &loc = handler->GetLoc();
    auto state = loc.State();

    // Verify that the speed in the state is the average of the wheel speeds
    EXPECT_NEAR(state.speed, expected_speed, 1e-6);

  } catch (const std::exception &e) {
    FAIL() << "Exception while testing rear wheel speeds: " << e.what();
  }
}
