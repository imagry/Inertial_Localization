#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include <memory>
#include <string>
#include <iostream>

#include "../Utils/short_term_localization.hpp"
#include "../Utils/Sensors.hpp"
#include "../Utils/units.hpp"

// Simple test class (we'll create the ShortTermLocalization object in each test)
class ShortTermLocalizationTest : public ::testing::Test {
};

// Test straight-line driving
TEST_F(ShortTermLocalizationTest, StraightLineDriving) {
    // Create a ShortTermLocalization object for this test
    // Parameters: heading mode, speed mode, wheelbase
    auto localizer = std::make_unique<ShortTermLocalization>("IMU", "default", 2.8);
    
    // Set initial heading to 0 radians (north in NED)
    localizer->UpdateHeading(0.0);
    
    // Set steering angle to 0 (straight)
    localizer->UpdateDelta(0.0);
    
    // 2. Update speed with 1 m/s
    localizer->UpdateSpeed(1.0);
    
    // Create initial state and set the initial time
    LocState initial_state;
    initial_state.pos_ = {0.0, 0.0};
    initial_state.psi_ = 0.0;
    initial_state.delta_ = 0.0;
    initial_state.speed_ = 1.0;
    localizer->ResetVehicleState(0.0, initial_state);
    
    // 3. Update front axle position after 1 second
    localizer->UpdateFrontAxlePosition(1.0);
    
    // 4. Output the state and verify correct
    auto state = localizer->State();
    std::cout << "State (NED) after 1 second:" << std::endl;
    std::cout << "Position: [" << state.pos_[0] << ", " << state.pos_[1] << "]" << std::endl;
    std::cout << "Heading: " << state.psi_ << " rad" << std::endl;
    std::cout << "Steering angle: " << state.delta_ << " rad" << std::endl;
    std::cout << "Speed: " << state.speed_ << " m/s" << std::endl;
    
    // For straight line driving with heading 0, after 1 second at 1 m/s,
    // we should have moved 1 meter north (in NED coordinates)
    EXPECT_NEAR(state.pos_[0], 1.0, 1e-6);  // Forward direction (north) in NED
    EXPECT_NEAR(state.pos_[1], 0.0, 1e-6);  // No lateral movement
    EXPECT_NEAR(state.psi_, 0.0, 1e-6);     // Heading should remain 0
    EXPECT_NEAR(state.delta_, 0.0, 1e-6);   // Steering angle should remain 0
    EXPECT_NEAR(state.speed_, 1.0, 1e-6);   // Speed should remain 1 m/s
    
    // 5. Output the ENU state and verify correct
    auto enu_state = localizer->StateENU();
    std::cout << "State (ENU) after 1 second:" << std::endl;
    std::cout << "Position: [" << enu_state.pos_[0] << ", " << enu_state.pos_[1] << "]" << std::endl;
    std::cout << "Heading: " << enu_state.psi_ << " rad" << std::endl;
    std::cout << "Steering angle: " << enu_state.delta_ << " rad" << std::endl;
    std::cout << "Speed: " << enu_state.speed_ << " m/s" << std::endl;
    
    // In ENU, a forward motion to the north in NED becomes a forward motion to the east
    // and the heading 0 (north) in NED becomes heading of -0 (0 but negated) in ENU
    EXPECT_NEAR(enu_state.pos_[0], 1.0, 1e-6);    // East direction in ENU
    EXPECT_NEAR(enu_state.pos_[1], 0.0, 1e-6);    // No lateral movement
    EXPECT_NEAR(enu_state.psi_, 0.0, 1e-6);       // Heading in ENU (should be negated)
    EXPECT_NEAR(enu_state.delta_, 0.0, 1e-6);     // Steering angle should remain the same
    EXPECT_NEAR(enu_state.speed_, 1.0, 1e-6);     // Speed should remain 1 m/s
}

// Test driving with 45-degree heading
TEST_F(ShortTermLocalizationTest, DrivingWith45DegreeHeading) {
    // Create a ShortTermLocalization object for this test
    // Parameters: heading mode, speed mode, wheelbase
    auto localizer = std::make_unique<ShortTermLocalization>("IMU", "default", 2.8);
    
    // Set initial heading to 45 degrees (PI/4 radians)
    double heading = M_PI / 4.0;
    localizer->UpdateHeading(heading);
    
    // Set steering angle to 0 (straight)
    localizer->UpdateDelta(0.0);
    
    // Update speed with 1 m/s
    localizer->UpdateSpeed(1.0);
    
    // Create initial state and set the initial time
    LocState initial_state;
    initial_state.pos_ = {0.0, 0.0};
    initial_state.psi_ = heading;
    initial_state.delta_ = 0.0;
    initial_state.speed_ = 1.0;
    localizer->ResetVehicleState(0.0, initial_state);
    
    // Update front axle position after 1 second
    localizer->UpdateFrontAxlePosition(1.0);
    
    // Output the state and verify correct
    auto state = localizer->State();
    std::cout << "State (NED) with 45-degree heading after 1 second:" << std::endl;
    std::cout << "Position: [" << state.pos_[0] << ", " << state.pos_[1] << "]" << std::endl;
    std::cout << "Heading: " << state.psi_ << " rad" << std::endl;
    
    // For 45-degree heading, after 1 second at 1 m/s,
    // we should have moved sqrt(2)/2 meters north and sqrt(2)/2 meters east in NED
    double expected_x = cos(heading) * 1.0;  // cos(45°) = sqrt(2)/2
    double expected_y = sin(heading) * 1.0;  // sin(45°) = sqrt(2)/2
    
    EXPECT_NEAR(state.pos_[0], expected_x, 1e-6);  // North component in NED
    EXPECT_NEAR(state.pos_[1], expected_y, 1e-6);  // East component in NED
    EXPECT_NEAR(state.psi_, heading, 1e-6);        // Heading should remain PI/4
    EXPECT_NEAR(state.delta_, 0.0, 1e-6);          // Steering angle should remain 0
    EXPECT_NEAR(state.speed_, 1.0, 1e-6);          // Speed should remain 1 m/s
    
    // Output the ENU state and verify correct
    auto enu_state = localizer->StateENU();
    std::cout << "State (ENU) with 45-degree heading after 1 second:" << std::endl;
    std::cout << "Position: [" << enu_state.pos_[0] << ", " << enu_state.pos_[1] << "]" << std::endl;
    std::cout << "Heading: " << enu_state.psi_ << " rad" << std::endl;
    
    // In ENU, the conversion is:
    // x_enu = x_ned
    // y_enu = -y_ned
    // psi_enu = -psi_ned
    EXPECT_NEAR(enu_state.pos_[0], expected_x, 1e-6);       // East direction in ENU
    EXPECT_NEAR(enu_state.pos_[1], -expected_y, 1e-6);      // North direction in ENU (negated)
    EXPECT_NEAR(enu_state.psi_, -heading, 1e-6);            // Heading in ENU (negated)
    EXPECT_NEAR(enu_state.delta_, 0.0, 1e-6);               // Steering angle should remain the same
    EXPECT_NEAR(enu_state.speed_, 1.0, 1e-6);               // Speed should remain 1 m/s
}

// Test driving with 90-degree heading
TEST_F(ShortTermLocalizationTest, DrivingWith90DegreeHeading) {
    // Create a ShortTermLocalization object for this test
    // Parameters: heading mode, speed mode, wheelbase
    auto localizer = std::make_unique<ShortTermLocalization>("IMU", "default", 2.8);
    
    // Set initial heading to 90 degrees (PI/2 radians)
    double heading = M_PI / 2.0;
    localizer->UpdateHeading(heading);
    
    // Set steering angle to 0 (straight)
    localizer->UpdateDelta(0.0);
    
    // Update speed with 1 m/s
    localizer->UpdateSpeed(1.0);
    
    // Create initial state and set the initial time
    LocState initial_state;
    initial_state.pos_ = {0.0, 0.0};
    initial_state.psi_ = heading;
    initial_state.delta_ = 0.0;
    initial_state.speed_ = 1.0;
    localizer->ResetVehicleState(0.0, initial_state);
    
    // Update front axle position after 1 second
    localizer->UpdateFrontAxlePosition(1.0);
    
    // Output the state and verify correct
    auto state = localizer->State();
    std::cout << "State (NED) with 90-degree heading after 1 second:" << std::endl;
    std::cout << "Position: [" << state.pos_[0] << ", " << state.pos_[1] << "]" << std::endl;
    std::cout << "Heading: " << state.psi_ << " rad" << std::endl;
    
    // For 90-degree heading, after 1 second at 1 m/s,
    // we should have moved 0 meters north and 1 meter east in NED
    double expected_x = cos(heading) * 1.0;  // cos(90°) = 0
    double expected_y = sin(heading) * 1.0;  // sin(90°) = 1
    
    EXPECT_NEAR(state.pos_[0], expected_x, 1e-6);  // North component in NED
    EXPECT_NEAR(state.pos_[1], expected_y, 1e-6);  // East component in NED
    EXPECT_NEAR(state.psi_, heading, 1e-6);        // Heading should remain PI/2
    EXPECT_NEAR(state.delta_, 0.0, 1e-6);          // Steering angle should remain 0
    EXPECT_NEAR(state.speed_, 1.0, 1e-6);          // Speed should remain 1 m/s
    
    // Output the ENU state and verify correct
    auto enu_state = localizer->StateENU();
    std::cout << "State (ENU) with 90-degree heading after 1 second:" << std::endl;
    std::cout << "Position: [" << enu_state.pos_[0] << ", " << enu_state.pos_[1] << "]" << std::endl;
    std::cout << "Heading: " << enu_state.psi_ << " rad" << std::endl;
    
    // In ENU, the conversion is:
    // x_enu = x_ned
    // y_enu = -y_ned
    // psi_enu = -psi_ned
    EXPECT_NEAR(enu_state.pos_[0], expected_x, 1e-6);       // East direction in ENU
    EXPECT_NEAR(enu_state.pos_[1], -expected_y, 1e-6);      // North direction in ENU (negated)
    EXPECT_NEAR(enu_state.psi_, -heading, 1e-6);            // Heading in ENU (negated)
    EXPECT_NEAR(enu_state.delta_, 0.0, 1e-6);               // Steering angle should remain the same
    EXPECT_NEAR(enu_state.speed_, 1.0, 1e-6);               // Speed should remain 1 m/s
}

// Test driving with -45-degree heading
TEST_F(ShortTermLocalizationTest, DrivingWithNegative45DegreeHeading) {
    // Create a ShortTermLocalization object for this test
    // Parameters: heading mode, speed mode, wheelbase
    auto localizer = std::make_unique<ShortTermLocalization>("IMU", "default", 2.8);
    
    // Set initial heading to -45 degrees (-PI/4 radians)
    double heading = -M_PI / 4.0;
    localizer->UpdateHeading(heading);
    
    // Set steering angle to 0 (straight)
    localizer->UpdateDelta(0.0);
    
    // Update speed with 1 m/s
    localizer->UpdateSpeed(1.0);
    
    // Create initial state and set the initial time
    LocState initial_state;
    initial_state.pos_ = {0.0, 0.0};
    initial_state.psi_ = heading;
    initial_state.delta_ = 0.0;
    initial_state.speed_ = 1.0;
    localizer->ResetVehicleState(0.0, initial_state);
    
    // Update front axle position after 1 second
    localizer->UpdateFrontAxlePosition(1.0);
    
    // Output the state and verify correct
    auto state = localizer->State();
    std::cout << "State (NED) with -45-degree heading after 1 second:" << std::endl;
    std::cout << "Position: [" << state.pos_[0] << ", " << state.pos_[1] << "]" << std::endl;
    std::cout << "Heading: " << state.psi_ << " rad" << std::endl;
    
    // For -45-degree heading, after 1 second at 1 m/s,
    // we should have moved sqrt(2)/2 meters north and -sqrt(2)/2 meters east in NED
    double expected_x = cos(heading) * 1.0;  // cos(-45°) = sqrt(2)/2
    double expected_y = sin(heading) * 1.0;  // sin(-45°) = -sqrt(2)/2
    
    EXPECT_NEAR(state.pos_[0], expected_x, 1e-6);  // North component in NED
    EXPECT_NEAR(state.pos_[1], expected_y, 1e-6);  // East component in NED
    EXPECT_NEAR(state.psi_, heading, 1e-6);        // Heading should remain -PI/4
    EXPECT_NEAR(state.delta_, 0.0, 1e-6);          // Steering angle should remain 0
    EXPECT_NEAR(state.speed_, 1.0, 1e-6);          // Speed should remain 1 m/s
    
    // Output the ENU state and verify correct
    auto enu_state = localizer->StateENU();
    std::cout << "State (ENU) with -45-degree heading after 1 second:" << std::endl;
    std::cout << "Position: [" << enu_state.pos_[0] << ", " << enu_state.pos_[1] << "]" << std::endl;
    std::cout << "Heading: " << enu_state.psi_ << " rad" << std::endl;
    
    // In ENU, the conversion is:
    // x_enu = x_ned
    // y_enu = -y_ned
    // psi_enu = -psi_ned
    EXPECT_NEAR(enu_state.pos_[0], expected_x, 1e-6);       // East direction in ENU
    EXPECT_NEAR(enu_state.pos_[1], -expected_y, 1e-6);      // North direction in ENU (negated)
    EXPECT_NEAR(enu_state.psi_, -heading, 1e-6);            // Heading in ENU (negated)
    EXPECT_NEAR(enu_state.delta_, 0.0, 1e-6);               // Steering angle should remain the same
    EXPECT_NEAR(enu_state.speed_, 1.0, 1e-6);               // Speed should remain 1 m/s
}

// Test driving with -90-degree heading
TEST_F(ShortTermLocalizationTest, DrivingWithNegative90DegreeHeading) {
    // Create a ShortTermLocalization object for this test
    // Parameters: heading mode, speed mode, wheelbase
    auto localizer = std::make_unique<ShortTermLocalization>("IMU", "default", 2.8);
    
    // Set initial heading to -90 degrees (-PI/2 radians)
    double heading = -M_PI / 2.0;
    localizer->UpdateHeading(heading);
    
    // Set steering angle to 0 (straight)
    localizer->UpdateDelta(0.0);
    
    // Update speed with 1 m/s
    localizer->UpdateSpeed(1.0);
    
    // Create initial state and set the initial time
    LocState initial_state;
    initial_state.pos_ = {0.0, 0.0};
    initial_state.psi_ = heading;
    initial_state.delta_ = 0.0;
    initial_state.speed_ = 1.0;
    localizer->ResetVehicleState(0.0, initial_state);
    
    // Update front axle position after 1 second
    localizer->UpdateFrontAxlePosition(1.0);
    
    // Output the state and verify correct
    auto state = localizer->State();
    std::cout << "State (NED) with -90-degree heading after 1 second:" << std::endl;
    std::cout << "Position: [" << state.pos_[0] << ", " << state.pos_[1] << "]" << std::endl;
    std::cout << "Heading: " << state.psi_ << " rad" << std::endl;
    
    // For -90-degree heading, after 1 second at 1 m/s,
    // we should have moved 0 meters north and -1 meter east in NED
    double expected_x = cos(heading) * 1.0;  // cos(-90°) = 0
    double expected_y = sin(heading) * 1.0;  // sin(-90°) = -1
    
    EXPECT_NEAR(state.pos_[0], expected_x, 1e-6);  // North component in NED
    EXPECT_NEAR(state.pos_[1], expected_y, 1e-6);  // East component in NED
    EXPECT_NEAR(state.psi_, heading, 1e-6);        // Heading should remain -PI/2
    EXPECT_NEAR(state.delta_, 0.0, 1e-6);          // Steering angle should remain 0
    EXPECT_NEAR(state.speed_, 1.0, 1e-6);          // Speed should remain 1 m/s
    
    // Output the ENU state and verify correct
    auto enu_state = localizer->StateENU();
    std::cout << "State (ENU) with -90-degree heading after 1 second:" << std::endl;
    std::cout << "Position: [" << enu_state.pos_[0] << ", " << enu_state.pos_[1] << "]" << std::endl;
    std::cout << "Heading: " << enu_state.psi_ << " rad" << std::endl;
    
    // In ENU, the conversion is:
    // x_enu = x_ned
    // y_enu = -y_ned
    // psi_enu = -psi_ned
    EXPECT_NEAR(enu_state.pos_[0], expected_x, 1e-6);       // East direction in ENU
    EXPECT_NEAR(enu_state.pos_[1], -expected_y, 1e-6);      // North direction in ENU (negated)
    EXPECT_NEAR(enu_state.psi_, -heading, 1e-6);            // Heading in ENU (negated)
    EXPECT_NEAR(enu_state.delta_, 0.0, 1e-6);               // Steering angle should remain the same
    EXPECT_NEAR(enu_state.speed_, 1.0, 1e-6);               // Speed should remain 1 m/s
}

// Test driving with 0-degree heading and +30-degree steering angle
TEST_F(ShortTermLocalizationTest, DrivingWithZeroHeadingAnd30DegreeSteering) {
    // Create a ShortTermLocalization object for this test
    // Parameters: heading mode, speed mode, wheelbase
    auto localizer = std::make_unique<ShortTermLocalization>("steering_wheel", "default", 2.8);
    
    // Set initial heading to 0 radians (north in NED)
    double heading = 0.0;
    localizer->UpdateHeading(heading);
    
    // Set steering angle to +30 degrees (PI/6 radians)
    double steering_angle = M_PI / 6.0;  // 30 degrees
    localizer->UpdateDelta(steering_angle);
    
    // Update speed with 1 m/s
    localizer->UpdateSpeed(1.0);
    
    // Create initial state and set the initial time
    LocState initial_state;
    initial_state.pos_ = {0.0, 0.0};
    initial_state.psi_ = heading;
    initial_state.delta_ = steering_angle;
    initial_state.speed_ = 1.0;
    localizer->ResetVehicleState(0.0, initial_state);
    
    // Update front axle position after 1 second
    localizer->UpdateFrontAxlePosition(1.0);
    
    // Output the state and verify correct
    auto state = localizer->State();
    std::cout << "State (NED) with 0-degree heading and +30-degree steering after 1 second:" << std::endl;
    std::cout << "Position: [" << state.pos_[0] << ", " << state.pos_[1] << "]" << std::endl;
    std::cout << "Heading: " << state.psi_ << " rad" << std::endl;
    std::cout << "Steering angle: " << state.delta_ << " rad" << std::endl;
    
    // With 0-degree heading and +30-degree steering, after 1 second at 1 m/s,
    // we expect position to change as if the vehicle was heading at +30 degrees
    // and the heading should change according to the bicycle model formula in steering_wheel mode
    double effective_angle = steering_angle;  // In steering mode, the effective angle would be the steering angle
    double expected_x = cos(effective_angle) * 1.0;  // cos(30°) ≈ 0.866
    double expected_y = sin(effective_angle) * 1.0;  // sin(30°) ≈ 0.5
    // For steering_wheel mode, the heading change is calculated as:
    // speed * sin(steering_angle) / wheelbase * dt
    double expected_heading_change = 1.0 * sin(steering_angle) / 2.8 * 1.0;
    double expected_heading = heading + expected_heading_change;
    
    EXPECT_NEAR(state.pos_[0], expected_x, 1e-6);  // North component in NED
    EXPECT_NEAR(state.pos_[1], expected_y, 1e-6);  // East component in NED
    EXPECT_NEAR(state.psi_, expected_heading, 1e-6);  // Heading should change in steering_wheel mode
    EXPECT_NEAR(state.delta_, steering_angle, 1e-6); // Steering angle should remain PI/6
    EXPECT_NEAR(state.speed_, 1.0, 1e-6);          // Speed should remain 1 m/s
    
    // Output the ENU state and verify correct
    auto enu_state = localizer->StateENU();
    std::cout << "State (ENU) with 0-degree heading and +30-degree steering after 1 second:" << std::endl;
    std::cout << "Position: [" << enu_state.pos_[0] << ", " << enu_state.pos_[1] << "]" << std::endl;
    std::cout << "Heading: " << enu_state.psi_ << " rad" << std::endl;
    std::cout << "Steering angle: " << enu_state.delta_ << " rad" << std::endl;
    
    // In ENU, the conversion is:
    // x_enu = x_ned
    // y_enu = -y_ned
    // psi_enu = -psi_ned
    EXPECT_NEAR(enu_state.pos_[0], expected_x, 1e-6);       // East direction in ENU
    EXPECT_NEAR(enu_state.pos_[1], -expected_y, 1e-6);      // North direction in ENU (negated)
    EXPECT_NEAR(enu_state.psi_, -expected_heading, 1e-6);   // Heading in ENU (negated)
    EXPECT_NEAR(enu_state.delta_, steering_angle, 1e-6);    // Steering angle should remain the same
    EXPECT_NEAR(enu_state.speed_, 1.0, 1e-6);               // Speed should remain 1 m/s
}

// Test driving with 0-degree heading and -30-degree steering angle
TEST_F(ShortTermLocalizationTest, DrivingWithZeroHeadingAndNegative30DegreeSteering) {
    // Create a ShortTermLocalization object for this test
    // Parameters: heading mode, speed mode, wheelbase
    auto localizer = std::make_unique<ShortTermLocalization>("steering_wheel", "default", 2.8);
    
    // Set initial heading to 0 radians (north in NED)
    double heading = 0.0;
    localizer->UpdateHeading(heading);
    
    // Set steering angle to -30 degrees (-PI/6 radians)
    double steering_angle = -M_PI / 6.0;  // -30 degrees
    localizer->UpdateDelta(steering_angle);
    
    // Update speed with 1 m/s
    localizer->UpdateSpeed(1.0);
    
    // Create initial state and set the initial time
    LocState initial_state;
    initial_state.pos_ = {0.0, 0.0};
    initial_state.psi_ = heading;
    initial_state.delta_ = steering_angle;
    initial_state.speed_ = 1.0;
    localizer->ResetVehicleState(0.0, initial_state);
    
    // Update front axle position after 1 second
    localizer->UpdateFrontAxlePosition(1.0);
    
    // Output the state and verify correct
    auto state = localizer->State();
    std::cout << "State (NED) with 0-degree heading and -30-degree steering after 1 second:" << std::endl;
    std::cout << "Position: [" << state.pos_[0] << ", " << state.pos_[1] << "]" << std::endl;
    std::cout << "Heading: " << state.psi_ << " rad" << std::endl;
    std::cout << "Steering angle: " << state.delta_ << " rad" << std::endl;
    
    // With 0-degree heading and -30-degree steering, after 1 second at 1 m/s,
    // we expect position to change as if the vehicle was heading at -30 degrees
    // and the heading should change according to the bicycle model formula in steering_wheel mode
    double effective_angle = steering_angle;  // In steering mode, the effective angle would be the steering angle
    double expected_x = cos(effective_angle) * 1.0;  // cos(-30°) ≈ 0.866
    double expected_y = sin(effective_angle) * 1.0;  // sin(-30°) ≈ -0.5
    // For steering_wheel mode, the heading change is calculated as:
    // speed * sin(steering_angle) / wheelbase * dt
    double expected_heading_change = 1.0 * sin(steering_angle) / 2.8 * 1.0;
    double expected_heading = heading + expected_heading_change;
    
    EXPECT_NEAR(state.pos_[0], expected_x, 1e-6);  // North component in NED
    EXPECT_NEAR(state.pos_[1], expected_y, 1e-6);  // East component in NED
    EXPECT_NEAR(state.psi_, expected_heading, 1e-6);  // Heading should change in steering_wheel mode
    EXPECT_NEAR(state.delta_, steering_angle, 1e-6); // Steering angle should remain -PI/6
    EXPECT_NEAR(state.speed_, 1.0, 1e-6);          // Speed should remain 1 m/s
    
    // Output the ENU state and verify correct
    auto enu_state = localizer->StateENU();
    std::cout << "State (ENU) with 0-degree heading and -30-degree steering after 1 second:" << std::endl;
    std::cout << "Position: [" << enu_state.pos_[0] << ", " << enu_state.pos_[1] << "]" << std::endl;
    std::cout << "Heading: " << enu_state.psi_ << " rad" << std::endl;
    std::cout << "Steering angle: " << enu_state.delta_ << " rad" << std::endl;
    
    // In ENU, the conversion is:
    // x_enu = x_ned
    // y_enu = -y_ned
    // psi_enu = -psi_ned
    EXPECT_NEAR(enu_state.pos_[0], expected_x, 1e-6);       // East direction in ENU
    EXPECT_NEAR(enu_state.pos_[1], -expected_y, 1e-6);      // North direction in ENU (negated)
    EXPECT_NEAR(enu_state.psi_, -expected_heading, 1e-6);   // Heading in ENU (negated)
    EXPECT_NEAR(enu_state.delta_, steering_angle, 1e-6);    // Steering angle should remain the same
    EXPECT_NEAR(enu_state.speed_, 1.0, 1e-6);               // Speed should remain 1 m/s
}
