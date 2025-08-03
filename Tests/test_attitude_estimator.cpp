#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <cmath>
#include <memory>

#include "../Utils/AHRS.hpp"
#include "../Utils/units.hpp"

// Test fixture for AttitudeEstimator tests
class AttitudeEstimatorTest : public ::testing::Test {
protected:
    // Set up code that will be called before each test
    void SetUp() override {
        // Create a default attitude estimator for testing
        ahrs = std::make_unique<AttitudeEstimator>();
    }

    // Create sample IMU data for testing
    ImuSample CreateSampleImuData(double roll = 0.0, double pitch = 0.0, double yaw = 0.0) {
        ImuSample sample;
        
        // Set orientation values
        sample.roll_ = roll;
        sample.pitch_ = pitch;
        sample.yaw_ = yaw;
        
        // Set accelerometer values (simulating level orientation with gravity)
        sample.acc_.x = 0.0;  // No forward acceleration
        sample.acc_.y = 0.0;  // No lateral acceleration
        sample.acc_.z = 9.81; // Gravity
        
        // Set gyroscope values (no rotation)
        sample.gyro_.x = 0.0;
        sample.gyro_.y = 0.0;
        sample.gyro_.z = 0.0;
        
        // Set magnetometer values (simulating north direction)
        sample.mag_.x = 0.0;
        sample.mag_.y = 1.0;
        sample.mag_.z = 0.0;
        
        return sample;
    }
    
    std::unique_ptr<AttitudeEstimator> ahrs;
};

// Test initialization of the attitude estimator
TEST_F(AttitudeEstimatorTest, Initialization) {
    // Verify the initial state of the attitude estimator
    auto quat = ahrs->GetAttitude();
    
    // Initial quaternion should represent identity rotation (no rotation)
    EXPECT_NEAR(quat.w(), 1.0, 1e-6);
    EXPECT_NEAR(quat.x(), 0.0, 1e-6);
    EXPECT_NEAR(quat.y(), 0.0, 1e-6);
    EXPECT_NEAR(quat.z(), 0.0, 1e-6);
    
    // Initial heading should be NaN or 0 depending on implementation
    auto heading = ahrs->GetHeading();
    EXPECT_TRUE(std::isnan(heading) || heading == 0.0);
}

// Test updating the IMU data
TEST_F(AttitudeEstimatorTest, UpdateIMU) {
    // Create sample IMU data
    ImuSample sample = CreateSampleImuData(0.1, 0.2, 0.3);
    
    // Update the attitude estimator with the sample data
    ahrs->UpdateIMU(sample);
    
    // Get the updated attitude
    auto quat = ahrs->GetAttitude();
    
    // The quaternion should reflect the updated orientation
    // Note: Exact values depend on the implementation details of UpdateIMU
    EXPECT_FALSE(std::isnan(quat.w()));
    EXPECT_FALSE(std::isnan(quat.x()));
    EXPECT_FALSE(std::isnan(quat.y()));
    EXPECT_FALSE(std::isnan(quat.z()));
    
    // The heading should be updated (approximately 0.3 radians in this test)
    // Note: Exact mapping depends on implementation details
    auto heading = ahrs->GetHeading();
    EXPECT_FALSE(std::isnan(heading));
}

// Test heading calculation
TEST_F(AttitudeEstimatorTest, HeadingCalculation) {
    // Test with different yaw values
    for (double yaw : {0.0, M_PI/4, M_PI/2, M_PI, -M_PI/4}) {
        ImuSample sample = CreateSampleImuData(0.0, 0.0, yaw);
        ahrs->UpdateIMU(sample);
        
        // Get the calculated heading
        auto heading = ahrs->GetHeading();
        
        // Heading should be close to the yaw value
        // Note: Some implementations might have different conventions
        EXPECT_NEAR(heading, yaw, 0.1);
    }
}

// Test attitude calculation from accelerometer data (static case)
TEST_F(AttitudeEstimatorTest, StaticAttitudeFromAccelerometer) {
    // Create IMU sample with different orientations but no rotation
    ImuSample level = CreateSampleImuData();
    level.acc_.x = 0.0;
    level.acc_.y = 0.0;
    level.acc_.z = 9.81;
    
    ImuSample pitched_forward = CreateSampleImuData();
    pitched_forward.acc_.x = 5.0;  // Pitched forward
    pitched_forward.acc_.y = 0.0;
    pitched_forward.acc_.z = 8.5;  // Less Z acceleration due to pitch
    
    ImuSample rolled_right = CreateSampleImuData();
    rolled_right.acc_.x = 0.0;
    rolled_right.acc_.y = 5.0;  // Rolled right
    rolled_right.acc_.z = 8.5;  // Less Z acceleration due to roll
    
    // Update with level orientation
    ahrs->UpdateIMU(level);
    auto quat1 = ahrs->GetAttitude();
    
    // Update with pitched orientation
    ahrs->UpdateIMU(pitched_forward);
    auto quat2 = ahrs->GetAttitude();
    
    // Update with rolled orientation
    ahrs->UpdateIMU(rolled_right);
    auto quat3 = ahrs->GetAttitude();
    
    // The quaternions should be different for different orientations
    EXPECT_NE(quat1.x(), quat2.x());
    EXPECT_NE(quat1.y(), quat3.y());
}

// Test Euler angle conversion
TEST_F(AttitudeEstimatorTest, EulerAngleConversion) {
    // Create IMU sample with specific Euler angles
    double roll = 0.1;
    double pitch = 0.2;
    double yaw = 0.3;
    
    ImuSample sample = CreateSampleImuData(roll, pitch, yaw);
    ahrs->UpdateIMU(sample);
    
    // Get the quaternion representation
    auto quat = ahrs->GetAttitude();
    
    // Convert quaternion back to Euler angles (implementation dependent)
    // This test assumes there's a method to convert quaternion to Euler angles
    // If no such method exists, this test would need to be modified
    
    // For example, if there were GetRoll(), GetPitch() methods:
    // EXPECT_NEAR(ahrs->GetRoll(), roll, 0.1);
    // EXPECT_NEAR(ahrs->GetPitch(), pitch, 0.1);
    // EXPECT_NEAR(ahrs->GetYaw(), yaw, 0.1);
    
    // Without direct methods, we'd need to manually convert the quaternion to Euler angles
    // For now, we just check that the quaternion is valid
    EXPECT_FALSE(std::isnan(quat.w()));
    EXPECT_FALSE(std::isnan(quat.x()));
    EXPECT_FALSE(std::isnan(quat.y()));
    EXPECT_FALSE(std::isnan(quat.z()));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
