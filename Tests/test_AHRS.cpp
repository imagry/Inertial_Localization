#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <cmath>
#include <iostream>

#include "../Utils/AHRS.hpp"
#include "../Utils/Sensors.hpp"
#include "../Utils/units.hpp"

// Test fixture for AHRS tests, focusing on GyroPromotion
class AHRSTest : public ::testing::Test {
protected:
    // Set up code that will be called before each test
    void SetUp() override {
        // Create a default attitude estimator for testing with required parameters
        double dt = 0.01;  // 10ms sample time
        double Ka = 0.001; // Standard gain value
        std::string convention = "NED"; // North-East-Down convention
        ahrs = std::make_unique<AttitudeEstimator>(dt, Ka, convention);
    }
    
    std::unique_ptr<AttitudeEstimator> ahrs;
};

// Empty test that always passes
TEST_F(AHRSTest, EmptyTestAlwaysPass) {
    // This test doesn't do anything but will always pass
    SUCCEED();
}

// Test that GyroPromotion correctly rotates based on gyro input for X-axis
TEST_F(AHRSTest, GyroPromotionTestXAxis) {
    // Initialize the attitude estimator with identity rotation matrix
    Matrix3d Rnb;
    Rnb << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;
    double dt = 0.01;  // 10ms sample time
    double Ka = 0.0;   // No accelerometer correction
    AttitudeEstimator AE_object(dt, Ka, "NED");
    AE_object.Rnb_ = Rnb;
    
    // Test case: Rotation around X-axis with 100 iterations
    std::vector<double> gyro = {1, 0, 0};  // 1 rad/s around X-axis
    double clock_time = 0.0;
    
    // Apply gyro promotion 100 times to simulate 1 second of rotation
    for (int i = 0; i < 100; i++) {
        AE_object.GyroPromotion(gyro, clock_time);
        clock_time += dt;
    }
    
    // Convert rotation matrix to Euler angles
    std::vector<double> euler = Rot_mat2euler(AE_object.Rnb_);
    
    // Print the actual values for debugging
    std::cout << "After 100 samples, X-axis rotation (phi, theta, psi): [" 
              << euler[0] << ", " << euler[1] << ", " << euler[2] << "]" << std::endl;
    
    // Check that the euler angles match the expected values [1.0,0,0]
    // After 100 iterations of dt=0.01 with angular velocity of 1 rad/s,
    // we expect a total rotation of 1 radian around the X-axis
    EXPECT_NEAR(euler[0], 1.0, 0.001);  // phi (roll) should be 1.0
    EXPECT_NEAR(euler[1], 0.0, 0.001);  // theta (pitch) should remain near zero
    EXPECT_NEAR(euler[2], 0.0, 0.001);  // psi (yaw) should remain near zero
}

// Test that GyroPromotion correctly rotates based on gyro input for Y-axis
TEST_F(AHRSTest, GyroPromotionTestYAxis) {
    // Initialize the attitude estimator with identity rotation matrix
    Matrix3d Rnb;
    Rnb << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;
    double dt = 0.01;  // 10ms sample time
    double Ka = 0.0;   // No accelerometer correction
    AttitudeEstimator AE_object(dt, Ka, "NED");
    AE_object.Rnb_ = Rnb;
    
    // Test case: Rotation around Y-axis with 100 iterations
    std::vector<double> gyro = {0, 1, 0};  // 1 rad/s around Y-axis
    double clock_time = 0.0;
    
    // Apply gyro promotion 100 times to simulate 1 second of rotation
    for (int i = 0; i < 100; i++) {
        AE_object.GyroPromotion(gyro, clock_time);
        clock_time += dt;
    }
    
    // Convert rotation matrix to Euler angles
    std::vector<double> euler = Rot_mat2euler(AE_object.Rnb_);
    
    // Print the actual values for debugging
    std::cout << "After 100 samples, Y-axis rotation (phi, theta, psi): [" 
              << euler[0] << ", " << euler[1] << ", " << euler[2] << "]" << std::endl;
    
    // Check that the euler angles match the expected values [0,1.0,0]
    // After 100 iterations of dt=0.01 with angular velocity of 1 rad/s,
    // we expect a total rotation of 1 radian around the Y-axis
    EXPECT_NEAR(euler[0], 0.0, 0.001);  // phi (roll) should remain near zero
    EXPECT_NEAR(euler[1], 1.0, 0.001);  // theta (pitch) should be 1.0
    EXPECT_NEAR(euler[2], 0.0, 0.001);  // psi (yaw) should remain near zero
}

// Test that GyroPromotion correctly rotates based on gyro input for Z-axis
TEST_F(AHRSTest, GyroPromotionTestZAxis) {
    // Initialize the attitude estimator with identity rotation matrix
    Matrix3d Rnb;
    Rnb << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;
    double dt = 0.01;  // 10ms sample time
    double Ka = 0.0;   // No accelerometer correction
    AttitudeEstimator AE_object(dt, Ka, "NED");
    AE_object.Rnb_ = Rnb;
    
    // Test case: Rotation around Z-axis with 100 iterations
    std::vector<double> gyro = {0, 0, 1};  // 1 rad/s around Z-axis
    double clock_time = 0.0;
    
    // Apply gyro promotion 100 times to simulate 1 second of rotation
    for (int i = 0; i < 100; i++) {
        AE_object.GyroPromotion(gyro, clock_time);
        clock_time += dt;
    }
    
    // Convert rotation matrix to Euler angles
    std::vector<double> euler = Rot_mat2euler(AE_object.Rnb_);
    
    // Print the actual values for debugging
    std::cout << "After 100 samples, Z-axis rotation (phi, theta, psi): [" 
              << euler[0] << ", " << euler[1] << ", " << euler[2] << "]" << std::endl;
    
    // Check that the euler angles match the expected values [0,0,1.0]
    // After 100 iterations of dt=0.01 with angular velocity of 1 rad/s,
    // we expect a total rotation of 1 radian around the Z-axis
    EXPECT_NEAR(euler[0], 0.0, 0.001);  // phi (roll) should remain near zero
    EXPECT_NEAR(euler[1], 0.0, 0.001);  // theta (pitch) should remain near zero
    EXPECT_NEAR(euler[2], 1.0, 0.001);  // psi (yaw) should be 1.0
}
