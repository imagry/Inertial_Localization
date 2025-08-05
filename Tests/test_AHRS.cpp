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
    
    // Store expected values
    const double expected_phi = 1.0;
    const double expected_theta = 0.0;
    const double expected_psi = 0.0;
    
    // Check that the euler angles match the expected values [1.0,0,0]
    // After 100 iterations of dt=0.01 with angular velocity of 1 rad/s,
    // we expect a total rotation of 1 radian around the X-axis
    EXPECT_NEAR(euler[0], expected_phi, 0.001);   // phi (roll) should be 1.0
    EXPECT_NEAR(euler[1], expected_theta, 0.001); // theta (pitch) should remain near zero
    EXPECT_NEAR(euler[2], expected_psi, 0.001);   // psi (yaw) should remain near zero
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
    
    // Store expected values
    const double expected_phi = 0.0;
    const double expected_theta = 1.0;
    const double expected_psi = 0.0;
    
    // Check that the euler angles match the expected values [0,1.0,0]
    // After 100 iterations of dt=0.01 with angular velocity of 1 rad/s,
    // we expect a total rotation of 1 radian around the Y-axis
    EXPECT_NEAR(euler[0], expected_phi, 0.001);   // phi (roll) should remain near zero
    EXPECT_NEAR(euler[1], expected_theta, 0.001); // theta (pitch) should be 1.0
    EXPECT_NEAR(euler[2], expected_psi, 0.001);   // psi (yaw) should remain near zero
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
    
    // Store expected values
    const double expected_phi = 0.0;
    const double expected_theta = 0.0;
    const double expected_psi = 1.0;
    
    // Check that the euler angles match the expected values [0,0,1.0]
    // After 100 iterations of dt=0.01 with angular velocity of 1 rad/s,
    // we expect a total rotation of 1 radian around the Z-axis
    EXPECT_NEAR(euler[0], expected_phi, 0.001);   // phi (roll) should remain near zero
    EXPECT_NEAR(euler[1], expected_theta, 0.001); // theta (pitch) should remain near zero
    EXPECT_NEAR(euler[2], expected_psi, 0.001);   // psi (yaw) should be 1.0
}

// Test UpdateGravity with a small angle change (1 degree around X-axis)
TEST_F(AHRSTest, UpdateGravityTestSmallAngle) {
    // Initialize the attitude estimator with identity rotation matrix
    Matrix3d Rnb;
    Rnb << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;
    double dt = 0.01;      // 10ms sample time
    double Ka = 1.0;       // Full accelerometer correction for faster convergence
    AttitudeEstimator AE_object(dt, Ka, "NED");
    AE_object.Rnb_ = Rnb;
    
    // Test case: Simulate 1 degree rotation around X-axis
    // acc = [0, sin(1), -cos(1)] * 9.81
    double angle_rad = 1.0 * M_PI / 180.0;  // 1 degree in radians
    std::vector<double> acc = {0, -9.81 * sin(angle_rad), -9.81 * cos(angle_rad)};
    
    // Apply gravity update
    AE_object.UpdateGravity(acc);
    
    // Convert rotation matrix to Euler angles
    std::vector<double> euler = Rot_mat2euler(AE_object.Rnb_);
    
    // Convert acceleration to Eigen vector
    Vector3d acc_eigen(acc[0], acc[1], acc[2]);
    
    // Normalize acceleration vector to match gravity vector scale
    Vector3d acc_normalized = acc_eigen / acc_eigen.norm();
    
    // Rotated normalized acceleration vector should align with navigation gravity
    Vector3d projected_gravity = AE_object.Rnb_ * acc_normalized;
    
    // Calculate the error vector between projected gravity and navigation frame gravity
    Vector3d error_vector = projected_gravity - AE_object.gn_;
    double error_norm = error_vector.norm();
    
    // Also check dot product to ensure vectors are properly aligned
    double dot_product = projected_gravity.dot(AE_object.gn_);
    
    // The error should be small, indicating that the vectors are aligned
    EXPECT_NEAR(error_norm, 0.0, 0.1) << "Error vector norm is too large";
    EXPECT_NEAR(dot_product, 1.0, 0.1) << "Vectors are not properly aligned";
    
    // Expected phi (roll) should be approximately 1 degree
    EXPECT_NEAR(euler[0], angle_rad, 0.01) << "Roll angle not as expected";
}

// Test UpdateGravity with negative acceleration in Y-axis
TEST_F(AHRSTest, UpdateGravityTestXAxisNegative) {
    // Initialize the attitude estimator with identity rotation matrix
    Matrix3d Rnb;
    Rnb << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;
    double dt = 0.01;      // 10ms sample time
    double Ka = 1.0;       // Full accelerometer correction for faster convergence
    AttitudeEstimator AE_object(dt, Ka, "NED");
    AE_object.Rnb_ = Rnb;
    
    // Test case: Simulate -1 degree rotation around X-axis
    // acc = [0, -sin(1), -cos(1)] * 9.81
    double angle_rad = 1.0 * M_PI / 180.0;  // 1 degree in radians
    std::vector<double> acc = {0, 9.81 * sin(angle_rad), -9.81 * cos(angle_rad)};
    
    // Apply gravity update
    AE_object.UpdateGravity(acc);
    
    // Convert rotation matrix to Euler angles
    std::vector<double> euler = Rot_mat2euler(AE_object.Rnb_);
    
    // Convert acceleration to Eigen vector
    Vector3d acc_eigen(acc[0], acc[1], acc[2]);
    
    // Normalize acceleration vector to match gravity vector scale
    Vector3d acc_normalized = acc_eigen / acc_eigen.norm();
    
    // Rotated normalized acceleration vector should align with navigation gravity
    Vector3d projected_gravity = AE_object.Rnb_ * acc_normalized;
    
    // Calculate the error vector between projected gravity and navigation frame gravity
    Vector3d error_vector = projected_gravity - AE_object.gn_;
    double error_norm = error_vector.norm();
    
    // Also check dot product to ensure vectors are properly aligned
    double dot_product = projected_gravity.dot(AE_object.gn_);
    
    // The error should be small, indicating that the vectors are aligned
    EXPECT_NEAR(error_norm, 0.0, 0.1) << "Error vector norm is too large";
    EXPECT_NEAR(dot_product, 1.0, 0.1) << "Vectors are not properly aligned";
    
    // Expected phi (roll) should be approximately -1 degree
    EXPECT_NEAR(euler[0], -1.0 * angle_rad, 0.01) << "Roll angle not as expected";
}

// Test UpdateGravity with positive acceleration in X-axis (Y-axis rotation)
TEST_F(AHRSTest, UpdateGravityTestYAxisPositive) {
    // Initialize the attitude estimator with identity rotation matrix
    Matrix3d Rnb;
    Rnb << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;
    double dt = 0.01;      // 10ms sample time
    double Ka = 1.0;       // Full accelerometer correction for faster convergence
    AttitudeEstimator AE_object(dt, Ka, "NED");
    AE_object.Rnb_ = Rnb;
    
    // Test case: Simulate 1 degree rotation around Y-axis
    // acc = [sin(1), 0, -cos(1)] * 9.81
    double angle_rad = 1.0 * M_PI / 180.0;  // 1 degree in radians
    std::vector<double> acc = {9.81 * sin(angle_rad), 0, -9.81 * cos(angle_rad)};
    
    // Apply gravity update
    AE_object.UpdateGravity(acc);
    
    // Convert rotation matrix to Euler angles
    std::vector<double> euler = Rot_mat2euler(AE_object.Rnb_);
    
    // Convert acceleration to Eigen vector
    Vector3d acc_eigen(acc[0], acc[1], acc[2]);
    
    // Normalize acceleration vector to match gravity vector scale
    Vector3d acc_normalized = acc_eigen / acc_eigen.norm();
    
    // Rotated normalized acceleration vector should align with navigation gravity
    Vector3d projected_gravity = AE_object.Rnb_ * acc_normalized;
    
    // Calculate the error vector between projected gravity and navigation frame gravity
    Vector3d error_vector = projected_gravity - AE_object.gn_;
    double error_norm = error_vector.norm();
    
    // Also check dot product to ensure vectors are properly aligned
    double dot_product = projected_gravity.dot(AE_object.gn_);
    
    // The error should be small, indicating that the vectors are aligned
    EXPECT_NEAR(error_norm, 0.0, 0.1) << "Error vector norm is too large";
    EXPECT_NEAR(dot_product, 1.0, 0.1) << "Vectors are not properly aligned";
    
    // Expected theta (pitch) should be approximately 1 degree
    EXPECT_NEAR(euler[1], angle_rad, 0.01) << "Pitch angle not as expected";
}

// Test UpdateGravity with negative acceleration in X-axis (Y-axis rotation)
TEST_F(AHRSTest, UpdateGravityTestYAxisNegative) {
    // Initialize the attitude estimator with identity rotation matrix
    Matrix3d Rnb;
    Rnb << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;
    double dt = 0.01;      // 10ms sample time
    double Ka = 1.0;       // Full accelerometer correction for faster convergence
    AttitudeEstimator AE_object(dt, Ka, "NED");
    AE_object.Rnb_ = Rnb;
    
    // Test case: Simulate -1 degree rotation around Y-axis
    // acc = [-sin(1), 0, -cos(1)] * 9.81
    double angle_rad = 1.0 * M_PI / 180.0;  // 1 degree in radians
    std::vector<double> acc = {-9.81 * sin(angle_rad), 0, -9.81 * cos(angle_rad)};
    
    // Apply gravity update
    AE_object.UpdateGravity(acc);
    
    // Convert rotation matrix to Euler angles
    std::vector<double> euler = Rot_mat2euler(AE_object.Rnb_);
    
    // Convert acceleration to Eigen vector
    Vector3d acc_eigen(acc[0], acc[1], acc[2]);
    
    // Normalize acceleration vector to match gravity vector scale
    Vector3d acc_normalized = acc_eigen / acc_eigen.norm();
    
    // Rotated normalized acceleration vector should align with navigation gravity
    Vector3d projected_gravity = AE_object.Rnb_ * acc_normalized;
    
    // Calculate the error vector between projected gravity and navigation frame gravity
    Vector3d error_vector = projected_gravity - AE_object.gn_;
    double error_norm = error_vector.norm();
    
    // Also check dot product to ensure vectors are properly aligned
    double dot_product = projected_gravity.dot(AE_object.gn_);
    
    // The error should be small, indicating that the vectors are aligned
    EXPECT_NEAR(error_norm, 0.0, 0.1) << "Error vector norm is too large";
    EXPECT_NEAR(dot_product, 1.0, 0.1) << "Vectors are not properly aligned";
    
    // Expected theta (pitch) should be approximately -1 degree
    EXPECT_NEAR(euler[1], -1.0 * angle_rad, 0.01) << "Pitch angle not as expected";
}
