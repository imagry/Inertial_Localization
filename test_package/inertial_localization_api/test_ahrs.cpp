#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <vector>

#include "inertial_localization_api/utils/attitude_estimator.h"
#include "inertial_localization_api/utils/sensor_types.h"
#include "inertial_localization_api/utils/units.h"
using Eigen::Matrix3d;
using Eigen::Vector3d;
using inertial_localization_api::AttitudeEstimator;
// Test fixture for AHRS tests, focusing on GyroPromotion
class AHRSTest : public ::testing::Test {
protected:
  // Set up code that will be called before each test
  void SetUp() override {
    // Create a default attitude estimator for testing with required parameters
    double dt = 0.01;               // 10ms sample time
    double Ka = 0.001;              // Standard gain value
    std::string convention = "NED"; // North-East-Down convention
    ahrs = std::make_unique<AttitudeEstimator>(dt, Ka, convention);
  }

  std::unique_ptr<AttitudeEstimator> ahrs;

  static std::vector<double> Rot_mat2euler(const Eigen::Matrix3d &R) {
    std::vector<double> eulerAngles(3);

    double phi, theta, psi;

    // The calculation for the angles depends on the value of R(2,0).
    // A singularity occurs when R(2,0) is close to +/-1, known as gimbal lock.
    if (R(2, 0) < 1.0) {
      if (R(2, 0) > -1.0) {
        // General case: not a gimbal lock situation
        theta = -asin(R(2, 0));
        phi = atan2(R(2, 1), R(2, 2));
        psi = atan2(R(1, 0), R(0, 0));
      } else {
        // Gimbal lock case, R(2,0) is close to -1
        // In this case, phi and psi are not uniquely determined.
        // We set phi to 0 and calculate psi from R(1,2) and R(1,1).
        theta = M_PI / 2.0;
        phi = 0.0;
        psi = atan2(-R(1, 2), R(1, 1));
      }
    } else {
      // Gimbal lock case, R(2,0) is close to 1
      // Similarly, phi is set to 0.
      theta = -M_PI / 2.0;
      phi = 0.0;
      psi = atan2(-R(1, 2), R(1, 1));
    }

    eulerAngles[0] = phi;   // Roll (phi)
    eulerAngles[1] = theta; // Pitch (theta)
    eulerAngles[2] = psi;   // Yaw (psi)

    return eulerAngles;
  }
};

// Empty test that always passes
TEST_F(AHRSTest, EmptyTestAlwaysPass) {
  // This test doesn't do anything but will always pass
  SUCCEED();
}

// Test that GyroPromotion correctly rotates based on gyro input for X-axis
TEST_F(AHRSTest, GyroPromotionTestXAxis) {
  // Initialize the attitude estimator with identity rotation matrix
  Matrix3d rnb_;
  rnb_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  double dt = 0.01; // 10ms sample time
  double Ka = 0.0;  // No accelerometer correction
  AttitudeEstimator AE_object(dt, Ka, "NED");
  AE_object.rnb_ = rnb_;

  // Test case: Rotation around X-axis with 100 iterations
  std::vector<double> gyro = {1, 0, 0}; // 1 rad/s around X-axis
  double clock_time = 0.0;

  // Apply gyro promotion 100 times to simulate 1 second of rotation
  for (int i = 0; i < 100; i++) {
    AE_object.GyroPromotion(gyro, clock_time);
    clock_time += dt;
  }

  // Convert rotation matrix to Euler angles
  std::vector<double> euler = Rot_mat2euler(AE_object.rnb_);

  // Store expected values
  const double expected_phi = 1.0;
  const double expected_theta = 0.0;
  const double expected_psi = 0.0;

  // Check that the euler angles match the expected values [1.0,0,0]
  // After 100 iterations of dt=0.01 with angular velocity of 1 rad/s,
  // we expect a total rotation of 1 radian around the X-axis
  EXPECT_NEAR(euler[0], expected_phi, 0.001); // phi (roll) should be 1.0
  EXPECT_NEAR(euler[1], expected_theta,
              0.001); // theta (pitch) should remain near zero
  EXPECT_NEAR(euler[2], expected_psi,
              0.001); // psi (yaw) should remain near zero
}

// Test that GyroPromotion correctly rotates based on gyro input for Y-axis
TEST_F(AHRSTest, GyroPromotionTestYAxis) {
  // Initialize the attitude estimator with identity rotation matrix
  Matrix3d rnb_;
  rnb_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  double dt = 0.01; // 10ms sample time
  double Ka = 0.0;  // No accelerometer correction
  AttitudeEstimator AE_object(dt, Ka, "NED");
  AE_object.rnb_ = rnb_;

  // Test case: Rotation around Y-axis with 100 iterations
  std::vector<double> gyro = {0, 1, 0}; // 1 rad/s around Y-axis
  double clock_time = 0.0;

  // Apply gyro promotion 100 times to simulate 1 second of rotation
  for (int i = 0; i < 100; i++) {
    AE_object.GyroPromotion(gyro, clock_time);
    clock_time += dt;
  }

  // Convert rotation matrix to Euler angles
  std::vector<double> euler = Rot_mat2euler(AE_object.rnb_);

  // Store expected values
  const double expected_phi = 0.0;
  const double expected_theta = 1.0;
  const double expected_psi = 0.0;

  // Check that the euler angles match the expected values [0,1.0,0]
  // After 100 iterations of dt=0.01 with angular velocity of 1 rad/s,
  // we expect a total rotation of 1 radian around the Y-axis
  EXPECT_NEAR(euler[0], expected_phi,
              0.001); // phi (roll) should remain near zero
  EXPECT_NEAR(euler[1], expected_theta, 0.001); // theta (pitch) should be 1.0
  EXPECT_NEAR(euler[2], expected_psi,
              0.001); // psi (yaw) should remain near zero
}

// Test that GyroPromotion correctly rotates based on gyro input for Z-axis
TEST_F(AHRSTest, GyroPromotionTestZAxis) {
  // Initialize the attitude estimator with identity rotation matrix
  Matrix3d rnb_;
  rnb_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  double dt = 0.01; // 10ms sample time
  double Ka = 0.0;  // No accelerometer correction
  AttitudeEstimator AE_object(dt, Ka, "NED");
  AE_object.rnb_ = rnb_;

  // Test case: Rotation around Z-axis with 100 iterations
  std::vector<double> gyro = {0, 0, 1}; // 1 rad/s around Z-axis
  double clock_time = 0.0;

  // Apply gyro promotion 100 times to simulate 1 second of rotation
  for (int i = 0; i < 100; i++) {
    AE_object.GyroPromotion(gyro, clock_time);
    clock_time += dt;
  }

  // Convert rotation matrix to Euler angles
  std::vector<double> euler = Rot_mat2euler(AE_object.rnb_);

  // Store expected values
  const double expected_phi = 0.0;
  const double expected_theta = 0.0;
  const double expected_psi = 1.0;

  // Check that the euler angles match the expected values [0,0,1.0]
  // After 100 iterations of dt=0.01 with angular velocity of 1 rad/s,
  // we expect a total rotation of 1 radian around the Z-axis
  EXPECT_NEAR(euler[0], expected_phi,
              0.001); // phi (roll) should remain near zero
  EXPECT_NEAR(euler[1], expected_theta,
              0.001); // theta (pitch) should remain near zero
  EXPECT_NEAR(euler[2], expected_psi, 0.001); // psi (yaw) should be 1.0
}

// Test UpdateGravity with a small angle change (1 degree around X-axis)
TEST_F(AHRSTest, UpdateGravityTestSmallAngle) {
  // Initialize the attitude estimator with identity rotation matrix
  Matrix3d rnb_;
  rnb_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  double dt = 0.01; // 10ms sample time
  double Ka = 1.0;  // Full accelerometer correction for faster convergence
  AttitudeEstimator AE_object(dt, Ka, "NED");
  AE_object.rnb_ = rnb_;

  // Test case: Simulate 1 degree rotation around X-axis
  // acc = [0, sin(1), -cos(1)] * 9.81
  double angle_rad = 1.0 * M_PI / 180.0; // 1 degree in radians
  std::vector<double> acc = {0, -9.81 * sin(angle_rad), -9.81 * cos(angle_rad)};

  // Apply gravity update
  AE_object.UpdateGravity(acc);

  // Convert rotation matrix to Euler angles
  std::vector<double> euler = Rot_mat2euler(AE_object.rnb_);

  // Convert acceleration to Eigen vector
  Vector3d acc_eigen(acc[0], acc[1], acc[2]);

  // Normalize acceleration vector to match gravity vector scale
  Vector3d acc_normalized = acc_eigen / acc_eigen.norm();

  // Rotated normalized acceleration vector should align with navigation gravity
  Vector3d projected_gravity = AE_object.rnb_ * acc_normalized;

  // Calculate the error vector between projected gravity and navigation frame
  // gravity
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
  Matrix3d rnb_;
  rnb_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  double dt = 0.01; // 10ms sample time
  double Ka = 1.0;  // Full accelerometer correction for faster convergence
  AttitudeEstimator AE_object(dt, Ka, "NED");
  AE_object.rnb_ = rnb_;

  // Test case: Simulate -1 degree rotation around X-axis
  // acc = [0, -sin(1), -cos(1)] * 9.81
  double angle_rad = 1.0 * M_PI / 180.0; // 1 degree in radians
  std::vector<double> acc = {0, 9.81 * sin(angle_rad), -9.81 * cos(angle_rad)};

  // Apply gravity update
  AE_object.UpdateGravity(acc);

  // Convert rotation matrix to Euler angles
  std::vector<double> euler = Rot_mat2euler(AE_object.rnb_);

  // Convert acceleration to Eigen vector
  Vector3d acc_eigen(acc[0], acc[1], acc[2]);

  // Normalize acceleration vector to match gravity vector scale
  Vector3d acc_normalized = acc_eigen / acc_eigen.norm();

  // Rotated normalized acceleration vector should align with navigation gravity
  Vector3d projected_gravity = AE_object.rnb_ * acc_normalized;

  // Calculate the error vector between projected gravity and navigation frame
  // gravity
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
  Matrix3d rnb_;
  rnb_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  double dt = 0.01; // 10ms sample time
  double Ka = 1.0;  // Full accelerometer correction for faster convergence
  AttitudeEstimator AE_object(dt, Ka, "NED");
  AE_object.rnb_ = rnb_;

  // Test case: Simulate 1 degree rotation around Y-axis
  // acc = [sin(1), 0, -cos(1)] * 9.81
  double angle_rad = 1.0 * M_PI / 180.0; // 1 degree in radians
  std::vector<double> acc = {9.81 * sin(angle_rad), 0, -9.81 * cos(angle_rad)};

  // Apply gravity update
  AE_object.UpdateGravity(acc);

  // Convert rotation matrix to Euler angles
  std::vector<double> euler = Rot_mat2euler(AE_object.rnb_);

  // Convert acceleration to Eigen vector
  Vector3d acc_eigen(acc[0], acc[1], acc[2]);

  // Normalize acceleration vector to match gravity vector scale
  Vector3d acc_normalized = acc_eigen / acc_eigen.norm();

  // Rotated normalized acceleration vector should align with navigation gravity
  Vector3d projected_gravity = AE_object.rnb_ * acc_normalized;

  // Calculate the error vector between projected gravity and navigation frame
  // gravity
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
  Matrix3d rnb_;
  rnb_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  double dt = 0.01; // 10ms sample time
  double Ka = 1.0;  // Full accelerometer correction for faster convergence
  AttitudeEstimator AE_object(dt, Ka, "NED");
  AE_object.rnb_ = rnb_;

  // Test case: Simulate -1 degree rotation around Y-axis
  // acc = [-sin(1), 0, -cos(1)] * 9.81
  double angle_rad = 1.0 * M_PI / 180.0; // 1 degree in radians
  std::vector<double> acc = {-9.81 * sin(angle_rad), 0, -9.81 * cos(angle_rad)};

  // Apply gravity update
  AE_object.UpdateGravity(acc);

  // Convert rotation matrix to Euler angles
  std::vector<double> euler = Rot_mat2euler(AE_object.rnb_);

  // Convert acceleration to Eigen vector
  Vector3d acc_eigen(acc[0], acc[1], acc[2]);

  // Normalize acceleration vector to match gravity vector scale
  Vector3d acc_normalized = acc_eigen / acc_eigen.norm();

  // Rotated normalized acceleration vector should align with navigation gravity
  Vector3d projected_gravity = AE_object.rnb_ * acc_normalized;

  // Calculate the error vector between projected gravity and navigation frame
  // gravity
  Vector3d error_vector = projected_gravity - AE_object.gn_;
  double error_norm = error_vector.norm();

  // Also check dot product to ensure vectors are properly aligned
  double dot_product = projected_gravity.dot(AE_object.gn_);

  // The error should be small, indicating that the vectors are aligned
  EXPECT_NEAR(error_norm, 0.0, 0.1) << "Error vector norm is too large";
  EXPECT_NEAR(dot_product, 1.0, 0.1) << "Vectors are not properly aligned";

  // Expected theta (pitch) should be approximately -1 degree
  EXPECT_NEAR(euler[1], -1.0 * angle_rad, 0.01)
      << "Pitch angle not as expected";
}
