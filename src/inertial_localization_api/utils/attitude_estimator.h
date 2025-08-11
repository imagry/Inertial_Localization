// Copyright (c) 2024 Imagry. All Rights Reserved.
// Unauthorized copying of this file, via any medium is strictly prohibited.
// Proprietary and confidential.

#ifndef INERTIAL_LOCALIZATION_ATTITUDE_ESTIMATOR_H_
#define INERTIAL_LOCALIZATION_ATTITUDE_ESTIMATOR_H_

#include <string>
#include <vector>

#include <Eigen/Dense>

namespace inertial_localization_api {

// Estimates attitude using an AHRS algorithm.
class AttitudeEstimator {
public:
  AttitudeEstimator(double dt, double ka,
                    const std::string &coor_sys_convention = "NED");

  // Updates the attitude estimate using gyroscope data.
  void GyroPromotion(const std::vector<double> &gyro, double clock);

  // Updates the attitude estimate using accelerometer data.
  void UpdateGravity(const std::vector<double> &acc);

  // Initializes the rotation matrix from Euler angles.
  void InitializeRotation(double phi0, double theta0, double psi0);

  // Public members
  double dt_;
  double phi_;
  double theta_;
  double psi;
  double kgx_;
  double kgy_;
  double kgz_;
  double g_;
  double update_time_;
  Eigen::Matrix3d rnb_;
  Eigen::Vector3d gn_;
  Eigen::Vector3d gb_;
  Eigen::Vector3d mn_;
  Eigen::Vector3d mb_;
  std::string coor_sys_convention_;
  bool clock_initialized_ = false;
  bool rotation_initialized_ = false;
};

// Converts a rotation matrix to Euler angles.
std::vector<double> RotMat2Euler(const Eigen::Matrix3d &r);

// Creates a skew-symmetric matrix from a 3D vector.
Eigen::Matrix3d Vec2SkewSymmetric(const std::vector<double> &vec);

// Orthonormalizes a rotation matrix.
Eigen::Matrix3d OrthonormalizeRotationMatrix(const Eigen::Matrix3d &r);

// Applies the TRIAD algorithm to find the attitude.
Eigen::Matrix3d Triad(const Eigen::Vector3d &fb, const Eigen::Vector3d &mb,
                      const Eigen::Vector3d &fn, const Eigen::Vector3d &mn);

// Converts a rotation matrix to a quaternion.
std::vector<double> RotMat2Quaternion(const Eigen::Matrix3d &r);

// Converts a quaternion to Euler angles.
std::vector<double> Quaternion2Euler(const std::vector<double> &q);

} // namespace inertial_localization_api

#endif // INERTIAL_LOCALIZATION_ATTITUDE_ESTIMATOR_H_
