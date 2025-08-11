// Copyright (c) 2024 Imagry. All Rights Reserved.
// Unauthorized copying of this file, via any medium is strictly prohibited.
// Proprietary and confidential.

#include "inertial_localization_api/utils/attitude_estimator.h"

#include <cmath>
#include <iostream>
#include <string>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>

namespace inertial_localization_api {

AttitudeEstimator::AttitudeEstimator(double dt, double ka,
                                     const std::string &coor_sys_convention)
    : dt_(dt), phi_(0.0), theta_(0.0), psi(0.0), kgx_(ka), kgy_(ka), kgz_(ka),
      g_(9.81), update_time_(0.0), coor_sys_convention_(coor_sys_convention) {
  rnb_ = Eigen::Matrix3d::Identity();
  if (coor_sys_convention == "NED") {
    gb_ << 0, 0, -1;
    gn_ << 0, 0, -1;
    mb_ << 1, 0, 0;
    mn_ << 1, 0, 0;
  } else if (coor_sys_convention == "ENU") {
    gb_ << 0, 0, 1;
    gn_ << 0, 0, 1;
    mb_ << 0, 1, 0;
    mn_ << 0, 1, 0;
  } else {
    std::cerr << "Unknown coordinate system convention: " << coor_sys_convention
              << std::endl;
  }
}

void AttitudeEstimator::GyroPromotion(const std::vector<double> &gyro,
                                      double clock) {
  if (clock_initialized_) {
    dt_ = clock - update_time_;
  }
  update_time_ = clock;
  clock_initialized_ = true;

  Eigen::Matrix3d omega_nb_b = Vec2SkewSymmetric(gyro);
  Eigen::Matrix3d rnb_dot_m = rnb_ * omega_nb_b;
  Eigen::Matrix3d rnb_m = rnb_ + rnb_dot_m * dt_;
  rnb_ = OrthonormalizeRotationMatrix(rnb_m);
}

void AttitudeEstimator::UpdateGravity(const std::vector<double> &acc) {
  Eigen::Vector3d gb_m = rnb_.transpose() * gn_;
  Eigen::Vector3d acc_eigen(acc.data());
  Eigen::Vector3d e = acc_eigen.normalized() - gb_m;
  Eigen::DiagonalMatrix<double, 3> kg_mat(kgx_, kgy_, kgz_);
  Eigen::Vector3d gb_p = gb_m + kg_mat * e;
  gb_p.normalize();
  Eigen::Vector3d mb = rnb_.transpose() * mn_;
  Eigen::Matrix3d cnb = Triad(gb_p, mb.normalized(), gn_, mn_.normalized());
  rnb_ = cnb;
  gb_ = rnb_.transpose() * gn_;
}

void AttitudeEstimator::InitializeRotation(double phi0, double theta0,
                                           double psi0) {
  rnb_ = Eigen::AngleAxisd(psi0, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(theta0, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(phi0, Eigen::Vector3d::UnitX());
}

std::vector<double> RotMat2Euler(const Eigen::Matrix3d &r) {
  double phi = atan2(r(2, 1), r(2, 2));
  double theta = -asin(r(2, 0));
  double psi = atan2(r(1, 0), r(0, 0));
  return {phi, theta, psi};
}

Eigen::Matrix3d Vec2SkewSymmetric(const std::vector<double> &vec) {
  Eigen::Matrix3d ssm;
  ssm << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;
  return ssm;
}

Eigen::Matrix3d OrthonormalizeRotationMatrix(const Eigen::Matrix3d &r) {
  return r * (r.transpose() * r).inverse().sqrt();
}

Eigen::Matrix3d Triad(const Eigen::Vector3d &fb, const Eigen::Vector3d &mb,
                      const Eigen::Vector3d &fn, const Eigen::Vector3d &mn) {
  Eigen::Vector3d ou1 = fb.normalized();
  Eigen::Vector3d ou2 = (fb.cross(mb)).normalized();
  Eigen::Vector3d ou3 = (fb.cross(fb.cross(mb))).normalized();

  Eigen::Vector3d r1 = fn;
  Eigen::Vector3d r2 = (fn.cross(mn)).normalized();
  Eigen::Vector3d r3 = (fn.cross(fn.cross(mn))).normalized();

  Eigen::Matrix3d mou;
  mou << ou1, ou2, ou3;
  Eigen::Matrix3d mr;
  mr << r1, r2, r3;

  return mr * mou.transpose();
}

std::vector<double> RotMat2Quaternion(const Eigen::Matrix3d &r) {
  Eigen::Quaterniond q(r);
  return {q.x(), q.y(), q.z(), q.w()};
}

std::vector<double> Quaternion2Euler(const std::vector<double> &q) {
  if (q.size() != 4) {
    throw std::invalid_argument("Quaternion must have 4 elements");
  }
  double w = q[3];
  double x = q[0];
  double y = q[1];
  double z = q[2];

  double sinr_cosp = 2 * (w * x + y * z);
  double cosr_cosp = 1 - 2 * (x * x + y * y);
  double roll = atan2(sinr_cosp, cosr_cosp);

  double sinp = 2 * (w * y - z * x);
  double pitch;
  if (std::abs(sinp) >= 1) {
    pitch = std::copysign(M_PI / 2, sinp);
  } else {
    pitch = asin(sinp);
  }

  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  double yaw = atan2(siny_cosp, cosy_cosp);

  return {roll, pitch, yaw};
}

} // namespace inertial_localization_api
