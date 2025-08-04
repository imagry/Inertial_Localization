/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/

#pragma once

#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

// Forward declarations for Eigen types used in this file
namespace Eigen {
// Existing declarations used in this file
}

using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::RowVector3d;
using Eigen::Vector;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::pair;
using std::string;
using std::vector;

/* AHRS classes */

/**
 * @brief Class for handling IMU recording segments for attitude estimation.
 * 
 * This class processes and stores IMU data from various file formats
 * to be used for attitude estimation and analysis.
 */
class SegmentIMURecording {
 private:
    enum FileFormat { trc, ridi, trc2, aidriver};

 public:
    // Sensor data attributes
    vector<double> time_IMU_;
    vector<double> time_ori_;
    vector<double> phi_;
    vector<double> theta_;
    vector<double> psi_;
    vector<vector<double>> gyro_;
    vector<vector<double>> acc_;
    double dt_ = -1;
    int number_of_IMU_samples_;
    
    /**
     * @brief Default constructor
     */
    SegmentIMURecording() = default;
    
    /**
     * @brief Construct a new Segment IMU Recording from a file
     * 
     * @param file_path Path to the IMU data file
     */
    SegmentIMURecording(
        std::filesystem::path file_path);   // construct from file
    
    /**
     * @brief Compute the time step between IMU samples
     */
    void Compute_dt();
};

/**
 * @brief Class for estimating attitude based on IMU data.
 * 
 * This class implements algorithms to fuse gyroscope and accelerometer data
 * for robust attitude and heading estimation.
 */
class AttitudeEstimator {
 public:
    // Attitude estimation parameters and state
    double dt_;
    double phi_;
    double theta_;
    double psi_;
    double Kgx_;
    double Kgy_;
    double Kgz_;
    double g_;
    double update_time_;
    
    // Rotation and reference vectors
    Matrix3d Rnb_;
    Vector3d gn_;
    Vector3d gb_;
    Vector3d mn_;
    Vector3d mb_;
    std::string coor_sys_convention_;
    bool clock_initialized_ = false;
    bool rotation_initialized_ = false;
    
    /**
     * @brief Constructs an AttitudeEstimator with specified parameters.
     * 
     * @param dt Time step between updates in seconds
     * @param Ka Gain parameter for the accelerometer correction
     * @param coor_sys_convention Coordinate system convention ("NED" or "ENU")
     */
    AttitudeEstimator(double dt, double Ka,
                      std::string coor_sys_convention = "NED");
    
    /**
     * @brief Processes gyroscope measurements to update the rotation matrix.
     * 
     * @param gyro Vector of 3 angular velocities (x, y, z) in rad/s
     * @param clock Current timestamp in seconds
     */
    void GyroPromotion(vector<double> gyro, double clock);
    
    /**
     * @brief Updates the gravity vector estimation using accelerometer data.
     * 
     * @param acc Vector of 3 accelerations (x, y, z) in m/s^2
     */
    void UpdateGravity(vector<double> acc);
    
    /**
     * @brief Initializes the rotation matrix with given Euler angles.
     * 
     * @param phi0 Initial roll angle in radians
     * @param theta0 Initial pitch angle in radians
     * @param psi0 Initial yaw angle in radians
     */
    void InitializeRotation(double phi0, double theta0, double psi0);
    
    /**
     * @brief Runs an experiment using recorded IMU data.
     * 
     * @param exp The IMU recording segment containing data to process
     * @param file_name Path to the output CSV file
     * @param initial_heading Initial heading in radians
     */
    void Run_exp(
        SegmentIMURecording exp,
        string file_name, double initial_heading);
};

/**
 * @brief Test function for the AHRS functionality
 * 
 * @return int Status code (0 for success)
 */
int AHRS_test();

/**
 * @brief Convert a rotation matrix to Euler angles (phi, theta, psi)
 * 
 * @param Rnb Rotation matrix from body to navigation frame
 * @return vector<double> Euler angles [phi, theta, psi] in radians
 */
vector<double> Rot_mat2euler(vector<vector<double>> Rnb);

/**
 * @brief Convert an Eigen rotation matrix to Euler angles (phi, theta, psi)
 * 
 * @param R Eigen rotation matrix from body to navigation frame
 * @return vector<double> Euler angles [phi, theta, psi] in radians
 */
vector<double> Rot_mat2euler(Matrix3d R);

/**
 * @brief Create a skew-symmetric matrix from a 3D vector
 * 
 * @param Vec Input 3D vector
 * @return Matrix3d Skew-symmetric matrix
 */
Matrix3d Vec2SkewSimetric(vector<double> Vec);

/**
 * @brief Orthonormalize a rotation matrix
 * 
 * @param R Input rotation matrix
 * @return Matrix3d Orthonormalized rotation matrix
 */
Matrix3d OrthonormalizeRotationMatrix(Matrix3d R);

/**
 * @brief Apply the TRIAD algorithm to determine attitude
 * 
 * @param fb First vector in body frame
 * @param mb Second vector in body frame
 * @param fn First vector in navigation frame
 * @param mn Second vector in navigation frame
 * @return Matrix3d Rotation matrix from body to navigation frame
 */
Matrix3d TRIAD(Vector3d fb, Vector3d mb, Vector3d fn, Vector3d mn);

/**
 * @brief Construct a rotation matrix from three column vectors
 * 
 * @param col1 First column
 * @param col2 Second column
 * @param col3 Third column
 * @return Matrix3d Constructed rotation matrix
 */
Matrix3d Construct_rot_mat_from_columns(Vector3d col1, Vector3d col2,
Vector3d col3);

/**
 * @brief Convert a rotation matrix to quaternion representation
 * 
 * @param R Rotation matrix
 * @return vector<double> Quaternion [x, y, z, w]
 */
vector<double> Rot_mat2quaternion(const vector<vector<double>>& R);

/**
 * @brief Convert multiple rotation matrices to quaternions
 * 
 * @param R Array of rotation matrices
 * @return vector<vector<double>> Array of quaternions
 */
vector<vector<double>> Rot_mat2quaternion(
    const vector<vector<vector<double>>>& R);

/**
 * @brief Split an array of 2D coordinates into separate X and Y arrays
 * 
 * @param array_of_2d_coor Array of 2D coordinates
 * @return pair<vector<double>, vector<double>> Pair of X and Y arrays
 */
pair<vector<double>, vector<double>> Split_array_of_2d_coor(
    vector<vector<double>> array_of_2d_coor);

/**
 * @brief Find the index of the minimum value in a vector
 * 
 * @param vec Input vector
 * @return int Index of minimum value
 */
int Argmin(vector<double> vec);

/**
 * @brief Calculate the square of a number
 * 
 * @param x Input number
 * @return double Square of the number
 */
double Square(double x);

/**
 * @brief Calculate the Euclidean norm (2-norm) of a 2D vector
 * 
 * @param vec 2D vector
 * @return double Euclidean norm
 */
double Vector_2_norm(const vector<double>& vec);

/**
 * @brief Calculate the norms of multiple 2D vectors
 * 
 * @param nX2_array Array of 2D vectors
 * @return vector<double> Vector of norms
 */
vector<double> Norm_nX2_array(const vector<vector<double>>& nX2_array);

/**
 * @brief Convert quaternion to Euler angles
 * 
 * @param q Quaternion [w, x, y, z]
 * @return vector<double> Euler angles [roll, pitch, yaw]
 */
vector<double> Quaternion2euler(const vector<double>& q);
