/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#pragma once
#include <vector>
#include <string>
#include <filesystem>
#include <tuple>
#include <Eigen/Dense>
#include "units.hpp"

// Replaced directive with declarations:
using Eigen::Matrix3d;
using Eigen::Matrix2d;
using Eigen::Matrix;
using Eigen::Vector2d;
using Eigen::RowVector3d;
using Eigen::RowVectorXd;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Dynamic;
using std::vector;

//==============================================================================
// Vector Operations
//==============================================================================

/**
 * @brief Adds a scalar value to each element of a vector.
 * 
 * @param vec Input vector
 * @param scalar Value to add to each element
 * @return std::vector<double> New vector with scalar added to each element
 */
// scalar);
std::vector<double> AddScalarToVector(const std::vector<double> &vec,
                                      double scalar);

/**
 * @brief Multiplies each element of a vector by a scalar.
 * 
 * @param vec Input vector
 * @param scalar Value to multiply each element by
 * @return std::vector<double> New vector with elements multiplied by scalar
 */
std::vector<double> MultiplyVectorByScalar(const std::vector<double> &vec,
                                           double scalar);

/**
 * @brief Converts a standard vector to an Eigen vector.
 * 
 * @param vec Standard vector to convert
 * @return VectorXd Eigen vector containing the same elements
 */
VectorXd ConvertVectorToEigen(const std::vector<double> &vec);

/**
 * @brief Converts an Eigen vector to a standard vector.
 * 
 * @param vec_eigen Eigen vector to convert
 * @return std::vector<double> Standard vector containing the same elements
 */
std::vector<double> ConvertEigenToVector(const VectorXd &vec_eigen);

/**
 * @brief Converts an Eigen row vector to a standard vector.
 * 
 * @param vec_eigen Eigen row vector to convert
 * @return std::vector<double> Standard vector containing the same elements
 */
std::vector<double> ConvertEigenToVector(const RowVectorXd &vec_eigen);

/**
 * @brief Converts a standard vector to an Eigen row vector.
 * 
 * @param vec Standard vector to convert
 * @return RowVectorXd Eigen row vector containing the same elements
 */
RowVectorXd ConvertVectorToEigenRowVector(const std::vector<double> &vec);

/**
 * @brief Calculates the Euclidean norm (magnitude) of a 2D vector.
 * 
 * @param vec2d Input 2D vector
 * @return double Euclidean norm of the vector
 */
double VectorNorm2D(const std::vector<double> &vec2d);

/**
 * @brief Computes the absolute value of each element in a vector.
 * 
 * @param vec Input vector
 * @return std::vector<double> Vector with absolute values
 */
std::vector<double> AbsVector(const std::vector<double> &vec);

/**
 * @brief Creates a sequence of evenly spaced values within a specified range.
 * 
 * @tparam T_arrange Type of values in the range (must support addition)
 * @param start First value in the sequence
 * @param stop Value to stop before (exclusive)
 * @param step Spacing between values
 * @return std::vector<T_arrange> Vector containing the sequence
 */
template <typename T_arrange>
std::vector<T_arrange> Arange(
    T_arrange start,
    T_arrange stop,
    T_arrange step = 1) {
    std::vector<T_arrange> values;
    for (T_arrange value = start; value < stop; value += step) {
        values.push_back(value);
    }
    return values;
}
/**
 * @brief Creates a sequence of evenly spaced double values (convenience overload).
 * 
 * @param start First value in the sequence
 * @param stop Value to stop before (exclusive)
 * @param step Spacing between values
 * @return std::vector<double> Vector containing the sequence
 */
std::vector<double> Arange(double start, double stop, double step);

/**
 * @brief Computes the differences between consecutive elements of a vector.
 * 
 * @param u Input vector
 * @return std::vector<double> Vector of differences with size = u.size() - 1
 */
std::vector<double> Diff(const std::vector<double> &u);

//==============================================================================
// Angle Operations
//==============================================================================

/**
 * @brief Folds angles to the range [-π, π] for a vector of angles.
 * 
 * @param u Vector of angles in radians
 * @return std::vector<PreciseRadians> Vector of folded angles
 */
std::vector<PreciseRadians> FoldAngles(const std::vector<PreciseRadians> &u);

/**
 * @brief Folds a single angle to the range [-π, π].
 * 
 * @param u Angle in radians
 * @return PreciseRadians Folded angle
 */
PreciseRadians FoldAngles(const PreciseRadians &u);

/**
 * @brief Converts a sequence of angles to a continuous representation.
 * 
 * Ensures that angles don't jump between -π and π by adding or subtracting
 * 2π as needed to maintain continuity.
 * 
 * @param u Input vector of angles in radians
 * @param y Output vector of continuous angles
 */
void ContinuousAngle(const std::vector<PreciseRadians> &u, 
                     std::vector<PreciseRadians> *y);

/**
 * @brief Converts degrees to radians.
 * 
 * @param deg Angle in degrees
 * @return PreciseRadians Angle in radians
 */
PreciseRadians Deg2Rad(Degrees deg);
//==============================================================================
// Matrix and Transformation Operations
//==============================================================================

/**
 * @brief Creates a 2D rotation matrix from a yaw angle.
 * 
 * @param psi Yaw angle in radians
 * @return Eigen::Matrix3d 3x3 rotation matrix
 */
Eigen::Matrix3d Mpsi(PreciseRadians psi);

/**
 * @brief Creates a 2D affine transformation matrix from position and orientation.
 * 
 * @param x X position
 * @param y Y position
 * @param psi Yaw angle in radians
 * @return Eigen::Matrix3d 3x3 affine transformation matrix
 */
Eigen::Matrix3d AffineTransformation2D(double x, double y, double psi);

/**
 * @brief Computes the inverse of a 2D affine transformation matrix.
 * 
 * @param T Original transformation matrix
 * @return Eigen::Matrix3d Inverse transformation matrix
 */
Eigen::Matrix3d InvAffineTransformation2D(Eigen::Matrix3d T);

/**
 * @brief Projects a set of 2D points using a transformation matrix.
 * 
 * @param T12 Transformation matrix
 * @param points Matrix of points to transform
 * @return Eigen::Matrix<double, Dynamic, 2> Transformed points
 */
Eigen::Matrix<double, Dynamic, 2> ProjectPoints2D(
    Matrix3d T12, Eigen::Matrix<double, Dynamic, 2> points);

/**
 * @brief Limits a value to within a specified range.
 * 
 * @param u Input value
 * @param u_min Minimum allowed value
 * @param u_max Maximum allowed value
 * @return double Limited value
 */
double LimitValue(double u, double u_min, double u_max);