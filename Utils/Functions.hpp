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
 * @brief Converts a standard matrix to an Eigen matrix.
 * 
 * @param mat Standard matrix (vector of vectors) to convert
 * @return MatrixXd Eigen matrix containing the same elements
 */
MatrixXd ConvertMatrixToEigen(const std::vector<std::vector<double>> &mat);

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
 * @brief Subtracts the second vector from the first vector element-wise.
 * 
 * @param vec1 First vector
 * @param vec2 Second vector to subtract
 * @return std::vector<double> Result of subtraction (vec1 - vec2)
 */
std::vector<double> SubtractVectors(const std::vector<double> &vec1, const std::vector<double> &vec2);

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
std::vector<T_arrange> Arange(T_arrange start, T_arrange stop, T_arrange step = 1) {
    std::vector<T_arrange> result;
    for (T_arrange i = start; i < stop; i += step) {
        result.push_back(i);
    }
    return result;
}

/**
 * @brief Creates a linearly spaced vector of specified size between start and end values.
 * 
 * @param start First value in the sequence
 * @param end Last value in the sequence
 * @param size Number of elements in the sequence
 * @return std::vector<double> Vector containing the linearly spaced sequence
 */
std::vector<double> Linspace(double start, double end, size_t size);

/**
 * @brief Checks if a number is within a specified range.
 * 
 * @param num Number to check
 * @param limits Vector containing [min, max] range limits
 * @return bool True if the number is within the range, false otherwise
 */
bool NumberInRange(double num, const std::vector<double>& limits);

/**
 * @brief Checks if multiple vectors are within specified ranges.
 * 
 * @param nums 2D vector of values to check
 * @param limits 2D vector of range limits
 * @return std::vector<bool> Vector of boolean results for each input vector
 */
std::vector<bool> NumbersInRange(const std::vector<std::vector<double>>& nums,
                                const std::vector<std::vector<double>>& limits);

/**
 * @brief Calculates a weighted average of values.
 * 
 * @param nums Vector of values to average
 * @param weights Vector of weights corresponding to each value
 * @return double The weighted average
 */
double WeightedAverage(const std::vector<double>& nums,
                       const std::vector<double>& weights);

/**
 * @brief Calculates a weighted average of 2D coordinates.
 * 
 * @param xy 2D vector of coordinates (each inner vector is [x, y])
 * @param weights Vector of weights corresponding to each coordinate
 * @return std::vector<double> The weighted average coordinate [x, y]
 */
std::vector<double> WeightedXY(const std::vector<std::vector<double>>& xy,
                              const std::vector<double>& weights);

/**
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

/**
 * @brief Calculate the mean value of a vector
 * 
 * @param u Input vector
 * @return double Mean value
 */
double Mean(const std::vector<double> &u);

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

/**
 * @brief Find the index of the minimum value in a vector
 * 
 * @param vec Input vector
 * @return int Index of minimum value
 */
int Argmin(std::vector<double> vec);