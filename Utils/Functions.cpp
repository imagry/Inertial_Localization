/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#include <iostream>
#include <fstream>
#include <cmath>
#include <filesystem>
#include <algorithm>
#include <vector>
#include <numeric>
#include <tuple>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <unsupported/Eigen/Splines>
#include <stdexcept>
#include "units.hpp"// NOLINT
#include "Functions.hpp"// NOLINT
#include "DataHandling.hpp"// NOLINT


using Eigen::Spline;
using Eigen::SplineFitting;
using Eigen::Matrix3d;
using Eigen::Matrix2d;
using Eigen::Matrix;
using Eigen::Vector2d;
using Eigen::RowVector3d;
using Eigen::Sequential;
using std::vector;
typedef Eigen::Spline<double, 1, 2> Spline1D;
typedef Eigen::SplineFitting<Spline1D> SplineFitting1D;

// vector operations
// std::vector<double> AddScalarToVector(std::vector<double> &vec, double
// scalar) {
//     for (double& element : vec)
//         element += scalar;
//     return vec;
// }
std::vector<double> AddScalarToVector(const std::vector<double> &vec,
                                      double scalar) {
    std::vector<double> result;
    for (int i = 0; i < vec.size(); i++) {
        result.push_back(vec[i] + scalar);
    }
    return result;
}
std::vector<double> MultiplyVectorByScalar(const std::vector<double> &vec,
                                           double scalar) {
    std::vector<double> result;
    for (int i = 0; i < vec.size(); i++) {
        result.push_back(vec[i] * scalar);
    }
    return result;
}
VectorXd ConvertVectorToEigen(const std::vector<double> &vec) {
    VectorXd vec_eigen(vec.size());
    for (int i = 0; i < vec.size(); i++) vec_eigen[i] = vec[i];
    return vec_eigen;
}

MatrixXd ConvertMatrixToEigen(const std::vector<std::vector<double>> &mat) {
    int n_rows = mat.size();
    int n_cols = mat[0].size();

    MatrixXd mat_eigen(n_rows, n_cols);

    for (int i = 0; i < n_rows; i++)
        for (int j = 0; j < n_cols; j++)
            mat_eigen(i, j) = mat[i][j];
    return mat_eigen;
}

std::vector<double> ConvertEigenToVector(const VectorXd &vec_eigen) {
    std::vector<double> vec(vec_eigen.size());
    for (int i = 0; i < vec_eigen.size(); i++) vec[i] = vec_eigen[i];
    return vec;
}
std::vector<double> ConvertEigenToVector(const RowVectorXd &vec_eigen) {
    std::vector<double> vec(vec_eigen.size());
    for (int i = 0; i < vec_eigen.size(); i++) vec[i] = vec_eigen[i];
    return vec;
}
RowVectorXd ConvertVectorToEigenRowVector(const std::vector<double> &vec) {
    RowVectorXd vec_eigen(vec.size());
    for (int i = 0; i < vec.size(); i++) {
        vec_eigen[i] = vec[i];
    }
    return vec_eigen;
}
double VectorNorm2D(const std::vector<double> &vec2d) {
    if (vec2d.size() != 2) {
        throw std::invalid_argument("VectorNorm2D: Input vector must have exactly 2 elements");
    }
    double vec_norm = sqrt(pow(vec2d[0], 2) + pow(vec2d[1], 2));
    return vec_norm;
}
std::vector<double> AbsVector(const std::vector<double> &vec) {
    std::vector<double> result;
    for (int i = 0; i < vec.size(); i++) {
        result.push_back(abs(vec[i]));
    }
    return result;
}

std::vector<double> SubtractVectors(const std::vector<double> &vec1, const std::vector<double> &vec2) {
    std::vector<double> res;
    transform(vec1.begin(), vec1.end(), vec2.begin(), back_inserter(res),
    [](double a, double b) { return a - b; });
    return res;
}

std::vector<double> Arange(double start, double stop, double step = 1) {
    std::vector<double> values;
    for (double value = start; value < stop; value += step) {
        values.push_back(value);
    }
    return values;
}

std::vector<double> Linspace(double start, double end, size_t size) {
    std::vector<double> vec(size);
    vec[0] = start;     // exactly start
    vec[size - 1] = end;    // exactly end

    if (size > 2) {
        double delta = (end - start) / (size - 1);
        for (int i = 1; i < size - 1; i++) {
            vec[i] = start + delta * i;
        }
    }
    return vec;
}

bool NumberInRange(double num, const std::vector<double>& limits) {
    return (num > limits[0]) && (num < limits[1]);
}

std::vector<bool> NumbersInRange(const std::vector<std::vector<double>>& nums,
const std::vector<std::vector<double>>& limits) {
    int n = nums.size();
    std::vector<bool> res(n);
    if (n == 0) {
        return res;
    }

    int m = nums[0].size();
    for (int i = 0; i < n; i++) {
        bool r = true;
        for (int j = 0; j < m; j++) {
            r &= NumberInRange(nums[i][j], limits[j]);
        }
        res[i] = r;
    }
    return res;
}

double WeightedAverage(const std::vector<double>& nums,
const std::vector<double>& weights) {
    int n = nums.size();
    double total = 0;
    double total_weights = 0;
    for (int i = 0; i < n; i++) {
        total += nums[i] * weights[i];
        total_weights += weights[i];
    }
    return total / total_weights;
}

std::vector<double> WeightedXY(const std::vector<std::vector<double>>& xy,
const std::vector<double>& weights) {
    int n = xy.size();
    double x = 0;
    double y = 0;
    double total_weights = 0;
    for (int i = 0; i < n; i++) {
        x += xy[i][0] * weights[i];
        y += xy[i][1] * weights[i];
        total_weights += weights[i];
    }
    std::vector<double> res = { x / total_weights, y / total_weights };
    return res;
}

//
std::vector<double> Diff(const std::vector<double> &u) {
    std::vector<double> diff(u.size());
    std::adjacent_difference(u.begin(), u.end(), diff.begin());
    diff.erase(diff.begin());
    return diff;
}

double Mean(const std::vector<double> &u) {
    double sum = std::accumulate(u.begin(), u.end(), 0.0);
    double m = sum / u.size();
    return m;
}

std::vector<PreciseRadians> FoldAngles(const std::vector<PreciseRadians> &u) {
    // angles are folded to +-pi
    int num_of_samples = u.size();
    std::vector<double> y;
    for (int i = 0; i < num_of_samples; i++) {
        double angle = fmod(u[i], 2 * M_PI);
        if (angle > M_PI) {
            angle -= 2 * M_PI;
        } else if (angle < -M_PI) {
            angle += 2 * M_PI;
        }
        y.push_back(angle);
    }
    return y;
}
PreciseRadians FoldAngles(const PreciseRadians &u) {
    // angle is folded to +-pi
    double angle = fmod(u, 2 * M_PI);
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}
void ContinuousAngle(const std::vector<PreciseRadians> &u, 
                     std::vector<PreciseRadians> *y){
    int n = u.size();
    (*y).push_back(u[0]);
    int counter = 0;
    for (int i = 1; i < n; i++) {
        if (u[i] - u[i-1] > M_PI) {
            counter -= 1;
        } else if (u[i] - u[i-1] < -M_PI) {
            counter += 1;
        } 
        (*y).push_back(u[i] + counter * 2 * M_PI);
    } 
                           
}
PreciseRadians Deg2Rad(Degrees deg) { return deg * M_PI / 180.0; }
Eigen::Matrix3d Mpsi(PreciseRadians psi) {
    Eigen::Matrix3d Mpsi;
    Mpsi << std::cos(psi), std::sin(psi), 0, -std::sin(psi), std::cos(psi), 0,
        0, 0, 1;
    return Mpsi;
}
Eigen::Matrix3d AffineTransformation2D(double x, double y, double psi) {
    /*
    calculate an affine transformation matrix which projects 2D points in CS1
    in to CS2.
    x,y are the coordinates of the origin of CS2 in CS1
    psi is the angle of Cs1 relative to CS2
    */
    Eigen::Matrix2d R = Mpsi(psi).block<2, 2>(0, 0);
    Eigen::Vector2d O2(x, y);
    Eigen::RowVector3d third_row(0, 0, 1);

    Eigen::Matrix3d T;
    T.block<2, 2>(0, 0) = R;
    T.block<2, 1>(0, 2) = -R * O2;
    T.row(2) = third_row;

    return T;
}
Eigen::Matrix3d InvAffineTransformation2D(Eigen::Matrix3d T) {
    Eigen::Matrix2d R = T.block<2, 2>(0, 0);
    Eigen::Vector2d O1 = T.block<2, 1>(0, 2);
    Eigen::Matrix3d T_inv;
    T_inv.block<2, 2>(0, 0) = R.transpose();
    T_inv.block<2, 1>(0, 2) = -R.transpose() * O1;
    T_inv.row(2) << 0, 0, 1;
    return T_inv;
}
Eigen::Matrix<double, Dynamic, 2> ProjectPoints2D(
    Matrix3d T12, Eigen::Matrix<double, Dynamic, 2> points) {
    /*
    project multiple points in CS1 on a CS2
    T12 is the affine transformation matrix from CS1 to CS2
    points is an nX2 array.
    */
    if (points.rows() <= 0) {
        throw std::invalid_argument("ProjectPoints2D: Input points matrix must have at least one row");
    }
    int num_points = points.rows();
    Eigen::Matrix<double, 3, Eigen::Dynamic> points_homo;
    //    points_homo = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3,
    //    num_points);
    points_homo = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, num_points);
    points_homo.topRows<2>() = points.transpose();
    points_homo.row(2) =
        Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(1, num_points);
    Eigen::Matrix<double, 3, Dynamic> points_projected_homo = T12 * points_homo;
    Eigen::Matrix<double, Dynamic, 2> points_projected =
        points_projected_homo.topRows<2>().transpose();
    return points_projected;
}
double LimitValue(double u, double u_min, double u_max) {
    double y = std::max(u, u_min);
    y = std::min(y, u_max);
    return y;
}

int Argmin(std::vector<double> vec) {
    int idx = std::min_element(vec.begin(), vec.end()) - vec.begin();
    return idx;
}

double Vector_2_norm(const std::vector<double>& vec) {
    // like np.linalg.norm(vec)
    if (vec.size() != 2) {
        throw std::invalid_argument("Vector_2_norm: invalid dimensions (must be 2)");
    }
    return sqrt(pow(vec[0], 2) + pow(vec[1], 2));
}

std::vector<double> Norm_nX2_array(const std::vector<std::vector<double>>& nX2_array) {
    // like np.linalg.norm(nX2_array, axis=1)
    if (nX2_array.empty() || nX2_array[0].size() != 2) {
        throw std::invalid_argument("Norm_nX2_array: invalid dimensions (must be nx2)");
    }
    std::vector<double> arr_norm;
    for (int i = 0; i < nX2_array.size(); i++) {
        arr_norm.push_back(Vector_2_norm(nX2_array[i]));
    }
    return arr_norm;
}

double VectorNorm3D(const Vec3d& vec) {
    return sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2));
}

double StandardDeviation(const std::vector<double>& vec) {
    if (vec.size() < 2) {
        return 0.0;
    }
    Eigen::VectorXd eigen_vec = ConvertVectorToEigen(vec);
    double mean = eigen_vec.mean();
    double sq_sum = (eigen_vec.array() - mean).square().sum();
    return std::sqrt(sq_sum / (eigen_vec.size() - 1));
}
