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

// namespace fs = std::filesystem;
// using namespace Eigen;
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

// vector operations
// std::vector<double> AddScalarToVector(std::vector<double> &vec, double
// scalar);
std::vector<double> AddScalarToVector(const std::vector<double> &vec,
                                      double scalar);
std::vector<double> MultiplyVectorByScalar(const std::vector<double> &vec,
                                           double scalar);
VectorXd ConvertVectorToEigen(const std::vector<double> &vec);
std::vector<double> ConverteigenToVecor(const VectorXd &vec_eigen);
std::vector<double> ConverteigenToVecor(const RowVectorXd &vec_eigen);
RowVectorXd ConvertVectorToEigenRowVector(const std::vector<double> &vec);
double VectorNorm2D(const std::vector<double> &vec2d);
std::vector<double> AbsVector(const std::vector<double> &vec);
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
// Was originally step = 1, removed by Dor as default value is already
// defined in header file
std::vector<double> Arange(double start, double stop, double step);
std::vector<double> Diff(const std::vector<double> &u);
std::vector<PreciseRadians> FoldAngles(const std::vector<PreciseRadians> &u);
PreciseRadians FoldAngles(const PreciseRadians &u);
void ContinuousAngle(const std::vector<PreciseRadians> &u, 
                           std::vector<PreciseRadians> *y);
PreciseRadians Deg2Rad(Degrees deg);
std::tuple<std::vector<double>, std::vector<double>>
SplineInterpulationEigenXBased(const std::vector<double> &input_x,
                                    const std::vector<double> &input_y,
                                    double dx);
void CalcCurveLength(const std::vector<double> &x, const std::vector<double> &y,
                     std::vector<double> *s);
void CalcCurveHeading(const std::vector<double> &x,
                      const std::vector<double> &y,
                      std::vector<PreciseRadians> *yaw);
std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
SplineInterpulationEigen(const std::vector<double> &input_x,
                            const std::vector<double> &input_y, double ds);
std::tuple<std::vector<double>, std::vector<double>>
SplineInterpulationEigen(const std::vector<double> &input_x,
                         const std::vector<double> &input_y,
                         const std::vector<double> &s_interp);
std::tuple<std::vector<double>, std::vector<double>, bool>
LinearPathInterpulation(const std::vector<double> &input_x,
                        const std::vector<double> &input_y,
                        const std::vector<double> &s_interp);
Eigen::Matrix3d Mpsi(PreciseRadians psi);
Eigen::Matrix3d AffineTransformation2D(double x, double y, double psi);
Eigen::Matrix3d InvAffineTransformation2D(Eigen::Matrix3d T);
Eigen::Matrix<double, Dynamic, 2> ProjectPoints2D(
    Matrix3d T12, Eigen::Matrix<double, Dynamic, 2> points);
double LimitValue(double u, double u_min, double u_max);
double deadzone(double u, double deadzone);
std::tuple<std::vector<double>, bool> LinearInterp(
    const std::vector<double>& x,
    const std::vector<double>& y,
    const std::vector<double>& xi
);
std::tuple<std::vector<PreciseMeters>, 
           std::vector<PreciseMeters>, 
           std::vector<PreciseRadians>> 
convert_path_control_points(
    const std::vector<PreciseMeters>& path_points_cp1_x,
    const std::vector<PreciseMeters>& path_points_cp1_y,
    PreciseMeters lr1, PreciseMeters lr2, PreciseMeters WB,
    bool convert_to_cp2_frame = false);