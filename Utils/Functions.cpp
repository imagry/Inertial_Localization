/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#include <iostream>
#include <fstream>
#include <cmath>
#include <cassert>  // for assertion
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
#define assertm(exp, msg) assert(((void)msg, exp))
// #define _USE_MATH_DEFINES
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
std::vector<double> ConverteigenToVecor(const VectorXd &vec_eigen) {
    std::vector<double> vec(vec_eigen.size());
    for (int i = 0; i < vec_eigen.size(); i++) vec[i] = vec_eigen[i];
    return vec;
}
std::vector<double> ConverteigenToVecor(const RowVectorXd &vec_eigen) {
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
    assert(vec2d.size() == 2);
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
std::vector<double> Arange(double start, double stop, double step = 1) {
    std::vector<double> values;
    for (double value = start; value < stop; value += step) {
        values.push_back(value);
    }
    return values;
}
//
std::vector<double> Diff(const std::vector<double> &u) {
    std::vector<double> diff(u.size());
    std::adjacent_difference(u.begin(), u.end(), diff.begin());
    diff.erase(diff.begin());
    return diff;
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

std::tuple<std::vector<double>, std::vector<double>>
SplineInterpulationEigenXBased(const std::vector<double> &input_x,
                                    const std::vector<double> &input_y,
                                    double dx) {
    RowVectorXd input_x_eigen = ConvertVectorToEigenRowVector(input_x);
    RowVectorXd input_y_eigen = ConvertVectorToEigenRowVector(input_y);
    // Fit and generate a spline function
    double x_spane = input_x.back() - input_x[0];
    input_x_eigen = input_x_eigen / x_spane;
    const auto fit =
        SplineFitting1D::Interpolate(input_y_eigen, 2, input_x_eigen);
    Spline1D spline(fit);
    // Traverse spline
    int num_elements = round(x_spane / dx);
    RowVectorXd x_interp_eigen = RowVectorXd::LinSpaced(
        Sequential, num_elements, input_x[0], input_x.back());
    x_interp_eigen = x_interp_eigen / x_spane;
    RowVectorXd y_interp_eigen;
    y_interp_eigen.resize(x_interp_eigen.size());
    for (int idx = 0; idx < x_interp_eigen.size(); idx++) {
        y_interp_eigen(idx) = spline(x_interp_eigen(idx)).coeff(0);
    }
    x_interp_eigen = x_interp_eigen * x_spane;
    auto y_interp = ConverteigenToVecor(y_interp_eigen);
    auto x_interp = ConverteigenToVecor(x_interp_eigen);
    return {x_interp, y_interp};
}

void CalcCurveLength(const std::vector<double> &x, const std::vector<double> &y,
                     std::vector<double> *s) {
    assert(x.size() == y.size());//TODO: handle size = 0
    s->resize(x.size());
    (*s)[0] = 0.0;
    double dx;
    double dy;
    double ds;
    for (int i = 1; i < x.size(); i++) {
        dx = x[i] - x[i - 1];
        dy = y[i] - y[i - 1];
        ds = sqrt(pow(dx, 2) + pow(dy, 2));
        (*s)[i] = (*s)[i - 1] + ds;
    }
}
void CalcCurveHeading(const std::vector<double> &x,
                      const std::vector<double> &y,
                      std::vector<PreciseRadians> *yaw) {
    assert(x.size() == y.size());
    assert(x.size() > 1);
    yaw->resize(x.size() - 1);
    double dx;
    double dy;
    for (int i = 1; i < x.size(); i++) {
        dx = x[i] - x[i - 1];
        dy = y[i] - y[i - 1];
        (*yaw)[i - 1] = atan2(dy, dx);
    }
}
std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
SplineInterpulationEigen(const std::vector<double> &input_x,
                              const std::vector<double> &input_y, double ds) {
    std::vector<double> input_s;
    CalcCurveLength(input_x, input_y, &input_s);

    RowVectorXd input_s_eigen = ConvertVectorToEigenRowVector(input_s);
    RowVectorXd input_x_eigen = ConvertVectorToEigenRowVector(input_x);
    RowVectorXd input_y_eigen = ConvertVectorToEigenRowVector(input_y);
    // Fit and generate a spline function
    double s_span = input_s.back();
    input_s_eigen = input_s_eigen / s_span;
    const auto fit_x =
        SplineFitting1D::Interpolate(input_x_eigen, 2, input_s_eigen);
    const auto fit_y =
        SplineFitting1D::Interpolate(input_y_eigen, 2, input_s_eigen);
    Spline1D spline_x(fit_x);
    Spline1D spline_y(fit_y);
    // Traverse spline
    int num_elements = round(s_span / ds);
    RowVectorXd s_interp_eigen = RowVectorXd::LinSpaced(
        Sequential, num_elements, input_s[0], input_s.back());
    s_interp_eigen = s_interp_eigen / s_span;
    RowVectorXd x_interp_eigen;
    RowVectorXd y_interp_eigen;
    x_interp_eigen.resize(s_interp_eigen.size());
    y_interp_eigen.resize(s_interp_eigen.size());
    for (int idx = 0; idx < s_interp_eigen.size(); idx++) {
        x_interp_eigen(idx) = spline_x(s_interp_eigen(idx)).coeff(0);
        y_interp_eigen(idx) = spline_y(s_interp_eigen(idx)).coeff(0);
    }
    s_interp_eigen = s_interp_eigen * s_span;
    auto s_interp = ConverteigenToVecor(s_interp_eigen);
    auto x_interp = ConverteigenToVecor(x_interp_eigen);
    auto y_interp = ConverteigenToVecor(y_interp_eigen);
    return {s_interp, x_interp, y_interp};
}
std::tuple<std::vector<double>, std::vector<double>>
SplineInterpulationEigen(const std::vector<double> &input_x,
                         const std::vector<double> &input_y,
                         const std::vector<double> &s_interp) {
    const bool print_interpulation_times = false;
    double t1, t2, t3;
    std::vector<double> input_s;
    CalcCurveLength(input_x, input_y, &input_s);
    RowVectorXd input_s_eigen = ConvertVectorToEigenRowVector(input_s);
    RowVectorXd input_x_eigen = ConvertVectorToEigenRowVector(input_x);
    RowVectorXd input_y_eigen = ConvertVectorToEigenRowVector(input_y);
    // Fit and generate a spline function
    double s_span = input_s.back();
    if (s_span < s_interp.back()){
        assert("VehicleControl::utils::Functions::SplineInterpulationEigen: exterpulation !!!");
    }
    input_s_eigen = input_s_eigen / s_span;
    if (print_interpulation_times) {
        t1 = WhatsTheTimeSeconds();
    }
    const auto fit_x =
        SplineFitting1D::Interpolate(input_x_eigen, 2, input_s_eigen);
    const auto fit_y =
        SplineFitting1D::Interpolate(input_y_eigen, 2, input_s_eigen);
    Spline1D spline_x(fit_x);
    Spline1D spline_y(fit_y);
    if (print_interpulation_times){
        double t2 = WhatsTheTimeSeconds();
        std::cout << "interpulator creation time = " << std::to_string(t2 - t1) << std::endl;
    }
    // Traverse spline
    RowVectorXd s_interp_eigen = ConvertVectorToEigen(s_interp);
    s_interp_eigen = s_interp_eigen / s_span;
    RowVectorXd x_interp_eigen;
    RowVectorXd y_interp_eigen;
    x_interp_eigen.resize(s_interp_eigen.size());
    y_interp_eigen.resize(s_interp_eigen.size());
    for (int idx = 0; idx < s_interp_eigen.size(); idx++) {
        x_interp_eigen(idx) = spline_x(s_interp_eigen(idx)).coeff(0);
        y_interp_eigen(idx) = spline_y(s_interp_eigen(idx)).coeff(0);
    }
    if (print_interpulation_times){
        double t3 = WhatsTheTimeSeconds();
        std::cout << "interpulation loop time = " << std::to_string(t3 - t2) << std::endl;
    }
    auto x_interp = ConverteigenToVecor(x_interp_eigen);
    auto y_interp = ConverteigenToVecor(y_interp_eigen);
    return {x_interp, y_interp};
}
std::tuple<std::vector<double>, std::vector<double>, bool>
LinearPathInterpulation(const std::vector<double> &input_x,
                        const std::vector<double> &input_y,
                        const std::vector<double> &s_interp) {
    std::vector<double> input_s;
    CalcCurveLength(input_x, input_y, &input_s);
    auto [x_interp, success_x]  = LinearInterp(input_s, input_x, s_interp);
    if (!success_x) {
        std::cerr << "[Warning] Functions::LinearPathInterpulation: x interpolation failed" << std::endl;
        return {x_interp, {}, success_x};
    }
    auto [y_interp, success_y] = LinearInterp(input_s, input_y, s_interp);
    if (!success_y) {
        std::cerr << "[Warning] Functions::LinearPathInterpulation: y interpolation failed" << std::endl;
        return {x_interp, y_interp, success_y};
    }
    return {x_interp, y_interp, true};
}
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
    assert(points.rows() > 0);
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
double deadzone(double u, double deadzone){
    if (std::abs(u) < deadzone) {
        return 0.0;
    } else if (u > 0) {
        return u - deadzone;
    } else {
        return u + deadzone;
    }
}
std::tuple<std::vector<double>, bool> LinearInterp(
    const std::vector<double>& x,
    const std::vector<double>& y,
    const std::vector<double>& xi
) {
    std::vector<double> yi;
    bool success = false;
    if (x.size() != y.size()){
        std::cerr << "[Warning] Functions::LinearInterp: x and y must be the same size." << std::endl;
        return std::make_tuple(yi, success);
    }
    if (x.size() < 2){
        std::cerr << "[Warning] Functions::LinearInterp: x and y must contain at least 2 points." << std::endl;
        return std::make_tuple(yi, success);
    }
    // check x is monotonically increasing
    for (int i = 1; i < x.size(); i++) {
        if (x[i] <= x[i - 1]) {
            std::cerr << "[Warning] Functions::LinearInterp: x is not monotonically increasing" << std::endl;
            return std::make_tuple(yi, success); 
        }
    }
    const double epsilon = 1e-6; // Small tolerance for floating-point comparison
    for (double xq : xi) {
        if (xq < x.front() - epsilon || xq > x.back() + epsilon){
            std::cerr << "Interpolation point xq: " << xq << ", x: [";
            for (const auto& val : x) {
                std::cerr << val << " ";
            }
            std::cerr << "]" << std::endl;
            if (xq < x.front()){
                std::cerr << "x.front() - xq: " << x.front() - xq << std::endl;
            } else if (xq > x.back()){
                std::cerr << "x.back() - xq: " << x.back() - xq << std::endl;
            }
            std::cerr << "[warning] Functions::LinearInterp: Interpolation point out of bounds." << std::endl;
            return std::make_tuple(yi, success);
        }
        // Find the upper bound
        auto it = std::upper_bound(x.begin(), x.end(), xq);// iterator pointing to the first element greater than xq.
        int i = std::distance(x.begin(), it) - 1; // calculate the distance in terms of indices from the first element.
        i = std::max(0, i); // Ensure i is not negative
        i = std::min(static_cast<int>(x.size()) - 2, i); // Ensure i is not negative
        double t = (xq - x[i]) / (x[i+1] - x[i]);
        double yq = y[i] + t * (y[i+1] - y[i]);
        yi.push_back(yq);
    }
    success = true;
    return std::make_tuple(yi, success);
}
std::tuple<std::vector<PreciseMeters>, 
           std::vector<PreciseMeters>, 
           std::vector<PreciseRadians>> 
convert_path_control_points(
    const std::vector<PreciseMeters>& path_points_cp1_x,
    const std::vector<PreciseMeters>& path_points_cp1_y,
    PreciseMeters lr1, PreciseMeters lr2, PreciseMeters WB,
    bool convert_to_cp2_frame) {
    
    // Ensure the input vectors have the same size
    if (path_points_cp1_x.size() != path_points_cp1_y.size()) {
        throw std::invalid_argument("VehicleControl::Functions::convert_path_control_points: Input vectors must have the same size.");
    }
    if (lr1 <= 0 || lr1 > WB) {
        throw std::invalid_argument("VehicleControl::Functions::convert_path_control_points: input path control point invalid");
    }
    if (lr2 < 0 || lr2 > WB) {
        throw std::invalid_argument("VehicleControl::Functions::convert_path_control_points: output path control point invalid");
    }
    if (WB < 0) {
        throw std::invalid_argument("VehicleControl::Functions::convert_path_control_points: wheelbase invalid");
    }//TODO handle exceptions add success flag
    std::vector<PreciseMeters> s(path_points_cp1_x.size());
    std::vector<PreciseRadians> tangent_angles_vec(path_points_cp1_x.size());//NOTE: input path length
    // Output vectors
    std::vector<PreciseMeters> path_points_cp2_x;
    std::vector<PreciseMeters> path_points_cp2_y;
    std::vector<PreciseRadians> path_points_cp2_psi;

    CalcCurveLength(path_points_cp1_x, path_points_cp1_y, &s);
    CalcCurveHeading(path_points_cp1_x, path_points_cp1_y, &tangent_angles_vec);

    // Initialize the heading angle
    PreciseRadians psi_i = 0.0;
    path_points_cp2_psi.push_back(psi_i);

    // Static vector in the ego frame
    Eigen::Vector2d static_vector_in_ego_frame(lr2 - lr1, 0.0);

    for (size_t i = 0; i < path_points_cp1_x.size(); ++i) {//TODO: check indices
        // // Calculate the tangent angle
        // PreciseRadians tan_ang_i = (i < path_points_cp1_x.size() - 1)
        //     ? atan2(path_points_cp1_y[i + 1] - path_points_cp1_y[i],
        //             path_points_cp1_x[i + 1] - path_points_cp1_x[i])
        //     : psi_i;

        // Create the rotation matrix R
        Eigen::Matrix2d R;
        R << cos(psi_i), sin(psi_i),
            -sin(psi_i), cos(psi_i);

        // Current point in CP1 frame
        Eigen::Vector2d cp1_point(path_points_cp1_x[i], path_points_cp1_y[i]);

        // Calculate the new point in CP2 frame
        Eigen::Vector2d cp2_point = cp1_point + R.transpose() * static_vector_in_ego_frame;

        // Append the new point to the output vectors
        path_points_cp2_x.push_back(cp2_point.x());
        path_points_cp2_y.push_back(cp2_point.y());

        if (i < path_points_cp1_x.size() - 1) {
            //NOTE: calculate psi_i -> vehicle heading for the next sample

            PreciseMeters ds = s[i + 1] - s[i];

            // Calculate the slip angle and steering angle
            PreciseRadians beta_i = tangent_angles_vec[i] - psi_i;
            PreciseRadians delta_i = atan(WB / lr1 * tan(beta_i));

            // Update the heading angle
            psi_i += ds / WB * sin(delta_i);
            path_points_cp2_psi.push_back(psi_i);
        }
    }

    if (convert_to_cp2_frame) {
        for (size_t i = 0; i < path_points_cp2_x.size(); ++i) {
            path_points_cp2_x[i] -= static_vector_in_ego_frame.x();
            path_points_cp2_y[i] -= static_vector_in_ego_frame.y();
        }
    }

    return {path_points_cp2_x, path_points_cp2_y, path_points_cp2_psi};
}