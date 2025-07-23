/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/

// define classes in this file only once
// see: https://stackoverflow.com/a/37686388/2999345
#pragma once
// #include "includes.hpp"
#include <vector>
// using namespace std;
#include <filesystem>
#include <string>
#include <utility>
#include <Eigen/Dense>
// using namespace Eigen;
// Replaced directive with declarations:
using Eigen::Matrix3d;
using Eigen::Matrix2d;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector;
using Eigen::VectorXd;
using Eigen::RowVector3d;
using std::vector;
using std::pair;
using std::string;
/* AHRS classes */

class SegmentIMURecording {
 private:
    enum FileFormat { trc, ridi, trc2, aidriver};

 public:
    // attributes
    vector<double> time_IMU;
    vector<double> time_ori;
    vector<double> phi;
    vector<double> theta;
    vector<double> psi;
    vector<vector<double>> gyro;
    vector<vector<double>> acc;
    double dt = -1;
    int number_of_IMU_samples;

    SegmentIMURecording() = default;
    SegmentIMURecording(
        const vector<double>& time,
        const vector<vector<double>>& gyro,
        const vector<vector<double>>& acc,
        double dt)
        : time_IMU(time), gyro(gyro), acc(acc), dt(dt) {
    }
    SegmentIMURecording(
        const vector<double>& time,
        const vector<vector<double>>& gyro,
        const vector<vector<double>>& acc)
        : time_IMU(time), gyro(gyro), acc(acc) {
    }
    SegmentIMURecording(
        std::filesystem::path file_path);   // construct from file
    void compute_dt();
};

class AttitudeEstimator {
 public:
    // attributes
    double dt, phi, theta, psi, Kgx, Kgy, Kgz, g, update_time;
    /*vector<vector<double>> Rnb;
    vector<double> gn, gb, mn, mb;*/
    Matrix3d Rnb;
    Vector3d gn, gb, mn, mb;
    std::string coor_sys_convention_;
    bool clock_initialized_ = false;
    bool rotation_initialized_ = false;
    // methods
    /*AttitudeEstimator(double dt, vector<vector<double>> Rnb, double Ka);*/
    AttitudeEstimator(double dt, double Ka,
                                     std::string coor_sys_convention = "NED");
    void GyroPromotion(vector<double> gyro, double clock);
    void UpdateGravity(vector<double> acc);
    void InitializeRotation(double phi0, double theta0, double psi0);
    void run_exp(
        SegmentIMURecording exp,
        string file_name, double initial_heading);
};

class buffer_of_scalars {
 public:
    // att
    vector<double> data;
    int window_size;
    int accumulated_size;
    // methods
    explicit buffer_of_scalars(int window_size);
    void add_data(double new_value);
    vector<double> get_data();
    void resest();
};
class buffer_of_vectors {
 public:
    // att
    vector<vector<double>> data;
    int window_size;
    int accumulated_size;
    int number_of_columns;
    // methods
    buffer_of_vectors(int window_size, int n_clos);
    void add_data(vector<double> new_line);
    vector<vector<double>> get_data();
    void resest();
};
class buffer_of_matrices {
 public:
    // att
    vector<vector<vector<double>>> data;
    int window_size;
    int accumulated_size;
    int number_of_rows;
    int number_of_columns;
    // methods
    buffer_of_matrices(int window_size, int n_rows, int n_clos);
    void add_data(vector<vector<double>> new_mat);
    vector<vector<vector<double>>> get_data();
    void resest();
};
void buffer_test();
int ahrs_test();
vector<double> diff(vector<double> u);
double mean(vector<double> u);
vector<double> rot_mat2euler(vector<vector<double>> Rnb);
vector<double> rot_mat2euler(Matrix3d R);
Matrix3d Vec2SkewSimetric(vector<double>  Vec);
vector<double> add_scalar_to_vector(vector<double> vec, double scalar);
VectorXd convert_vector_to_eigen(const vector<double>& vec);
MatrixXd convert_matrix_to_eigen(const vector<vector<double>>& mat);
Matrix3d OrthonormalizeRotationMatrix(Matrix3d R);
Matrix3d Vec2SkewSimetric(vector<double>  Vec);
Matrix3d TRIAD(Vector3d fb, Vector3d mb, Vector3d fn, Vector3d mn);
Matrix3d construct_rot_mat_from_columns(Vector3d col1, Vector3d col2,
Vector3d col3);
vector<double> add_scalar_to_vector(vector<double> vec, double scalar);
vector<double> subtract_vectors(const vector<double> &vec1,
const vector<double> &vec2);
vector<vector<double>> subtract_matrices(const vector<vector<double>> &mat1,
const vector<vector<double>> &mat2);
vector<double> rot_mat2quaternion(const vector<vector<double>>& R);
vector<vector<double>> rot_mat2quaternion(
    const vector<vector<vector<double>>>& R);
vector<double> rot_mat2r6d(const vector<vector<double>>& R);
vector<vector<double>> rot_mat2r6d(const vector<vector<vector<double>>>& R);
bool number_in_range(double num, const vector<double>& limits);
vector<bool> numbers_in_range(const vector<vector<double>>& nums,
const vector<vector<double>>& limits);
vector<double> linspace(double start, double end, size_t size);
double weighted_average(const vector<double>& nums,
const vector<double>& weights);
vector<double> weighted_xy(const vector<vector<double>>& xy,
const vector<double>& weights);
void write_csv(std::string filename, std::vector<std::pair<std::string,
std::vector<double>>> dataset);
pair<vector<double>, vector<double>> split_array_of_2d_coor(
    vector<vector<double>> array_of_2d_coor);
int argmin(vector<double> vec);
double square(double x);
double vector_2_norm(const vector<double>& vec);
vector<double> norm_nX2_array(const vector<vector<double>>& nX2_array);
vector<double> quaternion2euler(const vector<double>& q);
