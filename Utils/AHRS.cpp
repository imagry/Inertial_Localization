/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#include <math.h>
#include <string>
#include <utility>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <fstream>
#include <stdexcept>
#include <sstream>
#include <unsupported/Eigen/MatrixFunctions>
#define _USE_MATH_DEFINES
#include "AHRS.hpp"// NOLINT
using std::fstream;
using std::stringstream;
using std::istringstream;
using std::endl;
using std::cout;
using std::ios;
using Eigen::DiagonalMatrix;
using Eigen::AngleAxisd;
using Eigen::Quaterniond;

void SegmentIMURecording::Compute_dt() {
    std::vector<double> t_diff = Diff(time_IMU_);
    dt_ = Mean(t_diff);
}
SegmentIMURecording::SegmentIMURecording(std::filesystem::path file_path) {
    vector<string> row;
    string line, word;
    vector<vector<string>> content;
    fstream file(file_path, ios::in);
    double time_sample;

    int file_fmt, timestamp_ind;
    double time_factor, timestamp_thresh;

    int gyro_x_ind, gyro_y_ind, gyro_z_ind;
    int acc_x_ind, acc_y_ind, acc_z_ind;

    if (file.is_open()) {
        getline(file, line);    // read first line - header names
        int column_counter = 0;
        stringstream str(line);
        while (getline(str, word, ',')) {
            column_counter++;
        }

        if (column_counter == 40) {     // trc format
            std::cout << "TRC format file!" << endl;
            file_fmt = trc;

            timestamp_ind = 0;
            time_factor = 1e-3;
            timestamp_thresh = 0;

            gyro_x_ind = 1;
            gyro_y_ind = 2;
            gyro_z_ind = 3;

            acc_x_ind = 5;
            acc_y_ind = 6;
            acc_z_ind = 7;
        }   else if (column_counter == 24) {    // ridi format
            std::cout << "RIDI format file!" << endl;
            file_fmt = ridi;

            timestamp_ind = 1;
            time_factor = 1e-9;
            timestamp_thresh = -1;

            gyro_x_ind = 2;
            gyro_y_ind = 3;
            gyro_z_ind = 4;

            acc_x_ind = 5;
            acc_y_ind = 6;
            acc_z_ind = 7;
        } else if (column_counter == 48) {  // trc v2 format
            std::cout << "TRC2 format file!" << endl;
            file_fmt = trc2;

            timestamp_ind = 0;
            time_factor = 1e-3;
            timestamp_thresh = 0;

            gyro_x_ind = 1;
            gyro_y_ind = 2;
            gyro_z_ind = 3;

            acc_x_ind = 5;
            acc_y_ind = 6;
            acc_z_ind = 7;
        } else if (column_counter == 23) {    // aidriver format
            std::cout << "ai driver format file!" << endl;
            file_fmt = aidriver;

            timestamp_ind = 0;
            time_factor = 1;
            timestamp_thresh = 0;

            gyro_x_ind = 10;
            gyro_y_ind = 11;
            gyro_z_ind = 12;

            acc_x_ind = 1;
            acc_y_ind = 2;
            acc_z_ind = 3;
        } else {
            str.clear();
            str << "unknown file format with " << column_counter << " columns";
            std::cout << "unknown file format with " << column_counter
                << " columns" << endl;
            throw std::runtime_error(str.str());
        }

        while (getline(file, line)) {   // insert new line from file to line
            row.clear();
            stringstream str(line);
            while (getline(str, word, ',')) {
                row.push_back(word.c_str());
            }

            istringstream ss_2(row[timestamp_ind]);
            ss_2 >> time_sample;
            if (time_sample > timestamp_thresh) {
                this->time_ori_.push_back(time_sample * time_factor);
                if (file_fmt == trc || file_fmt == trc2) {
                    double angle_factor = -M_PI / 180;
                    this->psi_.push_back(
                        stod(row[9]) * angle_factor);   // azimuth
                    this->phi_.push_back(
                        stod(row[10]) * angle_factor);  // pitch
                    this->theta_.push_back(
                        stod(row[11]) * angle_factor);  // roll
                } else if (file_fmt == ridi) {
                    vector<double> q = { stod(row[20])/*w*/,
                    stod(row[21])/*x*/,
                    stod(row[22])/*y*/,
                    stod(row[23])/*z*/ };
                    vector<double> roll_pitch_yaw = Quaternion2euler(q);
                    this->psi_.push_back(roll_pitch_yaw[2]);     // azimuth
                    this->theta_.push_back(roll_pitch_yaw[1]);   // roll
                    this->phi_.push_back(roll_pitch_yaw[0]);     // pitch
                } else if (file_fmt == aidriver) {
                    double angle_factor = M_PI / 180;
                    this->psi_.push_back(
                        stod(row[7]) * angle_factor);   // azimuth
                    this->phi_.push_back(
                        stod(row[8]) * angle_factor);   // pitch
                    this->theta_.push_back(
                        stod(row[9]) * angle_factor);   // roll
                }
            }
            double gyro_SF;
            if (file_fmt == aidriver) {
                gyro_SF = M_PI / 180;
            } else {
                gyro_SF = 1;
            }
            if (this->time_ori_.size() > 0) {
                istringstream ss(row[timestamp_ind]);
                ss >> time_sample;
                this->time_IMU_.push_back(time_sample * time_factor);
                this->gyro_.push_back({ stod(row[gyro_x_ind]) * gyro_SF,
                                       stod(row[gyro_y_ind]) * gyro_SF,
                                       stod(row[gyro_z_ind]) * gyro_SF});
                this->acc_.push_back({ stod(row[acc_x_ind]),
                    stod(row[acc_y_ind]),
                    stod(row[acc_z_ind]) });
            }
        }
        std::cout << "Read " << this->time_IMU_.size() <<
            " lines from IMU file : " << file_path << endl;
    } else {
        std::cout << "Could not open IMU file: " << file_path << endl;
    }
    this->time_IMU_ = Add_scalar_to_vector(this->time_IMU_, -this->time_IMU_[0]);
    number_of_IMU_samples_ = static_cast<int>(time_IMU_.size());
    Compute_dt();
}
AttitudeEstimator::AttitudeEstimator(double dt, double Ka,
                                     std::string coor_sys_convention) {
    dt_ = dt;
    // std::memcpy(this->Rnb, Rnb, sizeof(double) * 9);
    Matrix3d Rnb;
    Rnb_ << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    Kgx_ = Ka;
    Kgy_ = Ka;
    Kgz_ = Ka;
    vector<double> euler = Rot_mat2euler(Rnb_);
    phi_ = euler[0];
    theta_ = euler[1];
    psi_ = euler[2];
    g_ = 9.81;
    update_time_ = 0;
    coor_sys_convention_ = coor_sys_convention;
    if (coor_sys_convention == "NED") {
        gb_ = { 0, 0, -1 };
        gn_ = { 0, 0, -1 };
        mb_ = { 1, 0, 0 };
        mn_ = { 1, 0, 0 };
    } else if (coor_sys_convention == "ENU") {
        gb_ = { 0, 0, 1 };
        gn_ = { 0, 0, 1 };
        mb_ = { 0, 1, 0 };
        mn_ = { 0, 1, 0 };
    } else {
        std::cout << "unknown coordinate system convention" << endl;
    }
}
void AttitudeEstimator::GyroPromotion(vector<double> gyro, double clock) {
    if (clock_initialized_) {
        dt_ = clock - update_time_;
    } else {
        clock_initialized_ = true;
    }
    update_time_ = clock;
    Matrix3d omega_nb_b = Vec2SkewSimetric(gyro);
    Matrix3d Rnb_dot_m = Rnb_ * omega_nb_b;
    Matrix3d Rnb_m = Rnb_ + Rnb_dot_m * dt_;
    Rnb_ = OrthonormalizeRotationMatrix(Rnb_m);
}
void AttitudeEstimator::UpdateGravity(vector<double> acc) {
    Vector3d gb_m = Rnb_.transpose() * gn_;
    Vector3d acc_eigen = Convert_vector_to_eigen(acc);
    Vector3d e = acc_eigen / g_ - gb_m;
    DiagonalMatrix<double, 3> Kg_mat(Kgx_, Kgy_, Kgz_);
    Vector3d gb_p = gb_m + Kg_mat * e;
    gb_p /= gb_p.norm();
    Vector3d mb = Rnb_.transpose() * mn_;
    Matrix3d Cnb = TRIAD(gb_p, mb / mb.norm(), gn_, mn_ / mn_.norm());
    Rnb_ = Cnb;
    gb_ = Rnb_.transpose() * gn_;
}
void AttitudeEstimator::InitializeRotation(double phi0, double theta0,
    double psi0) {
    Matrix3d R_0;
    R_0 = AngleAxisd(psi0, Vector3d::UnitZ()) * AngleAxisd(theta0,
        Vector3d::UnitY()) * AngleAxisd(phi0, Vector3d::UnitX());
    Rnb_ = R_0;
}
void AttitudeEstimator::Run_exp(SegmentIMURecording exp,
    string file_name, double initial_heading) {
    double phi_0 = exp.phi_[0];
    double theta_0 = exp.theta_[0];
    double psi_0 = initial_heading;     // exp.psi_[0];
    Matrix3d R_0;
    R_0 = AngleAxisd(psi_0, Vector3d::UnitZ()) * AngleAxisd(theta_0,
        Vector3d::UnitY()) * AngleAxisd(phi_0, Vector3d::UnitX());
    Rnb_ = R_0;

    vector<double> time_IMU;
    vector<double> phi_hat;
    vector<double> phi_e;
    vector<double> theta_hat;
    vector<double> theta_e;
    vector<double> psi_hat;
    vector<double> psi_e;
    vector<double> grv_x;
    vector<double> grv_y;
    vector<double> grv_z;
    vector<double> Rnb_11;
    vector<double> Rnb_12;
    vector<double> Rnb_13;
    vector<double> Rnb_21;
    vector<double> Rnb_22;
    vector<double> Rnb_23;
    vector<double> Rnb_31;
    vector<double> Rnb_32;
    vector<double> Rnb_33;

    for (int i = 0; i < exp.number_of_IMU_samples_; i++) {
        GyroPromotion(exp.gyro_[i], exp.time_IMU_[i]);
        UpdateGravity(exp.acc_[i]);
        vector<double> euler = Rot_mat2euler(Rnb_);
        time_IMU.push_back(exp.time_IMU_[i]);
        phi_hat.push_back(euler[0]);
        phi_e.push_back(0);
        theta_hat.push_back(euler[1]);
        theta_e.push_back(0);
        psi_hat.push_back(euler[2]);
        psi_e.push_back(0);
        grv_x.push_back(gb_(0) * g_);
        grv_y.push_back(gb_(1) * g_);
        grv_z.push_back(gb_(2) * g_);
        Rnb_11.push_back(Rnb_(0, 0));
        Rnb_12.push_back(Rnb_(0, 1));
        Rnb_13.push_back(Rnb_(0, 2));
        Rnb_21.push_back(Rnb_(1, 0));
        Rnb_22.push_back(Rnb_(1, 1));
        Rnb_23.push_back(Rnb_(1, 2));
        Rnb_31.push_back(Rnb_(2, 0));
        Rnb_32.push_back(Rnb_(2, 1));
        Rnb_33.push_back(Rnb_(2, 2));
    }
    // prepare data for export to file
    std::vector<std::pair<std::string, std::vector<double>>> vals = {
        {"time_IMU", time_IMU},
        {"phi_hat", phi_hat}, {"phi_e", phi_e}, {"theta_hat", theta_hat},
        {"theta_e", theta_e}, {"psi_hat", psi_hat}, {"psi_e", psi_e},
        {"grv_x", grv_x}, {"grv_y", grv_y}, {"grv_z", grv_z},
        {"Rnb_11", Rnb_11}, {"Rnb_12", Rnb_12}, {"Rnb_13", Rnb_13},
        {"Rnb_21", Rnb_21}, {"Rnb_22", Rnb_22}, {"Rnb_23", Rnb_23},
        {"Rnb_31", Rnb_31}, {"Rnb_32", Rnb_32}, {"Rnb_33", Rnb_33}
    };

    // Write the vector to CSV
    Write_csv(file_name, vals);
}
int Ahrs_test() {
    std::filesystem::path current_path = std::filesystem::current_path();
    cout << "Current working directory: " << current_path << endl;
    // std::filesystem::path imu_file_path = current_path / "data" /
    // "hao_leg2.csv";
    // std::filesystem::path imu_file_path = current_path / "data" /
    // "indoor_output_2023-07-03_10_35_58_edited.csv";
    // std::filesystem::path imu_file_name =
    // "indoor_output_2023-07-05_08_47_03_edited.csv";
    // std::filesystem::path imu_file_location = current_path
    // / "../data/backed_data_files";
    std::filesystem::path imu_file_name = "imu.csv";
    std::filesystem::path imu_file_location = current_path /
        "../data/backed_data_files/2025-05-21T11_52_50";
    std::filesystem::path imu_file_path = imu_file_location / imu_file_name;
    cout << "csv file path is : " << imu_file_path << endl;

    SegmentIMURecording imu_data = SegmentIMURecording(imu_file_path);
    double initial_heading_y = 0.0;
    double initial_heading_x = initial_heading_y - M_PI / 2;
    vector<double> psi_initialized = Add_scalar_to_vector(imu_data.psi_,
        -imu_data.psi_[0] + initial_heading_x);
    std::vector<std::pair<std::string, std::vector<double>>> vals = {
        {"psi", psi_initialized},
        {"phi", imu_data.phi_},
        {"theta", imu_data.theta_}
    };
    // Write the vector to CSV
    // Write_csv("data/hao_leg2-psi_phi_theta-cpp.csv", vals);
    Write_csv(current_path / "../data/temp_results"/"psi_phi_theta-cpp.csv",
        vals);
    Matrix3d Rnb;
    Rnb << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;
    // double Ka = 0.00026096;
    double Ka = 0.005;
    AttitudeEstimator AE_object = AttitudeEstimator(
        static_cast<double>(0.01), Ka, "NED");
    AE_object.Rnb_ = Rnb;
    // AE_object.Run_exp(imu_data, "CPP_AHRS_results_on_hao_leg2.csv");
    std::filesystem::path output_file_path = current_path /
        "../data/temp_results"/"CPP_AHRS_results.csv";
    AE_object.Run_exp(imu_data, output_file_path, initial_heading_x);
    // std::string system_cmd = "python ../Tests/python/plot_inputs.py --path "
    //         + output_file_path.string()
    //         + " --x time_IMU time_IMU time_IMU
    // --y phi_hat theta_hat psi_hat";
    std::string system_cmd =
        "python ../Tests/python/plot_AHRS_results.py --path_estimated "
            + output_file_path.string()
            + " --path_reference "
            + imu_file_path.string();
    system(system_cmd.c_str());
    // "time_IMU,phi_hat,phi_e,theta_hat,theta_e,psi_hat,psi_e"
    return 0;
}
BufferOfScalars::BufferOfScalars(int window_size) {
    window_size_ = window_size;
    accumulated_size_ = 0;
}
void BufferOfScalars::Add_data(double new_value) {
    data_.push_back(new_value);
    accumulated_size_ = data_.size();
}
vector<double> BufferOfScalars::Get_data() {
    vector<double> result;
    for (int i = 0; i < window_size_; i++) {
        result.push_back(data_[0]);
        data_.erase(data_.begin());
    }
    accumulated_size_ = data_.size();
    return result;
}
void BufferOfScalars::Reset() {
    for (int i = 0; i < accumulated_size_; i++) {
        data_.pop_back();
    }
    accumulated_size_ = data_.size();
}


BufferOfVectors::BufferOfVectors(int window_size, int n_cols) {
    window_size_ = window_size;
    number_of_columns_ = n_cols;
    accumulated_size_ = 0;
}
void BufferOfVectors::Add_data(vector<double> new_line) {
    if (new_line.size() != number_of_columns_) {
        throw std::invalid_argument("BufferOfVectors::Add_data: wrong number of columns");
    }
    data_.push_back(new_line);
    accumulated_size_ = data_.size();
}
vector<vector<double>> BufferOfVectors::Get_data() {
    vector<vector<double>>  result;
    for (int i = 0; i < window_size_; i++) {
        result.push_back(data_[0]);
        data_.erase(data_.begin());
    }
    accumulated_size_ = data_.size();
    return result;
}
void BufferOfVectors::Reset() {
    for (int i = 0; i < accumulated_size_; i++) {
        data_.pop_back();
    }
    accumulated_size_ = data_.size();
}

BufferOfMatrices::BufferOfMatrices(int window_size, int n_rows,
    int n_cols) {
    window_size_ = window_size;
    number_of_rows_ = n_rows;
    number_of_columns_ = n_cols;
    accumulated_size_ = 0;
}
void BufferOfMatrices::Add_data(vector<vector<double>> new_mat) {
    if (new_mat.size() != number_of_rows_) {
        throw std::invalid_argument("BufferOfMatrices::Add_data: wrong number of rows");
    }
    if (new_mat[0].size() != number_of_columns_) {
        throw std::invalid_argument("BufferOfMatrices::Add_data: wrong number of columns");
    }
    data_.push_back(new_mat);
    accumulated_size_ = data_.size();
}
vector<vector<vector<double>>> BufferOfMatrices::Get_data() {
    vector<vector<vector<double>>>  result;
    for (int i = 0; i < window_size_; i++) {
        result.push_back(data_[0]);
        data_.erase(data_.begin());
    }
    accumulated_size_ = data_.size();
    return result;
}
void BufferOfMatrices::Reset() {
    for (int i = 0; i < accumulated_size_; i++) {
        data_.pop_back();
    }
    accumulated_size_ = data_.size();
}

void Buffer_test() {
    vector<vector<double>> IMU = { {0, 0, 0}, {1, 1, 1}, {2, 2, 2},
        {3, 3, 3}, {4, 4, 4}, {5, 5, 5}, {6, 6, 6}, {7, 7, 7},
        {8, 8, 8}, {9, 9, 9} };
    int ws = 4;
    int n_cols = 3;
    BufferOfVectors buffer_obj(ws, n_cols);
    for (int i = 0; i < IMU.size(); i++) {
        buffer_obj.Add_data(IMU[i]);
        std::cout << "accumulated size is " << buffer_obj.accumulated_size_ <<
            endl;
        std::cout << "last element = " << buffer_obj.data_.back()[0] << endl;
        if (buffer_obj.accumulated_size_ >= buffer_obj.window_size_) {
            vector<vector<double>> output(buffer_obj.window_size_,
            vector<double>(buffer_obj.number_of_columns_));
            output = buffer_obj.Get_data();
            for (int i = 0; i < output.size(); i++) {
                std::cout << output[i][0] << endl;
            }
            std::cout << "accumulated size is " <<
            buffer_obj.accumulated_size_ << endl;
        }
    }
    std::cout << "accumulated size is " << buffer_obj.accumulated_size_ << endl;
    for (int i = 0; i < buffer_obj.data_.size(); i++) {
        std::cout << buffer_obj.data_[i][0] << endl;
    }
    buffer_obj.Reset();
    std::cout << "accumulated size after reset is " <<
        buffer_obj.accumulated_size_ << endl;
    for (int i = 0; i < buffer_obj.data_.size(); i++) {
        std::cout << buffer_obj.data_[i][0] << endl;
    }
}
vector<double> Diff(vector<double> u) {
    vector<double> diff(u.size());
    std::adjacent_difference(u.begin(), u.end(), diff.begin());
    diff.erase(diff.begin());
    return diff;
}

double Mean(vector<double> u) {
    double sum = accumulate(u.begin(), u.end(), 0.0);
    double m = sum / u.size();
    return m;
}

vector<double> Rot_mat2euler(vector<vector<double>> R) {
    vector<double> euler(3);

    double phi = atan2(R[2][1], R[2][2]);
    double theta = -atan(R[2][0] / sqrt(1 - pow(R[2][0], 2)));
    double psi = atan2(R[1][0], R[0][0]);
    euler[0] = phi;
    euler[1] = theta;
    euler[2] = psi;

    return euler;
}

vector<double> Rot_mat2euler(Matrix3d R) {
    vector<double> euler(3);

    double phi = atan2(R(2, 1), R(2, 2));
    double theta = -atan(R(2, 0) / sqrt(1 - pow(R(2, 0), 2)));
    double psi = atan2(R(1, 0), R(0, 0));
    euler[0] = phi;
    euler[1] = theta;
    euler[2] = psi;

    return euler;
}

Matrix3d Vec2SkewSimetric(vector<double> Vec) {
    /*vector<vector<double>> SSM = {
        {0      , -Vec[2], Vec[1] },
        {Vec[2] , 0      , -Vec[0]},
        {-Vec[1], Vec[0] , 0      }
    };*/
    Matrix3d SSM;
    SSM <<0      , -Vec[2], Vec[1],
          Vec[2] , 0      , -Vec[0],
          -Vec[1], Vec[0] , 0;
    return SSM;
}

Matrix3d TRIAD(Vector3d fb, Vector3d mb, Vector3d fn, Vector3d mn) {
    Vector3d W1 = fb / fb.norm();
    Vector3d W2 = mb / mb.norm();
    Vector3d V1 = fn;
    Vector3d V2 = mn;

    Vector3d Ou1 = W1;
    Vector3d Ou2 = W1.cross(W2) / W1.cross(W2).norm();
    Vector3d Ou3 = W1.cross(W1.cross(W2)) / W1.cross(W2).norm();

    Vector3d R1 = V1;
    Vector3d R2 = V1.cross(V2) / V1.cross(V2).norm();
    Vector3d R3 = V1.cross(V1.cross(V2)) / V1.cross(V2).norm();

    Matrix3d Mou;
    Mou << Ou1, Ou2, Ou3;
    Matrix3d Mr;
    Mr << R1, R2, R3;

    Matrix3d Cbn = Mr * Mou.transpose();

    return Cbn;
}

Matrix3d Construct_rot_mat_from_columns(Vector3d col1, Vector3d col2,
Vector3d col3) {
    Matrix3d mat_joined;
    mat_joined << col1, col2, col3;
    return mat_joined;
}

vector<double> Add_scalar_to_vector(vector<double> vec, double scalar) {
    for (double& element : vec)
        element += scalar;
    return vec;
}

VectorXd Convert_vector_to_eigen(const vector<double>& vec) {
    VectorXd vec_eigen(vec.size());
    for (int i = 0; i < vec.size(); i++)
        vec_eigen[i] = vec[i];
    return vec_eigen;
}

MatrixXd Convert_matrix_to_eigen(const vector<vector<double>>& mat) {
    int n_rows = mat.size();
    int n_cols = mat[0].size();

    MatrixXd mat_eigen(n_rows, n_cols);

    for (int i = 0; i < n_rows; i++)
        for (int j = 0; j < n_cols; j++)
            mat_eigen(i, j) = mat[i][j];
    return mat_eigen;
}

Matrix3d OrthonormalizeRotationMatrix(Matrix3d R) {
    // Rn = R.dot(np.linalg.inv(scipy.linalg.sqrtm(R.T.dot(R))))
    // # Rnb_m*(Rnb_m'*Rnb_m)^-0.5
    Matrix3d RTR = R.transpose() * R;
    Matrix3d sqrt_RTR = RTR.sqrt();
    Matrix3d inv_sqrt_RTR = sqrt_RTR.inverse();
    Matrix3d Rn = R * inv_sqrt_RTR;
    return Rn;
}

vector<double> Subtract_vectors(const vector<double> &vec1,
const vector<double> &vec2) {
    vector<double> res;
    transform(vec1.begin(), vec1.end(), vec2.begin(), back_inserter(res),
    [](double a, double b) { return a - b; });
    return res;
}

vector<vector<double>> Subtract_matrices(const vector<vector<double>> &mat1,
const vector<vector<double>> &mat2) {
    vector<vector<double>> res;
    transform(mat1.begin(), mat1.end(), mat2.begin(), back_inserter(res),
    [](vector<double> a, vector<double> b) { return Subtract_vectors(a, b); });
    return res;
}

vector<double> Rot_mat2quaternion(const vector<vector<double>>& R) {
    MatrixXd r_eigen = Convert_matrix_to_eigen(R);
    // compiler wanna make sure we're passing 3x3 matrix
    Quaterniond q_eigen(r_eigen.block<3, 3>(0, 0));
    // scipy returns x,y,z,w
    vector<double> q = { q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w() };

    return q;
}

vector<vector<double>> Rot_mat2quaternion(
    const vector<vector<vector<double>>>& R) {
    vector<vector<double>> res;
    for (vector<vector<double>> mat : R)
        res.push_back(Rot_mat2quaternion(mat));
    return res;
}

vector<double> Rot_mat2r6d(const vector<vector<double>>& R) {
    vector<double> r6 = { R[0][0], R[1][0], R[2][0], R[0][1], R[1][1],
        R[2][1], };
    return r6;
}

vector<vector<double>> Rot_mat2r6d(const vector<vector<vector<double>>>& R) {
    vector<vector<double>> res;
    for (vector<vector<double>> mat : R)
        res.push_back(Rot_mat2r6d(mat));
    return res;
}

bool Number_in_range(double num, const vector<double>& limits) {
    return (num > limits[0]) && (num < limits[1]);
}

vector<bool> Numbers_in_range(const vector<vector<double>>& nums,
const vector<vector<double>>& limits) {
    int n = nums.size();
    vector<bool> res(n);
    if (n == 0) {
        return res;
    }

    int m = nums[0].size();
    for (int i = 0; i < n; i++) {
        bool r = true;
        for (int j = 0; j < m; j++) {
            r &= Number_in_range(nums[i][j], limits[j]);
        }
        res[i] = r;
    }
    return res;
}

vector<double> Linspace(double start, double end, size_t size) {
    vector<double> vec(size);
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

double Weighted_average(const vector<double>& nums,
const vector<double>& weights) {
    int n = nums.size();
    double total = 0;
    double total_weights = 0;
    for (int i = 0; i < n; i++) {
        total += nums[i] * weights[i];
        total_weights += weights[i];
    }
    return total / total_weights;
}

vector<double> Weighted_xy(const vector<vector<double>>& xy,
const vector<double>& weights) {
    int n = xy.size();
    double x = 0;
    double y = 0;
    double total_weights = 0;
    for (int i = 0; i < n; i++) {
        x += xy[i][0] * weights[i];
        y += xy[i][1] * weights[i];
        total_weights += weights[i];
    }
    vector<double> res = { x / total_weights, y / total_weights };
    return res;
}

void Write_csv(std::string filename, std::vector<std::pair<std::string,
std::vector<double>>> dataset) {
    // Make a CSV file with one or more columns of integer values
    // Each column of data is represented by the
    // pair <column name, column data>
    //   as std::pair<std::string, std::vector<int>>
    // The dataset is represented as a vector of these columns
    // Note that all columns should be the same size

    // Create an output filestream object
    std::ofstream myFile(filename);

    // Send column names to the stream
    for (int j = 0; j < dataset.size(); ++j) {
        myFile << dataset.at(j).first;
        // No comma at end of line
        if (j != dataset.size() - 1) myFile << ",";
    }
    myFile << "\n";

    // Send data to the stream
    for (int i = 0; i < dataset.at(0).second.size(); ++i) {
        for (int j = 0; j < dataset.size(); ++j) {
            myFile << dataset.at(j).second.at(i);
            // No comma at end of line
            if (j != dataset.size() - 1) myFile << ",";
        }
        myFile << "\n";
    }

    // Close the file
    myFile.close();
}

pair<vector<double>, vector<double>> Split_array_of_2d_coor(
vector<vector<double>> array_of_2d_coor) {
    vector<double> X;
    vector<double> Y;
    for (int i = 0; i < array_of_2d_coor.size(); i++) {
        X.push_back(array_of_2d_coor[i][0]);
        Y.push_back(array_of_2d_coor[i][1]);
    }
    pair<vector<double>, vector<double>> result(X, Y);
    return result;
}

int Argmin(vector<double> vec) {
    int idx = std::min_element(vec.begin(), vec.end()) - vec.begin();
    return idx;
}

double Square(double x) {
    return x * x;
}

double Vector_2_norm(const vector<double>& vec) {
    // like np.linalg.norm(vec)
    if (vec.size() != 2) {
        throw std::invalid_argument("Vector_2_norm: invalid dimensions (must be 2)");
    }
    return sqrt(pow(vec[0], 2) + pow(vec[1], 2));
}

vector<double> Norm_nX2_array(const vector<vector<double>>& nX2_array) {
    // like np.linalg.norm(nX2_array, axis=1)
    if (nX2_array.empty() || nX2_array[0].size() != 2) {
        throw std::invalid_argument("Norm_nX2_array: invalid dimensions (must be nx2)");
    }
    vector<double> arr_norm;
    for (int i = 0; i < nX2_array.size(); i++) {
        arr_norm.push_back(Vector_2_norm(nX2_array[i]));
    }
    return arr_norm;
}

vector<double> Quaternion2euler(const vector<double>& q) {
    if (q.size() != 4) {
        throw std::invalid_argument("Quaternion2euler: quaternion must have 4 elements");
    }
    double w = q[0];
    double x = q[1];
    double y = q[2];
    double z = q[3];
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    double roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = sqrt(1 + 2 * (w * y - x * z));
    double cosp = sqrt(1 - 2 * (w * y - x * z));
    double pitch = 2 * atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = atan2(siny_cosp, cosy_cosp);

    vector<double> roll_pitch_yaw = { roll, pitch, yaw };
    return roll_pitch_yaw;
}
