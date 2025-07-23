/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#include "Classes.hpp"
#include "units.hpp"

struct IMU {
    PreciseSeconds imu_time_;
    Vec3d acc_;
    Vec3d b_acc_;
    Vec3d gyro_;
    Vec3d b_gyro_;
    Vec3d mag_;
    PreciseRadians pitch_;
    PreciseRadians roll_;
    PreciseRadians yaw_;
};
struct CarSensors {
    PreciseMps speed_;
    PreciseRadians steering_;
    double brake_;
    double throttle_;
};
struct GPS {
    PreciseSeconds GPS_time_;
    double latitude_;
    double longitude_;
    double height_;
    double speed_;
};
