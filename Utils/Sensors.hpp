/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#pragma once

#include "Classes.hpp"
#include "units.hpp"

struct Vec3d {
    double x;
    double y;
    double z;
};

struct ImuSample {
    double time_stamp;
    Vec3d acc_;
    Vec3d acc_b_;
    Vec3d gyro_;
    Vec3d gyro_b_;
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
    
    // Default constructor
    CarSensors() : speed_(0.0), steering_(0.0), brake_(0.0), throttle_(0.0) {}
};

struct GPS {
    PreciseSeconds gps_time_;  // Changed from GPS_time_ for consistency
    double latitude_;
    double longitude_;
    double height_;
    double speed_;
    
    // Default constructor
    GPS() : gps_time_(0.0), latitude_(0.0), longitude_(0.0), height_(0.0), speed_(0.0) {}
};
