// Copyright (c) 2024 Imagry. All Rights Reserved.
// Unauthorized copying of this file, via any medium is strictly prohibited.
// Proprietary and confidential.

#ifndef INERTIAL_LOCALIZATION_SENSOR_TYPES_H_
#define INERTIAL_LOCALIZATION_SENSOR_TYPES_H_

#include "inertial_localization_api/utils/units.h"

namespace inertial_localization_api {

struct Vec3d {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct ImuSample {
  double time_stamp = 0.0;
  Vec3d acc;
  Vec3d acc_b;
  Vec3d gyro;
  Vec3d gyro_b;
  Vec3d mag;
  PreciseRadians pitch = 0.0;
  PreciseRadians roll = 0.0;
  PreciseRadians yaw = 0.0;
};

struct CarSensors {
  PreciseMps speed = 0.0;
  PreciseRadians steering = 0.0;
  double brake = 0.0;
  double throttle = 0.0;
};

struct Gps {
  PreciseSeconds gps_time = 0.0;
  double latitude = 0.0;
  double longitude = 0.0;
  double height = 0.0;
  double speed = 0.0;
};

} // namespace inertial_localization_api

#endif // INERTIAL_LOCALIZATION_SENSOR_TYPES_H_
