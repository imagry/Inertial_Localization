/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/

#include <stdio.h>
#include <stdlib.h>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // Include this for std::vector support
#include <string>

#include "inertial_localization_api/ahrs_loc_handler.h"
#include "inertial_localization_api/utils/classes.h"
#include "inertial_localization_api/utils/sensor_types.h"
#include "inertial_localization_api/utils/short_term_localization.h"
#include "inertial_localization_api/utils/units.h"
using inertial_localization_api::AhrsLocHandler;
using inertial_localization_api::ImuSample;
using inertial_localization_api::Vec3d;
PYBIND11_MODULE(localization_pybind_module, m) {
  // Bind supporting structures
  pybind11::class_<Vec3d>(m, "Vec3d")
      .def(pybind11::init<>())
      .def_readwrite("x", &Vec3d::x)
      .def_readwrite("y", &Vec3d::y)
      .def_readwrite("z", &Vec3d::z);

  pybind11::class_<ImuSample>(m, "ImuSample")
      .def(pybind11::init<>())
      .def_readwrite("time_stamp", &ImuSample::time_stamp)
      .def_readwrite("acc_", &ImuSample::acc)
      .def_readwrite("acc_b_", &ImuSample::acc_b)
      .def_readwrite("gyro_", &ImuSample::gyro)
      .def_readwrite("gyro_b_", &ImuSample::gyro_b)
      .def_readwrite("mag_", &ImuSample::mag)
      .def_readwrite("pitch_", &ImuSample::pitch)
      .def_readwrite("roll_", &ImuSample::roll)
      .def_readwrite("yaw_", &ImuSample::yaw);

  pybind11::class_<AhrsLocHandler>(m, "AhrsLocHandler")
      .def(pybind11::init<const std::string &, const std::string &>())
      .def("UpdateImu",
           pybind11::overload_cast<const ImuSample &, PreciseSeconds>(
               &AhrsLocHandler::UpdateImu))
      .def("UpdateSpeed", &AhrsLocHandler::UpdateSpeed)
      .def("UpdateRearRightSpeed", &AhrsLocHandler::UpdateRearRightSpeed)
      .def("UpdateRearLeftSpeed", &AhrsLocHandler::UpdateRearLeftSpeed)
      .def("UpdateSteeringWheel", &AhrsLocHandler::UpdateSteeringWheel)
      .def("UpdateHeading", &AhrsLocHandler::UpdateHeading)
      .def("UpdateVehicleState", &AhrsLocHandler::UpdateVehicleState)
      .def("GetPosition", &AhrsLocHandler::GetPosition)
      .def("GetVehicleHeading", &AhrsLocHandler::GetVehicleHeading);
}
