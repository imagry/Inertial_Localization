/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
// #include "pybind_utils/pybind11_utils.h"
#include <pybind11/stl.h>  // Include this for std::vector support
#include "../../../ahrs_loc_handler.hpp"
#include "../../../Utils/units.hpp"
#include "../../../Utils/Classes.hpp"
#include "../../../Utils/short_term_localization.hpp"

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
        .def_readwrite("acc_", &ImuSample::acc_)
        .def_readwrite("acc_b_", &ImuSample::acc_b_)
        .def_readwrite("gyro_", &ImuSample::gyro_)
        .def_readwrite("gyro_b_", &ImuSample::gyro_b_)
        .def_readwrite("mag_", &ImuSample::mag_)
        .def_readwrite("pitch_", &ImuSample::pitch_)
        .def_readwrite("roll_", &ImuSample::roll_)
        .def_readwrite("yaw_", &ImuSample::yaw_);

    pybind11::class_<AHRSLocHandler>(m, "AHRSLocHandler")
        .def(pybind11::init<const std::string&,
                    const std::string&>())
        .def("UpdateIMU",
             pybind11::overload_cast<const ImuSample &, PreciseSeconds>(
                 &AHRSLocHandler::UpdateIMU))
        .def("UpdateSpeed", &AHRSLocHandler::UpdateSpeed)
        .def("UpdateRearRightSpeed", &AHRSLocHandler::UpdateRearRightSpeed)
        .def("UpdateRearLeftSpeed", &AHRSLocHandler::UpdateRearLeftSpeed)
        .def("UpdateSteeringWheel", &AHRSLocHandler::UpdateSteeringWheel)
        .def("UpdateHeading", &AHRSLocHandler::UpdateHeading)
        .def("UpdateVehicleState", &AHRSLocHandler::UpdateVehicleState)
        .def("GetPosition", &AHRSLocHandler::GetPosition)
        .def("GetVehicleHeading", &AHRSLocHandler::GetVehicleHeading);
}
