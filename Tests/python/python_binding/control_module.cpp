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
#include "../../../ControlAPI.hpp"
#include "../../../ahrs_loc_handler.hpp"
#include "../../../Utils/Controllers.hpp"
#include "../../../Utils/units.hpp"
#include "../../../Utils/Classes.hpp"

PYBIND11_MODULE(control_module, m) {
    // py::class_<MyClass>(m, "MyClass")
    //     .def(py::init<int>())
    //     .def("get_value", &MyClass::getValue)
    //     .def("set_value", &MyClass::setValue);
    pybind11::class_<StanleyController>(m, "StanleyController")
        .def(pybind11::init<double, std::vector<PreciseMeters> &,
                            std::vector<PreciseMeters> &, double>())
        .def("calc_steering_command", &StanleyController::CalcSteeringCommand)
        .def("get_delta", &StanleyController::GetDelta);
    pybind11::class_<PIDBasedLongitudinalController>(m,
        "PIDBasedLongitudinalController")
        .def(pybind11::init<
            PreciseSeconds,
            double,
            double,
            double,
            double,
            double,
            double,
            double,
            double,
            double,
            double,
            double,
            double,
            double,
            double,
            Percentage,
            Percentage,
            Percentage,
            Percentage,
            PreciseMeters,
            PreciseMeters,
            PreciseMeters,
            PreciseMeters,
            double>());
    pybind11::class_<ControlAPI>(m, "ControlAPI")
        // .def(pybind11::init<PreciseSeconds>())
        .def(pybind11::init<const std::string&,
                    const std::string&>())
        .def("MotionPlanningUpdate",
             pybind11::overload_cast<const std::vector<PreciseMeters> &,
                                     const std::vector<PreciseMeters> &,
                                     PreciseSeconds, const std::string &,
                                     PreciseSeconds>(
                 &ControlAPI::MotionPlanningUpdate))
        .def("CalculateSteeringCommand",
             pybind11::overload_cast<PreciseSeconds>(
                 &ControlAPI::CalculateSteeringCommand))
        .def("GetLongitudinalCommand",
            &ControlAPI::GetLongitudinalCommand)
        .def("CalculateLongitudinalCommand",
            &ControlAPI::CalculateLongitudinalCommand)
        .def("GetSteeringControllerStates",
             &ControlAPI::GetSteeringControllerStates)
        .def("GetLongitudinalControllerStates",
            &ControlAPI::GetLongitudinalControllerStates)
        .def("GetReferenceTrajectory", &ControlAPI::GetReferenceTrajectory)
        .def("GetDelta", &ControlAPI::GetDelta)
        .def("GetLateralError", &ControlAPI::GetLateralError)
        .def("GetHeadingError", &ControlAPI::GetHeadingError)
        .def("GetHeadingReference", &ControlAPI::GetHeadingReference)
        .def("GetSteeringCmd", &ControlAPI::GetSteeringCmd)
        .def("UpdateControlDebugStates", &ControlAPI::UpdateControlDebugStates)
        .def("TerminateControlStates", &ControlAPI::TerminateControlStates)
        .def("WriteDebugStates", &ControlAPI::WriteDebugStates);
    pybind11::class_<AHRSLocHandler>(m, "AHRSLocHandler")
        .def(pybind11::init<const std::string&,
                    const std::string&>())
        .def("UpdateIMU",
             pybind11::overload_cast<const ImuSample &, PreciseSeconds>(
                 &AHRSLocHandler::UpdateIMU))
        .def("UpdateSpeed", &AHRSLocHandler::UpdateSpeed)
        .def("UpdateSteeringWheel", &AHRSLocHandler::UpdateSteeringWheel)
        .def("UpdateHeading", &AHRSLocHandler::UpdateHeading)
        .def("ResetVehicleState", &AHRSLocHandler::ResetVehicleState)
        .def("GetPosition", &AHRSLocHandler::GetPosition)
        .def("GetVehicleHeading", &AHRSLocHandler::GetVehicleHeading);
}
