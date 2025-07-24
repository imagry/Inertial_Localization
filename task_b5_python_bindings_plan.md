# Task B5: Python Bindings for Localization Algorithm

## Overview
This document outlines the implementation plan for creating Python bindings for the localization algorithm components using pybind11. The goal is to make the localization functionality accessible from Python, enabling integration with the AI-driver system and supporting testing/development in Python.

## Current Status
- The repository already has a Python binding framework using pybind11
- Current bindings include both control and some localization components
- The binding code is located in `Tests/python/python_binding/control_module.cpp`
- `AHRSLocHandler` class is already partially bound with basic methods

## Implementation Plan

### 1. Update Existing Python Module

We'll use the existing `control_module.cpp` to provide the Python bindings for the localization components. The file already contains bindings for the `AHRSLocHandler` class, but we'll enhance it with additional methods needed for the localization system.

Current implementation in `control_module.cpp`:
```cpp
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
```

### 2. Essential Methods to Add or Enhance

#### 2.1 AHRSLocHandler
Enhance the existing binding with additional methods:
- `UpdateRearRightSpeed` and `UpdateRearLeftSpeed`: For wheel odometry data



### 3. Environment Setup and Build Process

The Python bindings will be built using the environment described in `Tests/python/Drive_Analysis_readme.MD`:

1. Create a Python virtual environment:
```bash
python3 -m venv vehicle_control_env
source vehicle_control_env/bin/activate
pip install -r requirements.txt
```

2. Install system dependencies:
```bash
sudo apt-get install libeigen3-dev cmake
```

3. Build the Python module using the provided rebuild script:
```bash
cd Tests/python/python_binding
./rebuild.sh
```

### 4. Integration with Existing Tests

Several example scripts already use the Python bindings:
- `Tests/python/python_binding/3_system_simulation_with_delays.py` uses `AHRSLocHandler`
- Other test scripts use `ControlAPI` which interacts with the localization system

We will ensure that these scripts continue to work and enhance them to demonstrate the new functionality.

## Conclusion
This implementation plan outlines the approach for enhancing the existing Python bindings for the localization algorithm. Instead of creating a separate module, we'll update the existing `control_module.cpp` to include comprehensive bindings for all localization components, ensuring a clean and consistent API for Python clients.
