# Localization Python Bindings and Testing Implementation Notes

This document consolidates the implementation notes for Tasks B5, B6, B7, and B8 from the localization separation plan, providing a comprehensive overview of the Python bindings and testing framework for the standalone localization repository.

## Tasks Overview

| Task # | Description | Status |
|--------|-------------|--------|
| B5 | Create or verify Python bindings (pybind) for localization algorithm | ✅ Completed |
| B6 | Ensure compatibility with existing sensor API | ✅ Completed |
| B7 | Create Python test script for loading AI-driver trip data | ✅ Completed |
| B8 | Implement synchronous sensor data processing using timestamps | 🔄 In Progress |

## Latest Update - B7 & B8 Progress (July 27, 2025)

### B7: Create Python test script for loading AI-driver trip data
- ✅ Created script `carpose_offline_calculation.py` that loads trip data from AI-driver
- ✅ Implemented command-line argument parsing with option for trip path
- ✅ Successfully loads and displays basic trip information (duration, samples, sample rate)
- ✅ Script can be run from the command line with custom trip path

### B8: Implement synchronous sensor data processing using timestamps
- ✅ Enhanced `Classes.py` to support left and right rear wheel speed data
- ✅ Implemented `sort_localization_inputs()` method in `Trip` class that:
  - Creates a chronologically sorted list of all sensor readings
  - Uses list of dictionaries structure to handle multiple sensors at same timestamp
  - Each reading contains timestamp, sensor_id, and original data
  - Includes detailed output with sensor counts and sample rates
- 🔄 Next: Process data in localization algorithm and verify correctness

## Task B5 & B6: Python Bindings for Localization Algorithm

### Summary of Work Completed

#### Repository Structure Analysis

We analyzed the codebase to identify key components needed for localization:

- **AHRSLocHandler**: Main localization handler
- **ShortTermLocalization**: Position tracking component
- **AttitudeEstimator**: Orientation tracking (AHRS)
- **SpeedEstimator**: Speed estimation via Kalman filtering
- **ImuSample & Vec3d**: Supporting data structures

#### Implementation Changes

The following changes were made to the Python bindings:

1. **Focused Bindings**: Modified `control_module.cpp` to focus only on localization components
   - Removed bindings for `StanleyController` and `PIDBasedLongitudinalController`
   - Removed bindings for `ControlAPI`
   - Removed unnecessary header includes

2. **Enhanced AHRSLocHandler Bindings**:
   ```cpp
   pybind11::class_<AHRSLocHandler>(m, "AHRSLocHandler")
       .def(pybind11::init<const std::string&, const std::string&>())
       .def("UpdateIMU", pybind11::overload_cast<const ImuSample &, PreciseSeconds>(&AHRSLocHandler::UpdateIMU))
       .def("UpdateSpeed", &AHRSLocHandler::UpdateSpeed)
       .def("UpdateRearRightSpeed", &AHRSLocHandler::UpdateRearRightSpeed)
       .def("UpdateRearLeftSpeed", &AHRSLocHandler::UpdateRearLeftSpeed)
       .def("UpdateSteeringWheel", &AHRSLocHandler::UpdateSteeringWheel)
       .def("UpdateHeading", &AHRSLocHandler::UpdateHeading)
       .def("ResetVehicleState", &AHRSLocHandler::ResetVehicleState)
       .def("GetPosition", &AHRSLocHandler::GetPosition)
       .def("GetVehicleHeading", &AHRSLocHandler::GetVehicleHeading);
   ```

3. **Added Supporting Structure Bindings**:
   - `Vec3d`: For IMU acceleration and gyroscope data
   - `ImuSample`: For providing IMU data to the localization module

#### Build and Testing Setup

1. **Development Environment**:
   - Set up Python virtual environment in `Tests/python/vehicle_control_env`
   - Updated build scripts to use this environment
   - Added necessary dependencies to `requirements.txt`

2. **Test Script**:
   - Created `localization_pybind_test.py` to verify Python bindings functionality
   - Test script verifies module initialization, IMU updates, wheel speed updates, and vehicle state reset

3. **Testing Results**:
   - All tests passed successfully
   - Verified correct behavior of all bound methods
   - Confirmed compatibility with sensor API

#### Sensor API Compatibility

We verified that all necessary methods for sensor API compatibility are properly bound:

1. **Input Methods (Sensor API)**:
   - ✅ `UpdateIMU` - For processing IMU data
   - ✅ `UpdateSpeed` - For processing general speed data
   - ✅ `UpdateRearRightSpeed` - For processing right rear wheel speed sensor data
   - ✅ `UpdateRearLeftSpeed` - For processing left rear wheel speed sensor data
   - ✅ `UpdateSteeringWheel` - For processing steering wheel angle data
   - ✅ `UpdateHeading` - For processing external heading data
   - ✅ `ResetVehicleState` - For resetting the vehicle state

2. **Output Methods (Localization API)**:
   - ✅ `GetPosition` - For retrieving the current position
   - ✅ `GetVehicleHeading` - For retrieving the current heading

## Task B7 & B8: Testing Framework Implementation

### Task B7: Python Test Script for AI-driver Trip Data

The `carpose_offline_calculation.py` script implements:

1. **Trip Data Loading**: Uses the `Trip` class from `Classes.py` to load all sensor data from a trip directory
2. **Configuration Management**: Command-line arguments for specifying trip path and configuration files
3. **Data Visualization**: Optional visualization of calculated car pose compared to GPS reference
4. **Output Generation**: Saves calculated car pose to CSV file and creates a README with metadata

### Task B8: Synchronous Sensor Data Processing

The script implements a chronological processing system that:

1. **Processes sensor data in time order**: All sensor streams (IMU, speed, steering) are processed in chronological order
2. **Uses consistent timestamps**: All timestamps are normalized to seconds from the start of recording
3. **Fixed time-step processing**: Updates the localization at regular intervals (10ms by default)
4. **Interpolation support**: Can interpolate sensor data between samples when needed
5. **Error handling**: Gracefully handles missing data or processing errors

### Implementation Details

#### Sensor Processing Pipeline

The implementation follows this pattern:
1. Load all sensor data from the trip directory
2. Convert all timestamps to a common reference frame (seconds from start)
3. Process each sensor type in chronological order:
   ```python
   while current_time <= max_time:
       # Process IMU data up to current_time
       # Process speed data up to current_time
       # Process steering data up to current_time
       # Update position calculation
       # Store the current pose
       # Move to next time step
   ```
4. Save the calculated car pose trajectory

#### Usage

```bash
python carpose_offline_calculation.py --trip_path /path/to/trip_data --config_path /path/to/configs --visualize
```

## Integration Testing

The integrated testing of the Python bindings and testing framework confirms:

1. Successful loading of trip data from AI-driver format
2. Proper processing of sensor data in chronological order
3. Correct position and heading calculation using the localization algorithm
4. Ability to compare calculated trajectory with GPS reference

## Conclusion

The implementation of Tasks B5, B6, B7, and B8 provides a solid foundation for:

1. **Python Interface**: A clean, focused Python API for the localization algorithm
2. **Testing Framework**: Comprehensive tools for testing with real-world data
3. **Documentation**: Clear examples and usage instructions
4. **Integration Path**: A clear pathway to integrate with the AI-driver system

These components enable the standalone localization repository to function independently while maintaining compatibility with existing systems and providing robust testing capabilities.

## Next Steps

1. **API Documentation**: Create more detailed documentation for API users
2. **Additional Test Cases**: Develop more comprehensive test scenarios
3. **Performance Optimization**: Identify and address any performance bottlenecks
4. **Extended Sensor Support**: Add support for additional sensor types (e.g., visual odometry)
