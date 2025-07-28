# Localization Python Bindings and Testing Implementation Notes

This document consolidates the implementation notes for Tasks B5, B6, B7, and B8 from the localization separation plan, providing a comprehensive overview of the Python bindings and testing framework for the standalone localization repository.

## Tasks Overview

| Task # | Description | Status |
|--------|-------------|--------|
| B5 | Create or verify Python bindings (pybind) for localization algorithm | ✅ Completed |
| B6 | Ensure compatibility with existing sensor API | ✅ Completed |
| B7 | Create Python test script for loading AI-driver trip data | ✅ Completed |
| B8 | Implement synchronous sensor data processing using timestamps | 🔄 In Progress |
| B10 | Remove control-related code and files | ✅ Completed |

## Latest Update - B10 Completion (July 28, 2025)

### B10: Remove Control-Related Code and Files
- ✅ Commented out control-related components in header files
- ✅ Commented out control-related functionality in implementation files
- ✅ Renamed `Controllers.cpp` and `Controllers.hpp` to use `.deprecated` extension
- ✅ Updated CMakeLists.txt to exclude control-related source files
- ✅ Fixed build issues related to Python bindings
- ✅ Verified localization functionality still works correctly with test script
- ✅ All localization tests pass with expected values:
  - X position: 670.500340
  - Y position: 504.368338
  - YAW angle: 0.778360

## Previous Update - B7 & B8 Progress (July 27, 2025)

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
- ✅ Fixed critical timestamp and unit conversion issues in the implementation:
  - Added proper IMU timestamp forwarding
  - Implemented unit conversion for gyroscope data (degrees to radians)
  - Added tqdm progress bar for better visibility during processing
- 🔄 Next: Initialize estimation heading from car pose and add comparison metrics with pass/fail criteria

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

### Implementation Challenges and Fixes (July 27, 2025)

Several critical issues were discovered and resolved during testing:

1. **IMU Timestamp Issues**: IMU samples weren't properly receiving timestamps
   - Fixed by adding explicit timestamp assignment: `imu_sample.time_stamp = timestamp`
   - Without this, the IMU data couldn't be properly synchronized with other sensors

2. **Angular Units Conversion**: Gyroscope data needed conversion from degrees to radians
   - Fixed conversion for both gyroscope readings and their bias values:
   ```python
   gyro_vec.x = float(data["x_gyro"]) * np.pi/180  # Convert to radians
   gyro_vec.y = float(data["y_gyro"]) * np.pi/180  # Convert to radians
   gyro_vec.z = float(data["z_gyro"]) * np.pi/180  # Convert to radians
   ```
   - Same conversion applied to gyroscope bias values

3. **Progress Tracking**: Added tqdm progress bar for better visibility during processing
   - Helps monitor processing of large datasets with potentially thousands of sensor readings

4. **External Car Pose Integration**: Added support for loading and comparing with external car pose data
   - Helps validate the localization algorithm against reference data

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

1. **Initialize Estimation Heading**: Add functionality to initialize the estimation heading from car pose data
2. **Comparison Metrics**: Implement quantitative comparison with external car pose data
3. **Pass/Fail Criteria**: Define objective criteria for determining if localization results are acceptable
4. **API Documentation**: Create more detailed documentation for API users
5. **Additional Test Cases**: Develop more comprehensive test scenarios
6. **Performance Optimization**: Identify and address any performance bottlenecks
7. **Extended Sensor Support**: Add support for additional sensor types (e.g., visual odometry)

## Task B10: Code Cleanup Plan - Removing Control-Related Code

### Overview

This plan outlines the incremental approach for removing control-related code from the repository, focusing on the localization components. The goal is to gradually transform this into a standalone localization library without control functionality.

### Testing Strategy

- Establish a baseline test script (`test_localization.sh`) to verify localization functionality after each removal step
- Define expected outputs (final position, heading) for regression testing
- Ensure all tests pass after each removal step before proceeding

### Removal Plan

#### Phase 0: Create Baseline Tests
1. **Create Test Script**
   - Create `test_localization.sh` that builds and runs the localization tests
   - Capture expected output from the test script as baseline
   - Ensure the test uses the Python script to validate localization functionality
   
2. **Identify Test Dependencies**
   - Map all test files referencing control components
   - Create a plan to modify/remove test cases that depend on control code
   - Ensure test infrastructure can validate only localization features

#### Phase 1: Control API & High-Level Components
1. **ControlAPI Class & Test References**
   - Remove `ControlAPI.hpp` and `ControlAPI.cpp`
   - Update `CMakeLists.txt` to remove references
   - **Remove/Modify Test References**:
     - Remove or comment out `TestControlAPI_init()`, `TestLQR_FromControlAPI()`, and `TestControlAPI_init2()` in `Tests/test_setup.hpp`
     - Update `Tests/test_setup.cpp` to remove the implementations of these functions
   - Run test script to verify localization still works

2. **Controller Components & Tests**
   - Remove `Utils/Controllers.hpp` and `Utils/Controllers.cpp` 
   - Update `Utils/CMakeLists.txt`
   - **Remove/Modify Test References**:
     - Remove or comment out `TestLQRControllerSingleSample()` in `Tests/ControllerTests.hpp`
     - Update corresponding implementation in `Tests/ControllerTests.cpp`
   - Run test script to verify localization components still function

#### Phase 2: Clean Up Integration Points
1. **Wrapper Layer**
   - Identify all control-related functions in `wrapper/control_api_wrapper.cpp`
   - Create a new file `localization_api_wrapper.cpp` with only localization functions
   - Update header file accordingly
   - Update `wrapper/CMakeLists.txt` to build the new file
   - Run test script to validate changes

2. **Test Cases & Infrastructure**
   - Remove all control-specific test cases from `Tests/ControllerTests.cpp`
   - Rename the file to `Tests/LocalizationTests.cpp` if appropriate
   - Update all references in `Tests/CMakeLists.txt` accordingly
   - Update any `#include` directives in other test files
   - Run test script to validate changes

#### Phase 3: Configuration Refinement
1. **Configuration Files**
   - Analyze `vehicle_config.json` and `control_config.json`
   - Create new `localization_config.json` with only localization parameters
   - Update all code references that load these configurations:
     - Update `AHRSLocHandler` initialization
     - Update Python bindings 
     - Update test cases
   - Verify all paths in test scripts point to new configuration
   - Run test script to verify configuration loading works correctly

2. **Debug/Utilities**
   - Remove control-specific debug states from `Utils/control_debug_states.cpp`
   - Create a new `Utils/localization_debug_states.cpp` file if needed
   - Update all references in other files:
     - Test cases
     - Python bindings
     - Main application code
   - Run test script to verify debug functionality is maintained

#### Phase 4: Python Bindings Cleanup
1. **Update Python Module**
   - Rename `control_module.cpp` to `localization_module.cpp`
   - Remove any remaining control-related bindings
   - Update all Python scripts to use the renamed module
   - Update build scripts and CMakeLists.txt
   - Run test script to verify Python bindings still work

2. **Python Test Infrastructure**
   - Update all Python test scripts to use the new module name
   - Remove any control-related test cases
   - Verify visualization tools still work
   - Run all Python tests to confirm functionality

#### Phase 5: Final Cleanup and Renaming
1. **Remaining References - Comprehensive Scan**
   - Use `grep` to scan entire codebase for control-related terms:
     ```bash
     grep -r "ControlAPI\|Stanley\|PID\|Longitudinal" --include="*.cpp" --include="*.hpp" .
     ```
   - Address any remaining references
   - Update all documentation to reflect localization-only focus
   - Run test script again to verify no regressions

2. **Repository Structure**
   - Reorganize directory structure for clarity:
     - Move localization core components to `core/` directory
     - Move test infrastructure to organized subdirectories
   - Update main README.md with clear focus on localization
   - Update build system (CMakeLists.txt files) to reflect new structure
   - Perform full clean build and run all tests

### Detailed Component Analysis

#### High-Level Components to Remove/Refactor

| Component | File | Action | Dependencies to Check |
|-----------|------|--------|------------------------|
| ControlAPI | ControlAPI.hpp/cpp | Remove | Wrapper, Tests, test_setup.hpp, test_setup.cpp |
| StanleyController | Utils/Controllers.hpp/cpp | Remove | ControlAPI, ControllerTests.hpp, ControllerTests.cpp |
| LongitudinalController | Utils/Controllers.hpp/cpp | Remove | ControlAPI, ControllerTests.hpp, ControllerTests.cpp |
| ControlWrapper | wrapper/control_api_wrapper.* | Refactor | Python bindings, control_module.cpp |
| ControlTests | Tests/ControllerTests.* | Refactor/Remove | test_setup.hpp, test_setup.cpp, CMakeLists.txt |
| Control Debug | Utils/control_debug_states.* | Remove | Utils headers, test cases |
| Python Bindings | control_module.cpp | Refactor | carpose_offline_calculation.py, other Python scripts |

#### Configuration Updates

| Config Section | Purpose | Action |
|----------------|---------|--------|
| Control_params | Control algorithm parameters | Remove |
| Lateral_controller | Stanley controller config | Remove |
| Longitudinal_controller | Speed controller config | Remove |
| Localization | AHRS and position tracking | Keep |
| Vehicle | Vehicle dimensions and properties | Keep relevant parts |

This phased approach ensures we can maintain a working localization system throughout the cleanup process, with regular testing to catch any regressions.
## Task B10: Remove Control-Related Code and Files

### Implementation Details

#### Key Files Modified

1. **Header Files**
   - `Utils/control_debug_states.hpp`: Commented out control-related methods
   - `Utils/Classes.hpp`: Removed control-related functionality
   - `ahrs_loc_handler.hpp`: Removed control interfaces

2. **Implementation Files**
   - `Utils/control_debug_states.cpp`: Commented out methods for debugging control states
   - `ahrs_loc_handler.cpp`: Removed control-related methods
   - `Main.cpp`: Commented out control test function calls

3. **Renamed/Deprecated Files**
   - Renamed `Controllers.cpp` → `Controllers.cpp.deprecated` 
   - Renamed `Controllers.hpp` → `Controllers.hpp.deprecated`
   - This prevents the Python binding's `GLOB_RECURSE` command from including these files

4. **Build System**
   - Updated `Utils/CMakeLists.txt`: Removed Controllers.cpp from source files list
   - Modified `Tests/python/python_binding/CMakeLists.txt`: Build works with localization-only code

### Challenges Addressed

1. **Python Binding Issues**: 
   - The Python binding build script was including all .cpp files regardless of CMake settings
   - Solution: Renamed files with .deprecated extension to exclude them from build

2. **Refactoring Control Debug States**:
   - Required careful commenting of control-related methods while preserving localization logging
   - Ensured proper function removal without breaking the remaining functionality

3. **Testing**:
   - Verified all localization functionality still works via the test_localization.sh script
   - All localization tests pass with expected position and orientation values

### Next Steps

- Consider creating proper abstractions between localization and control interfaces
- Further clean up deprecated code that is no longer used
- Improve Python testing framework with more comprehensive test cases
