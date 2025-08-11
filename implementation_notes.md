# Implementation Notes: StaticDynamicTest

This document outlines the development process for the `StaticDynamicTest` class, which was added to the Inertial Localization system to provide a robust mechanism for detecting the vehicle's motion state.

## 1. Initial Design and Scoping

The initial requirement was to create a class that could distinguish between `STATIC` and `DYNAMIC` states based on statistical analysis of sensor data. The proposed approach was to analyze the mean, standard deviation, and maximum values of key vehicle signals over a time window and compare them to configurable thresholds.

After discussion, the plan was enhanced to include the following signals for a more robust determination:
-   **Car Speed**: Mean and maximum value.
-   **IMU Accelerometer**: Standard deviation of the 3D vector norm.
-   **IMU Gyroscope**: Standard deviation of the 3D vector norm.

## 2. Implementation Steps

The implementation was carried out in the following sequence:

1.  **Configuration**: New thresholds for the static/dynamic test (`SD_test_acc_std_th`, `SD_test_gyro_std_th`, `SD_test_carspeed_mean_th`, `SD_test_carspeed_max_th`) were added to the `localization_config.json` file.

2.  **Utility Functions**:
    -   A `VectorNorm3D` function was added to `Utils/Functions.cpp` to calculate the Euclidean norm of a 3D vector, used for processing IMU data.
    -   A `StandardDeviation` function was implemented in `Utils/Functions.cpp`, leveraging the Eigen library for efficient and accurate calculation.

3.  **Class Definition**:
    -   The `StaticDynamicTest` class was created in `Utils/StaticDynamicTest.hpp` and `Utils/StaticDynamicTest.cpp`.
    -   The class uses `BufferAny` members to store recent samples of accelerometer norm, gyroscope norm, and car speed.
    -   The constructor was designed to take a `nlohmann::json` object to load its threshold configuration.

4.  **State Logic**:
    -   The `CalculateState` method was implemented to perform the statistical calculations on the data in the buffers.
    -   The state machine logic transitions the internal state between `NOT_INITIALIZED`, `STATIC`, and `DYNAMIC` based on whether the calculated statistics exceed the configured thresholds.
    -   The `UpdateIMU` and `UpdateCarSpeed` methods were implemented to add new sensor data to the buffers and immediately trigger a recalculation of the state.

5.  **Refactoring**: Based on feedback, the `BufferAny` class was enhanced with a `GetBufferValues` method to simplify and encapsulate the logic of retrieving all values from a buffer. The `StaticDynamicTest::CalculateState` method was refactored to use this new, cleaner interface.

## 3. Testing

A comprehensive test suite was developed for the `StaticDynamicTest` class using the Google Test framework.

1.  **Test File**: A new test file, `Tests/test_static_dynamic_test.cpp`, was created.

2.  **Test Data Generation**: A helper function, `GenerateRandomSignal`, was implemented to produce test signals with a specific mean and standard deviation using C++'s standard `<random>` library. This allowed for the creation of realistic, yet deterministic, test data.

3.  **Test Cases**: The test suite covers the following scenarios:
    -   Correct initialization to the `NOT_INITIALIZED` state.
    -   Ensuring the state remains `NOT_INITIALIZED` until the data buffers are full.
    -   Correct transition to the `STATIC` state when provided with low-variance data.
    -   Correct transition to the `DYNAMIC` state based on high variance/values from each of the individual sensor inputs (accelerometer, gyroscope, and car speed).
    -   State transitions from `STATIC` to `DYNAMIC` and vice-versa.

## 4. Build System Integration and Verification

1.  The new source files (`StaticDynamicTest.cpp` and `test_static_dynamic_test.cpp`) were added to the appropriate `CMakeLists.txt` files to integrate them into the build system.

2.  A build failure was encountered due to a circular include dependency (`Functions.hpp` -> `Sensors.hpp` -> `Classes.hpp` -> `Functions.hpp`).

3.  The issue was resolved by removing an unnecessary `#include "Classes.hpp"` from `Utils/Sensors.hpp`, which broke the dependency cycle.

4.  After fixing the build, all 28 unit tests in the project, including the 9 new tests for `StaticDynamicTest`, were run and passed successfully. This was achieved by executing the test runner from the `cmake-bin` directory, as specified in the `README.md`, which resolved pre-existing path issues in other test suites.
