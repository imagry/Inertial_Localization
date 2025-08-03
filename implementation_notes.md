### 2025-08-03: AHRS Component Analysis

* **Action:** Performed an analysis of Utils/AHRS.hpp and Utils/AHRS.cpp for code quality and potential issues.
* **Details:** Reviewed the AHRS (Attitude and Heading Reference System) component which is responsible for determining vehicle orientation and heading based on IMU data. The analysis focused on code structure, memory management patterns, and style conformance.
* **Files Analyzed:** `Utils/AHRS.hpp`, `Utils/AHRS.cpp`

### 2025-08-03: Memory Management Analysis of AHRS Component

* **Action:** Analyzed memory management patterns in the AHRS component.
* **Details:** 
  * **Memory Management**: No direct memory leaks detected as the code doesn't use raw pointers, `new`, or `delete` operations. The component primarily uses stack-allocated objects and STL containers which manage their own memory.
  * **RAII Compliance**: The component follows RAII principles with automatic resource management through standard containers like `vector`.
  * **Smart Pointers**: No usage of smart pointers (`std::unique_ptr`, `std::shared_ptr`) as dynamic memory allocation isn't employed in this component.
* **Files Analyzed:** `Utils/AHRS.hpp`, `Utils/AHRS.cpp`

### 2025-08-03: Concurrency Analysis of AHRS Component

* **Action:** Examined the AHRS component for thread-safety and proper mutex usage.
* **Details:** No mutex or thread synchronization mechanisms were found in the AHRS component. The code appears to be designed for single-threaded operation. If this component is accessed from multiple threads in the application, there could be potential race conditions when modifying shared state like the rotation matrix (`Rnb`) or when updating the attitude.
* **Files Analyzed:** `Utils/AHRS.hpp`, `Utils/AHRS.cpp`

### 2025-08-03: Style Guide Compliance of AHRS Component

* **Action:** Evaluated adherence to Google C++ Style Guide in the AHRS component.
* **Details:** Several style guide deviations were identified:
  * **Naming Conventions**: Class member variables aren't consistently following the trailing underscore convention (e.g., `dt`, `phi`, `theta` vs. `clock_initialized_`).
  * **Using Declarations**: Using declarations appear in global scope in the header file, which the Google Style Guide discourages.
  * **Include Order**: The include order doesn't follow the recommended style (own header, C headers, C++ headers, other libraries, project headers).
  * **Comments**: Some functions lack proper documentation comments.
  * **Variable Naming**: Some variable names are unclear or too short (e.g., `g`, `dt`).
* **Files Analyzed:** `Utils/AHRS.hpp`, `Utils/AHRS.cpp`

### 2025-08-03: Include Order Improvements for AHRS Component

* **Action:** Analyzed include order in AHRS.hpp and AHRS.cpp and prepared recommendations according to Google C++ Style Guide.
* **Details:** According to the Google C++ Style Guide, includes should be ordered as follows:
  1. Related header (i.e., AHRS.hpp for AHRS.cpp)
  2. C system headers (e.g., <math.h>)
  3. C++ standard library headers (e.g., <string>, <vector>)
  4. Other libraries' headers (e.g., Eigen)
  5. Your project's headers
  
  Each category should be separated by a blank line and sorted alphabetically within each category.

* **Proposed changes for AHRS.hpp:**
```cpp
/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/

#pragma once

#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

// Forward declarations for Eigen types, used in inline functions
namespace Eigen {
class Matrix;
class Matrix2d;
class Matrix3d;
class MatrixXd;
class RowVector3d;
class Vector;
class Vector2d;
class Vector3d;
class VectorXd;
}  // namespace Eigen
```

* **Proposed changes for AHRS.cpp:**
```cpp
/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Feb 19 2024 by Eran Vertzberger
*/

#include "AHRS.hpp"

#include <cassert>  // for assertion
#include <cmath>    // instead of math.h

#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <utility>

#include <unsupported/Eigen/MatrixFunctions>

#define assertm(exp, msg) assert(((void)msg, exp))
#define _USE_MATH_DEFINES
```

* **Files Modified:** `Utils/AHRS.hpp`, `Utils/AHRS.cpp`

### 2025-08-03: Naming Conventions Improvements for AHRS Component

* **Action:** Analyzed naming conventions in AHRS.hpp and AHRS.cpp and prepared recommendations according to Google C++ Style Guide.
* **Details:** According to the Google C++ Style Guide:
  1. Class/struct names should be in CamelCase with initial capital
  2. Function names should be in CamelCase with initial lowercase
  3. Variable names should be in snake_case
  4. Class member variables should have trailing underscores
  5. Constants should be named with a leading k followed by mixed case
  
  **Class/Type Names:**
  - `SegmentIMURecording` → Correct (CamelCase with initial capital)
  - `AttitudeEstimator` → Correct (CamelCase with initial capital)
  - `buffer_of_scalars` → Should be `BufferOfScalars`
  - `buffer_of_vectors` → Should be `BufferOfVectors`
  - `buffer_of_matrices` → Should be `BufferOfMatrices`
  
  **Function Names:**
  - `compute_dt()` → Should be `ComputeDt()`
  - `GyroPromotion()` → Correct but inconsistent with other methods (should be `gyroPromotion()` for consistency)
  - `UpdateGravity()` → Correct but inconsistent with other methods
  - `InitializeRotation()` → Correct but inconsistent with other methods
  - `run_exp()` → Should be `runExp()`
  - `add_data()` → Should be `addData()`
  - `get_data()` → Should be `getData()`
  - `resest()` → Should be `reset()` (also appears to be a typo)
  
  **Member Variables:**
  - Class `AttitudeEstimator`:
    - `dt`, `phi`, `theta`, `psi`, etc. → Should have trailing underscores: `dt_`, `phi_`, `theta_`, `psi_`
    - `clock_initialized_`, `rotation_initialized_` → Correctly follow convention
  
  - Class `SegmentIMURecording` and buffer classes:
    - All member variables should have trailing underscores

* **Files Modified:** `Utils/AHRS.hpp`, `Utils/AHRS.cpp`

### 2025-08-03: Function Documentation Improvements for AHRS Component

* **Action:** Analyzed function documentation in AHRS.hpp and AHRS.cpp and prepared recommendations according to Google C++ Style Guide.
* **Details:** According to the Google C++ Style Guide, function declarations should have comments that describe what the function does and how to use it. The comments should be descriptive ("Opens the file") rather than imperative ("Open the file").

* **Proposed documentation for key methods in AttitudeEstimator class:**

```cpp
/**
 * @brief Processes gyroscope measurements to update the rotation matrix.
 * 
 * Applies the gyroscope measurements to update the attitude estimation using
 * a first-order integration scheme. Also tracks the time between updates.
 * 
 * @param gyro Vector of 3 angular velocities (x, y, z) in rad/s
 * @param clock Current timestamp in seconds
 */
void GyroPromotion(vector<double> gyro, double clock);

/**
 * @brief Updates the gravity vector estimation using accelerometer data.
 * 
 * Applies the TRIAD algorithm to fuse accelerometer measurements with the
 * current attitude estimate. The result is used to correct the rotation matrix.
 * 
 * @param acc Vector of 3 accelerations (x, y, z) in m/s^2
 */
void UpdateGravity(vector<double> acc);

/**
 * @brief Initializes the rotation matrix with given Euler angles.
 * 
 * Sets the initial attitude of the system using roll (phi), pitch (theta),
 * and yaw (psi) angles.
 * 
 * @param phi0 Initial roll angle in radians
 * @param theta0 Initial pitch angle in radians
 * @param psi0 Initial yaw angle in radians
 */
void InitializeRotation(double phi0, double theta0, double psi0);

/**
 * @brief Runs an experiment using recorded IMU data.
 * 
 * Processes a sequence of IMU measurements to estimate attitude over time
 * and saves the results to a CSV file.
 * 
 * @param exp The IMU recording segment containing data to process
 * @param file_name Path to the output CSV file
 * @param initial_heading Initial heading in radians
 */
void run_exp(SegmentIMURecording exp, string file_name, double initial_heading);
```

* **Files Modified:** `Utils/AHRS.hpp`

### 2025-08-03: Implementation Example for AttitudeEstimator Class Refactoring

* **Action:** Created a sample implementation showing how to refactor the AttitudeEstimator class to conform to Google C++ Style Guide.
* **Details:** This example shows renaming of member variables to use trailing underscores, method names to use consistent camelCase with initial lowercase, and adding proper documentation.

**Original code snippet:**
```cpp
class AttitudeEstimator {
 public:
    // attributes
    double dt, phi, theta, psi, Kgx, Kgy, Kgz, g, update_time;
    /*vector<vector<double>> Rnb;
    vector<double> gn, gb, mn, mb;*/
    Matrix3d Rnb;
    Vector3d gn, gb, mn, mb;
    std::string coor_sys_convention_;
    bool clock_initialized_ = false;
    bool rotation_initialized_ = false;
    // methods
    /*AttitudeEstimator(double dt, vector<vector<double>> Rnb, double Ka);*/
    AttitudeEstimator(double dt, double Ka,
                                     std::string coor_sys_convention = "NED");
    void GyroPromotion(vector<double> gyro, double clock);
    void UpdateGravity(vector<double> acc);
    void InitializeRotation(double phi0, double theta0, double psi0);
    void run_exp(
        SegmentIMURecording exp,
        string file_name, double initial_heading);
};
```

**Refactored code snippet:**
```cpp
/**
 * @brief Class for estimating attitude based on IMU data.
 * 
 * This class implements a complementary filter to fuse gyroscope and
 * accelerometer data for robust attitude estimation.
 */
class AttitudeEstimator {
 public:
    /**
     * @brief Constructs an AttitudeEstimator with specified parameters.
     * 
     * @param dt_val Time step between updates in seconds
     * @param ka Gain parameter for the accelerometer correction
     * @param coor_sys_convention Coordinate system convention ("NED" or "ENU")
     */
    AttitudeEstimator(double dt_val, double ka,
                      std::string coor_sys_convention = "NED");
                      
    /**
     * @brief Processes gyroscope measurements to update the rotation matrix.
     * 
     * Applies the gyroscope measurements to update the attitude estimation using
     * a first-order integration scheme. Also tracks the time between updates.
     * 
     * @param gyro Vector of 3 angular velocities (x, y, z) in rad/s
     * @param clock Current timestamp in seconds
     */
    void gyroPromotion(vector<double> gyro, double clock);
    
    /**
     * @brief Updates the gravity vector estimation using accelerometer data.
     * 
     * Applies the TRIAD algorithm to fuse accelerometer measurements with the
     * current attitude estimate. The result is used to correct the rotation matrix.
     * 
     * @param acc Vector of 3 accelerations (x, y, z) in m/s^2
     */
    void updateGravity(vector<double> acc);
    
    /**
     * @brief Initializes the rotation matrix with given Euler angles.
     * 
     * Sets the initial attitude of the system using roll (phi), pitch (theta),
     * and yaw (psi) angles.
     * 
     * @param phi0 Initial roll angle in radians
     * @param theta0 Initial pitch angle in radians
     * @param psi0 Initial yaw angle in radians
     */
    void initializeRotation(double phi0, double theta0, double psi0);
    
    /**
     * @brief Runs an experiment using recorded IMU data.
     * 
     * Processes a sequence of IMU measurements to estimate attitude over time
     * and saves the results to a CSV file.
     * 
     * @param exp The IMU recording segment containing data to process
     * @param file_name Path to the output CSV file
     * @param initial_heading Initial heading in radians
     */
    void runExp(SegmentIMURecording exp, string file_name, double initial_heading);

 private:
    // Time step for integration in seconds
    double dt_;
    // Roll angle in radians
    double phi_;
    // Pitch angle in radians
    double theta_;
    // Yaw angle in radians
    double psi_;
    // Gain parameters for accelerometer correction
    double kgx_, kgy_, kgz_;
    // Gravity constant in m/s^2
    double g_;
    // Last update timestamp in seconds
    double update_time_;
    // Rotation matrix from body to navigation frame
    Matrix3d rnb_;
    // Gravity vector in navigation frame
    Vector3d gn_;
    // Gravity vector in body frame
    Vector3d gb_;
    // Magnetic field vector in navigation frame
    Vector3d mn_;
    // Magnetic field vector in body frame
    Vector3d mb_;
    // Coordinate system convention ("NED" or "ENU")
    std::string coor_sys_convention_;
    // Flag indicating if clock has been initialized
    bool clock_initialized_ = false;
    // Flag indicating if rotation matrix has been initialized
    bool rotation_initialized_ = false;
};
```

* **Files Modified:** `Utils/AHRS.hpp`, `Utils/AHRS.cpp`

### 2025-08-03: Code Style Improvements for ShortTermLocalization Component

* **Action:** Removed unnecessary `this->` qualifiers from ShortTermLocalization class methods.
* **Details:** Similar to the changes made in the AHRS component, removed redundant `this->` qualifiers when accessing member variables. The use of `this->` is only necessary to resolve name conflicts between member variables and function parameters or local variables, which was not the case in this file. Changes included:
  * In `UpdateIMU` methods: Changed `this->IMU_.*` to `IMU_.*` for all IMU field accesses
  * In `UpdateFrontAxlePosition`: Changed `this->update_time_ = clock;` to `update_time_ = clock;`
  * In commented code in `UpdatePosition`: Changed `this->update_time_ = clock;` to `update_time_ = clock;` for consistency
* **Files Modified:** `Utils/short_term_localization.cpp`
