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

### 2025-08-03: Analysis of SpeedEstimators Component

* **Action:** Performed an analysis of Utils/SpeedEstimators.hpp and Utils/SpeedEstimators.cpp for code quality and potential issues.
* **Details:** Reviewed the SpeedEstimators component which implements different algorithms for vehicle speed estimation. The analysis focused on code structure, naming conventions, and documentation.
* **Files Analyzed:** `Utils/SpeedEstimators.hpp`, `Utils/SpeedEstimators.cpp`

### 2025-08-03: Include Order Improvements for SpeedEstimators Component

* **Action:** Analyzed include order in SpeedEstimators.hpp and SpeedEstimators.cpp and prepared recommendations according to Google C++ Style Guide.
* **Details:** According to the Google C++ Style Guide, includes should be ordered as follows:
  1. Related header (i.e., SpeedEstimators.hpp for SpeedEstimators.cpp)
  2. C system headers (e.g., <math.h>)
  3. C++ standard library headers (e.g., <string>, <vector>)
  4. Other libraries' headers (e.g., Eigen)
  5. Your project's headers
  
  **Proposed changes for SpeedEstimators.cpp:**
```cpp
/* Copyright (c) 2024 Imagry. All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Created on Thu Nov 28 2024 by Dor Siman Tov
*/

#include "SpeedEstimators.hpp"

#include <cmath>  // instead of math.h

#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "units.hpp"
```

* **Files Analyzed:** `Utils/SpeedEstimators.cpp`

### 2025-08-03: Naming Conventions in SpeedEstimators Component

* **Action:** Analyzed naming conventions in the SpeedEstimators component.
* **Details:** Identified several naming inconsistencies:
  * Member variables in the KalmanFilter class like `IMU_ACC_NOISE_DENSITY_` use all-caps which is typically reserved for constants, not member variables.
  * According to Google C++ Style Guide, constants should be named with a leading k followed by mixed case (e.g., kImuAccNoiseDensity).
  * Class member variables correctly use trailing underscores.
  * Function names use CamelCase, which is consistent with the style guide.
  
* **Files Analyzed:** `Utils/SpeedEstimators.hpp`, `Utils/SpeedEstimators.cpp`

### 2025-08-03: Documentation Improvements for SpeedEstimators Component

* **Action:** Identified documentation gaps in the SpeedEstimators component.
* **Details:** The SpeedEstimators module lacks proper documentation for its classes and methods. Adding Doxygen-style comments similar to those in AHRS.hpp would improve code readability and maintainability.
  
  **Proposed documentation for SpeedEstimator base class:**
```cpp
/**
 * @brief Abstract base class for vehicle speed estimation.
 * 
 * This class defines the interface for different speed estimation strategies.
 * Each concrete implementation provides a specific algorithm for estimating
 * vehicle speed based on sensor data.
 */
class SpeedEstimator {
 public:
    virtual ~SpeedEstimator() = default;
    
    /**
     * @brief Updates the estimator with a new IMU sample.
     * 
     * @param sample Pointer to the new IMU sample
     */
    virtual void UpdateIMU(const ImuSample* sample) = 0;
    
    /**
     * @brief Updates the estimator with new wheel odometry data.
     * 
     * @param sample Pointer to the new wheel odometry sample
     */
    virtual void UpdateRearSpeeds(const WheelOdometrySample* sample) = 0;
    
    /**
     * @brief Estimates the vehicle state based on current sensor data.
     * 
     * @param clock Current timestamp
     */
    virtual void UpdateState(PreciseSeconds clock) = 0;
    
    /**
     * @brief Gets the current estimated vehicle speed.
     * 
     * @return PreciseMps Estimated speed in meters per second
     */
    virtual PreciseMps GetEstimatedSpeed() = 0;
    
    /**
     * @brief Checks if the current speed estimate is valid.
     * 
     * @return bool True if the estimate is valid, false otherwise
     */
    virtual bool IsEstimatedSpeedValid() = 0;
};
```

* **Files Analyzed:** `Utils/SpeedEstimators.hpp`

### 2025-08-03: Analysis of Classes Component

* **Action:** Performed an analysis of Utils/Classes.hpp for code quality and potential issues.
* **Details:** Reviewed the Classes component which provides various utility classes for signal processing, filtering, and data handling. The analysis focused on code structure, naming conventions, and documentation.
* **Files Analyzed:** `Utils/Classes.hpp`

### 2025-08-03: Naming Conventions in Classes Component

* **Action:** Analyzed naming conventions in the Classes component.
* **Details:** Identified several naming inconsistencies:
  * Class names use a mix of snake_case (`Buffer_any`) and CamelCase (`FirstOrderLPF`), while Google C++ Style Guide recommends CamelCase for all class names.
  * Method names generally follow CamelCase which is consistent with the style guide, though some like `Update` and `Reset` start with capitals which should be reserved for classes.
  * Member variables consistently use trailing underscores, which is correct.
  
  **Proposed changes for class names:**
  * `Buffer_any` → `BufferAny`
  * `Delay_any` → `DelayAny`
  
* **Files Analyzed:** `Utils/Classes.hpp`

### 2025-08-03: Implementation Patterns in Classes Component

* **Action:** Analyzed implementation patterns in the Classes component.
* **Details:** Several classes in Classes.hpp have their method implementations directly in the header file. While this is acceptable for small, performance-critical methods, it can lead to longer compilation times and potential ODR (One Definition Rule) violations if the header is included in multiple translation units.

  Recommendation: Consider moving larger method implementations to a separate .cpp file, especially for methods like `StepTest` in the `RateLimiter` class that are not performance-critical.
  
* **Files Analyzed:** `Utils/Classes.hpp`

### 2025-08-03: Documentation Improvements for Classes Component

* **Action:** Identified documentation gaps in the Classes component.
* **Details:** Documentation is inconsistent across the file. Some classes have good comments (like `FirstOrderLPF`), while others have minimal or no documentation.
  
  **Proposed documentation for Buffer_any class:**
```cpp
/**
 * @brief A generic buffer class that can store any type of values.
 * 
 * This class implements a circular buffer with a fixed maximum size.
 * When the buffer is full, adding new values removes the oldest ones.
 */
struct Buffer_any {
    std::deque<std::any> values_;  ///< Container for the buffer values
    int maximal_size_;             ///< Maximum number of values the buffer can hold
    
    /**
     * @brief Constructs a buffer with the specified maximum size.
     * 
     * @param max_size Maximum number of values the buffer can hold
     */
    explicit Buffer_any(int max_size);
    
    /**
     * @brief Adds a new value to the buffer.
     * 
     * If the buffer is already at maximum capacity, the oldest value is removed.
     * 
     * @param new_value Value to add to the buffer
     */
    void Update(std::any new_value);
    
    /**
     * @brief Gets the value at the specified index.
     * 
     * @param idx Index of the value to retrieve
     * @return std::any Value at the specified index
     */
    std::any GetValue(int idx);
    
    /**
     * @brief Gets the oldest value in the buffer.
     * 
     * @return std::any The first (oldest) value in the buffer
     */
    std::any GetFirst();
    
    /**
     * @brief Gets the newest value in the buffer.
     * 
     * @return std::any The last (newest) value in the buffer
     */
    std::any GetLast();
    
    /**
     * @brief Finds the value closest to the target.
     * 
     * Only works for buffers containing double values.
     * 
     * @param target Value to find the closest match for
     * @return std::tuple<int, double> Index and value of the closest match
     */
    std::tuple<int, double> ClosestValue(double target);
    
    /**
     * @brief Gets the current number of values in the buffer.
     * 
     * @return int Number of values currently stored
     */
    ````markdown
    int Size();
};
```

* **Files Analyzed:** `Utils/Classes.hpp`

## Code Quality Improvements

### 2023-11-15: Exception Handling Improvements

* **Action:** Replaced assert statements with proper exception handling in multiple modules
* **Details:** 
  * Removed `assert` statements from the codebase and replaced them with proper C++ exception handling
  * Used `std::invalid_argument` and `std::runtime_error` exceptions where appropriate to provide informative error messages
  * Removed `assertm` macro from several files and replaced its usage with exception throwing
  * Added proper error checking with conditional statements followed by exceptions
  * These changes make the code more robust in production environments where assertions might be disabled
* **Files Modified:** 
  * `Utils/Functions.cpp`: Replaced assertions in `VectorNorm2D()` and `ProjectPoints2D()` with proper exception handling
  * `Utils/AHRS.cpp`: Replaced assertions with exception handling in multiple methods including:
    * Error reporting in file format handling
    * Data validation in `BufferOfVectors::Add_data()`
    * Dimension checking in `BufferOfMatrices::Add_data()`
    * Vector validation in `Vector_2_norm()`, `Norm_nX2_array()`, and `Quaternion2euler()`
  * `Utils/DataHandling.cpp`: Removed unnecessary assertion include
* **Example changes:**

  **Before:**
  ```cpp
  double VectorNorm2D(const std::vector<double> &vec2d) {
      assert(vec2d.size() == 2);
      double vec_norm = sqrt(pow(vec2d[0], 2) + pow(vec2d[1], 2));
      return vec_norm;
  }
  ```

  **After:**
  ```cpp
  double VectorNorm2D(const std::vector<double> &vec2d) {
      if (vec2d.size() != 2) {
          throw std::invalid_argument("VectorNorm2D: invalid dimensions (must be 2)");
      }
      double vec_norm = sqrt(pow(vec2d[0], 2) + pow(vec2d[1], 2));
      return vec_norm;
  }
  ```

### 2023-11-15: Function Module Documentation Improvements

* **Action:** Added comprehensive Doxygen-style documentation to the Functions module
* **Details:**
  * Added detailed documentation comments to all functions in `Functions.hpp`
  * Fixed typos in function names (e.g., `ConverteigenToVecor` → `ConvertEigenToVector`)
  * Organized functions into logical groups with section comments
  * Added parameter descriptions and return value documentation
* **Files Modified:**
  * `Utils/Functions.hpp`: Added Doxygen comments to all functions and organized them into logical sections
  * `Utils/Functions.cpp`: Fixed function name typos to match header changes
* **Example improvements:**

  **Before:**
  ```cpp
  std::vector<double> AddScalarToVector(const std::vector<double> &vec,
                                        double scalar);
  std::vector<double> MultiplyVectorByScalar(const std::vector<double> &vec,
                                             double scalar);
  ```

  **After:**
  ```cpp
  /**
   * @brief Adds a scalar value to each element of a vector.
   * 
   * @param vec Input vector
   * @param scalar Value to add to each element
   * @return std::vector<double> New vector with scalar added to each element
   */
  std::vector<double> AddScalarToVector(const std::vector<double> &vec,
                                        double scalar);

  /**
   * @brief Multiplies each element of a vector by a scalar.
   * 
   * @param vec Input vector
   * @param scalar Value to multiply each element by
   * @return std::vector<double> New vector with elements multiplied by scalar
   */
  std::vector<double> MultiplyVectorByScalar(const std::vector<double> &vec,
                                             double scalar);
  ```

### 2023-11-15: Classes Module Refactoring

* **Action:** Renamed classes in the Classes module to follow CamelCase naming convention and added documentation
* **Details:**
  * Renamed `Buffer_any` class to `BufferAny` (CamelCase)
  * Renamed `Delay_any` class to `DelayAny` (CamelCase)
  * Added comprehensive Doxygen-style documentation to all classes and methods
  * Updated all method signatures and method calls to reflect the renamed classes
  * Ensured all tests still pass after the refactoring
* **Files Modified:**
  * `Utils/Classes.hpp`: Updated class names and added documentation
  * `Utils/Classes.cpp`: Updated all class and method implementations to match the header changes
* **Example improvements:**

  **Before:**
  ```cpp
  class Buffer_any {
    int max_size_;
    vector<std::any> data_;
  
  public:
    Buffer_any(int max_size);
    void Update(std::any data);
    std::any GetLast();
    int Size();
  };
  ```

  **After:**
  ```cpp
  /**
   * @brief Generic circular buffer that can store any data type.
   * 
   * This class provides a fixed-size circular buffer implementation
   * that can store values of any type using std::any.
   */
  class BufferAny {
    int max_size_;
    vector<std::any> data_;
  
  public:
    /**
     * @brief Constructs a buffer with specified maximum size.
     * 
     * @param max_size Maximum number of elements to store
     */
    BufferAny(int max_size);
    
    /**
     * @brief Adds a new element to the buffer.
     * 
     * If the buffer is full, the oldest element is removed.
     * 
     * @param data The element to add
     */
    void Update(std::any data);
    
    /**
     * @brief Gets the most recently added element.
     * 
     * @return std::any The most recent element
     */
    std::any GetLast();
    
    /**
     * @brief Gets the current number of elements in the buffer.
     * 
     * @return int Current buffer size
     */
    int Size();
  };
  ```

### 2023-11-15: SpeedEstimators Module Refactoring

* **Action:** Improved SpeedEstimators module with better documentation, naming conventions, and code organization
* **Details:**
  * Updated include order to follow Google C++ Style Guide (system headers, third-party libraries, then project headers)
  * Renamed member variables from ALL_CAPS to lowercase_with_underscore with trailing underscores
  * Added comprehensive Doxygen-style documentation to all classes and methods
  * Improved class hierarchy documentation with proper description of each algorithm
* **Files Modified:**
  * `Utils/SpeedEstimators.hpp`: Added documentation and updated naming conventions
  * `Utils/SpeedEstimators.cpp`: Updated include order and variable names to match header changes
* **Example improvements:**

  **Before:**
  ```cpp
  class SpeedEstimator {
  public:
      virtual ~SpeedEstimator() = default;
      virtual void UpdateIMU(const ImuSample* sample) = 0;
      virtual void UpdateRearSpeeds(const WheelOdometrySample* sample) = 0;
      virtual void UpdateState(PreciseSeconds clock) = 0;
      virtual PreciseMps GetEstimatedSpeed() = 0;
      virtual bool IsEstimatedSpeedValid() = 0;
  };
  ```

  **After:**
  ```cpp
  /**
   * @brief Abstract base class for vehicle speed estimation.
   * 
   * This class defines the interface for different speed estimation strategies.
   * Each concrete implementation provides a specific algorithm for estimating
   * vehicle speed based on sensor data.
   */
  class SpeedEstimator {
  public:
      virtual ~SpeedEstimator() = default;
      
      /**
       * @brief Updates the estimator with a new IMU sample.
       * 
       * @param sample Pointer to the new IMU sample
       */
      virtual void UpdateIMU(const ImuSample* sample) = 0;
      
      /**
       * @brief Updates the estimator with new wheel odometry data.
       * 
       * @param sample Pointer to the new wheel odometry sample
       */
      virtual void UpdateRearSpeeds(const WheelOdometrySample* sample) = 0;
      
      /**
       * @brief Estimates the vehicle state based on current sensor data.
       * 
       * @param clock Current timestamp
       */
      virtual void UpdateState(PreciseSeconds clock) = 0;
      
      /**
       * @brief Gets the current estimated vehicle speed.
       * 
       * @return PreciseMps Estimated speed in meters per second
       */
      virtual PreciseMps GetEstimatedSpeed() = 0;
      
      /**
       * @brief Checks if the current speed estimate is valid.
       * 
       * @return bool True if the estimate is valid, false otherwise
       */
      virtual bool IsEstimatedSpeedValid() = 0;
  };
  ```
````
};
```

* **Files Analyzed:** `Utils/Classes.hpp`
### 2025-08-03: Eliminated Deprecated ResetVehicleState Method

* **Action:** Removed the deprecated `ResetVehicleState` method from `AHRSLocHandler` class
* **Details:**
  * Simplified the API by using only `UpdateVehicleState` method for both regular updates and state resets
  * Updated all code references to use the consolidated method:
    * Python bindings in `Tests/python/python_binding/localization_pybind_module.cpp`
    * Test scripts in `Tests/python/python_binding/carpose_offline_calculation.py`
    * Control API wrapper in `wrapper/control_api_wrapper.cpp`
  * Verified full functionality with the regression test suite
  * Confirmed no performance impact while improving code maintainability
* **Files Modified:**
  * `ahrs_loc_handler.hpp`: Removed method declaration
  * `ahrs_loc_handler.cpp`: Updated implementation
  * `Tests/python/python_binding/localization_pybind_module.cpp`: Updated Python bindings
  * `Tests/python/python_binding/carpose_offline_calculation.py`: Updated test script
  * `wrapper/control_api_wrapper.cpp`: Updated API wrapper implementation
