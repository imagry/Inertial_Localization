# Localization System Documentation

## 1. Workflow Setup and Repository Organization

### 1.1 Initial Setup
- Created a standalone repository for the localization system by forking from the original control and localization repository
- Established proper `.gitignore` configuration to exclude build artifacts and third-party libraries from version control
- Set up project planning with `localization_separation_plan.md` focusing on Objective B: Standalone Localization Repository with API
- Created `.copilot-instructions.md` to maintain context during the refactoring process

### 1.2 Current Repository Structure
The repository currently maintains the original structure from the combined control and localization system, including:
- Core files for localization functionality
- Some control-related components that will be removed during refactoring
- Dependencies on third-party libraries (Eigen, JSON, etc.)
- Build system based on CMake

### 1.3 Existing Python Binding Framework
- The repository already includes a Python binding framework using pybind11
- Located in `Tests/python/python_binding/`
- Current bindings are primarily for control components (StanleyController, etc.)
- No dedicated bindings yet for localization components
- Uses CMake for building the Python extension module

## 2. Localization System Data Flow

### 2.1 Core Components

#### 2.1.1 Primary Localization Handler (`ahrs_loc_handler.cpp/hpp`)
- **Role**: Main entry point and coordinator for the localization system
- **Key Classes**: `AHRSLocHandler`
- **Initialization**:
  - Takes vehicle and control configuration (from JSON files)
  - Initializes localization components (AHRS, ShortTermLocalization)
- **Key Methods**:
  - `UpdatePosition`: Updates position based on current state
  - `UpdateIMU`: Processes IMU sensor data
  - `UpdateSpeed`: Updates vehicle speed information
  - `UpdateRearRightSpeed`/`UpdateRearLeftSpeed`: Updates wheel odometry data
  - `UpdateSteeringWheel`: Updates steering wheel angle measurements
  - `UpdateHeading`: Updates heading information
  - `GetLoc`: Retrieves localization object
- **Dependencies**:
  - Uses `ShortTermLocalization` for position tracking
  - Uses `AttitudeEstimator` (AHRS) for orientation
  - Uses `Delay` for handling time-delayed data
  - Uses `ControlDebugStates` for debugging

#### 2.1.2 Short-Term Localization (`Utils/short_term_localization.cpp/hpp`)
- **Role**: Handles short-term position and velocity estimation
- **Key Classes**: `ShortTermLocalization`, `LocState`
- **Initialization**:
  - Configures based on heading and speed estimation modes
  - Takes vehicle wheelbase as parameter
- **Key Methods**:
  - `State`: Returns current localization state
  - `UpdateDelta`: Updates steering angle (converted from steering wheel angle)
  - `UpdateHeading`: Updates vehicle heading
  - Various position update functions
- **Functionality**:
  - Processes data from IMU and other sensors
  - Uses steering wheel angle (converted to steering/tire angle) for vehicle kinematics
  - Performs dead reckoning calculations
  - Updates position and velocity estimates based on vehicle kinematic model

#### 2.1.3 Attitude and Heading Reference System (`Utils/AHRS.cpp/hpp`)
- **Role**: Determines vehicle orientation and heading
- **Key Classes**: `AttitudeEstimator`, `SegmentIMURecording`
- **Initialization**:
  - Takes sample rate, gain parameters
  - Configures coordinate system (NED - North, East, Down)
- **Functionality**:
  - Fuses gyroscope and accelerometer data
  - Maintains attitude estimation
  - Provides heading information for localization
  - Uses quaternion representation for orientation

#### 2.1.4 Speed Estimation (`Utils/SpeedEstimators.cpp/hpp`)
- **Role**: Provides accurate vehicle speed estimation using sensor fusion
- **Key Classes**: `SpeedEstimator` (interface), `KalmanFilter` (implementation)
- **Initialization**:
  - Configures based on sensor noise parameters
  - Takes initial speed and bias values
- **Key Methods**:
  - `UpdateIMU`: Process new IMU data
  - `UpdateRearSpeeds`: Process wheel odometry data
  - `UpdateState`: Estimate new vehicle state
  - `GetEstimatedSpeed`: Retrieve current speed estimate
- **Functionality**:
  - Fuses IMU and wheel odometry data
  - Uses Kalman filtering for optimal speed estimation
  - Handles sensor noise and bias

#### 2.1.5 Debug and Monitoring (`Utils/localization_debug_states.cpp/hpp`)
- **Role**: Provides debugging and state monitoring capabilities
- **Key Classes**: `ControlDebugStates`
- **Initialization**:
  - Takes debugging directory information
  - Configures based on localization mode
- **Functionality**:
  - Creates debug directories if needed
  - Logs internal states of the localization system
  - Saves data to files for analysis
  - Supports performance evaluation and visualization

#### 2.1.6 Control API Interface (`ControlAPI.cpp/hpp`)
- **Role**: Main API interface that integrates localization with the control system
- **Key Classes**: `ControlAPI`
- **Initialization**:
  - Takes vehicle and control configuration (from JSON files)
  - Creates an instance of `AHRSLocHandler` through the `GetAHRSLocHandlerInstance` factory method
- **Localization-Related Components**:
  - Contains `std::shared_ptr<AHRSLocHandler> localization_handler` for accessing localization functionality
- **Key Methods Using Localization**:
  - Uses `localization_handler->GetLoc().State()` to retrieve position and orientation
  - Uses `localization_handler->GetPosition()` for current position
  - Uses `localization_handler->GetVehicleHeading()` for current heading
  - Uses `localization_handler->GetDelay()` for time-delayed data handling
- **Role in the System**:
  - Serves as the main interface between control components and localization
  - Provides access to localization data for motion planning and control calculations
  - Primary point for integrating localization into the broader vehicle control system

### 2.2 Data Flow Diagram

```
                   ┌───────────────────┐
                   │  Sensor Inputs    │
                   │  - IMU            │
                   │  - Wheel Odometry │
                   │  - Steering Wheel │
                   │  - Speed          │
                   └─────────┬─────────┘
                             │
                             ▼
┌────────────────────┐    ┌─────────────────────────────────────────────┐
│    ControlAPI      │◄───┤         ahrs_loc_handler.cpp/hpp            │
│    Interface       │    │         Main Localization Handler           │
└────────────────────┘    └─┬─────────────────┬───────────────┬─────────┘
                            │                 │               │
                            ▼                 ▼               ▼
                  ┌─────────────────┐ ┌─────────────┐ ┌─────────────────┐
                  │  AHRS.cpp/hpp   │ │SpeedEstima- │ │short_term_      │
                  │  Attitude &     │ │tors.cpp/hpp │ │localization.cpp │
                  │  Heading        │ │Speed        │ │Position &       │
                  │  Reference      │ │Estimation   │ │Velocity         │
                  └────────┬────────┘ └─────┬───────┘ └────────┬────────┘
                           │                │                  │
                           └────────────────┼──────────────────┘
                                            │
                                            ▼
                                ┌───────────────────────┐
                                │ localization_debug_states  │
                                │ Debugging & Logging   │
                                └───────────┬───────────┘
                                            │
                                            ▼
                                ┌───────────────────────┐
                                │   Output Interface    │
                                │   - Position          │
                                │   - Velocity          │
                                │   - Orientation       │
                                └───────────────────────┘
```

## 3. Python Binding Requirements and Planning

### 3.1 Current Python Binding Structure
- Located in `Tests/python/python_binding/`
- Uses pybind11 for C++ to Python interface
- Current implementation in `localization_pybind_module.cpp` only binds control-related classes
- Build system uses CMake to generate Python extension module

### 3.2 Task B5: Python Binding Requirements for Localization

#### 3.2.1 Key Classes to Expose
- **`AHRSLocHandler`**: Main localization handler
  - Constructor with JSON configuration
  - Methods for updating position, IMU, and speed data
  - Methods for retrieving localization state
- **`ShortTermLocalization`**: Core localization component
  - Methods for accessing localization state
  - Configuration options (if applicable)
- **`AttitudeEstimator`**: AHRS component
  - Methods for updating and accessing orientation information
  - Access to quaternion representation
- **`SpeedEstimator`/`KalmanFilter`**: Vehicle speed estimation
  - Methods for processing IMU and wheel odometry data
  - Access to speed estimates and state information
- **`ControlAPI`** (localization-related parts only):
  - Access to the localization handler
  - Methods to retrieve position, heading, and velocity
  - Handling of time-delayed data through the delay compensation system

#### 3.2.2 Data Types to Support
- JSON configuration via Python dictionaries or JSON strings
- Sensor data structures (ImuSample, etc.)
- Timestamp handling (PreciseSeconds)
- Vector/Matrix data from Eigen

### 3.3 Implementation Plan for Task B5
1. Extend `localization_pybind_module.cpp` to include localization bindings
2. Create new Python extension specifically for localization components
3. Implement comprehensive test cases for the Python bindings
4. Ensure proper memory management and type conversion

### 3.4 Future Tasks
- Ensure compatibility with existing sensor API (Task B6)
- Create Python test script for loading AI-driver trip data (Task B7)
- Implement synchronous sensor data processing using timestamps (Task B8)
- Document API usage and examples (Task B9)
- Remove control-related unused functionality (Task B10)
