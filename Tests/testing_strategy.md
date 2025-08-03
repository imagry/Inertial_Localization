# Unit Testing Strategy for Inertial Localization System

Based on the review of the codebase and documentation, here's a comprehensive unit testing strategy using Google Test for the vehicle localization algorithm.

## 1. Core Components to Test

1. **AHRSLocHandler** - Main localization coordinator
2. **AttitudeEstimator (AHRS)** - Orientation and heading estimation
3. **ShortTermLocalization** - Position tracking
4. **SpeedEstimator** - Vehicle speed estimation

## 2. Detailed Testing Strategy

### 2.1 AHRSLocHandler Tests
The AHRSLocHandler is the central coordination point, so we'll test its ability to properly process inputs and coordinate between components.

#### Test Cases:
1. **Initialization Tests**
   - Test initialization with valid configuration files
   - Test initialization with invalid configuration (should handle gracefully)
   - Test default parameter initialization

2. **Sensor Update Tests**
   - Test IMU update processing
   - Test wheel speed update processing
   - Test steering wheel angle update processing
   - Test external heading update

3. **Integration Tests**
   - Test position calculation with simulated inputs
   - Test heading calculation with simulated inputs
   - Test correct propagation of sensor data through the system

### 2.2 AttitudeEstimator (AHRS) Tests
The AttitudeEstimator is responsible for determining vehicle orientation and heading.

#### Test Cases:
1. **Attitude Calculation Tests**
   - Test quaternion calculation from IMU data
   - Test Euler angle conversion accuracy
   - Test stability with noisy data

2. **Heading Estimation Tests**
   - Test heading calculation in different modes
   - Test heading continuity through turns
   - Test handling of magnetic interference

3. **IMU Processing Tests**
   - Test gyroscope integration
   - Test accelerometer processing
   - Test magnetometer processing

### 2.3 ShortTermLocalization Tests
The ShortTermLocalization handles vehicle position tracking using heading and velocity information.

#### Test Cases:
1. **Position Tracking Tests**
   - Test straight-line motion accuracy
   - Test curved path tracking
   - Test position updates with variable speed

2. **State Management Tests**
   - Test state initialization
   - Test state reset functionality
   - Test state retrieval and conversion

3. **Sensor Update Tests**
   - Test IMU data processing
   - Test wheel odometry processing
   - Test steering angle processing

### 2.4 SpeedEstimator Tests
The SpeedEstimator fuses sensor data to estimate vehicle speed.

#### Test Cases:
1. **Speed Calculation Tests**
   - Test speed calculation from wheel odometry
   - Test acceleration-based speed estimation
   - Test fusion of multiple sources

2. **Filtering Tests**
   - Test noise rejection
   - Test outlier rejection
   - Test smoothing during acceleration/deceleration

## 3. Implementation Plan

For each component, we'll create a dedicated test file following Google Test best practices:

1. **Test Fixtures** - Create fixture classes for each component to set up common test environments
2. **Mock Objects** - Use Google Mock to isolate components for unit testing
3. **Parameterized Tests** - Use parameterized tests for testing multiple configurations
4. **Data-Driven Tests** - Use test data files for testing with real-world data

## 4. Recommended Test File Structure

```
Tests/
├── test_ahrs_loc_handler.cpp  - Tests for the main localization handler
├── test_attitude_estimator.cpp - Tests for the AHRS component
├── test_short_term_localization.cpp - Tests for position tracking
├── test_speed_estimator.cpp - Tests for speed estimation
└── test_utils/ - Helper functions and test data
```

## 5. Examples of Test Implementation

Below are examples of how tests could be implemented for each component.
