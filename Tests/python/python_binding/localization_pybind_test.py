#!/usr/bin/env python3
# Copyright (c) 2024 Imagry. All Rights Reserved.
# Unauthorized copying of this file, via any medium is strictly prohibited.
# Proprietary and confidential.
# Created on July 24 2025

import os
import sys
import time

# Add the directory containing the module to Python's path
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

# Try to import the localization_pybind_module
try:
    from build import localization_pybind_module
    print("Successfully imported localization_pybind_module")
except ImportError as e:
    print(f"Error importing localization_pybind_module: {e}")
    print("Make sure the module has been built correctly")
    print("You may need to run 'cd Tests/python/python_binding && ./rebuild.sh'")
    sys.exit(1)

def test_init():
    """Test the initialization of the module and classes"""
    print("\nTesting module initialization...")
    
    # Test Vec3d class
    try:
        vec = localization_pybind_module.Vec3d()
        vec.x = 1.0
        vec.y = 2.0
        vec.z = 3.0
        print(f"Created Vec3d: ({vec.x}, {vec.y}, {vec.z}) - Success!")
    except Exception as e:
        print(f"Vec3d test failed: {e}")
        return False
    
    # Test ImuSample class
    try:
        imu = localization_pybind_module.ImuSample()
        imu.time_stamp = 123.456
        print(f"Created ImuSample with timestamp: {imu.time_stamp} - Success!")
    except Exception as e:
        print(f"ImuSample test failed: {e}")
        return False
    
    # Get paths to config files
    vehicle_config_path, control_config_path = get_config_paths()
    
    # Check if config files exist
    if not os.path.exists(vehicle_config_path):
        print(f"Error: vehicle_config.json not found at {vehicle_config_path}")
        return False
    
    if not os.path.exists(control_config_path):
        print(f"Error: localization_config.json not found at {control_config_path}")
        return False
    
    print(f"Config files found at:")
    print(f"  - {vehicle_config_path}")
    print(f"  - {control_config_path}")
    
    # Test AHRSLocHandler initialization
    try:
        handler = localization_pybind_module.AHRSLocHandler(vehicle_config_path, control_config_path)
        print("Created AHRSLocHandler successfully!")
        return True
    except Exception as e:
        print(f"AHRSLocHandler initialization failed: {e}")
        return False

def get_config_paths():
    """Get the absolute paths to the configuration files."""
    repo_root = os.path.abspath(os.path.join(script_dir, "../../../"))
    vehicle_config_path = os.path.join(repo_root, "vehicle_config.json")
    control_config_path = os.path.join(repo_root, "localization_config.json")
    return vehicle_config_path, control_config_path

def test_update_imu():
    """Test the UpdateIMU method"""
    print("\nTesting UpdateIMU method...")
    
    # Get paths to config files
    vehicle_config_path, control_config_path = get_config_paths()
    
    try:
        # Initialize AHRSLocHandler
        handler = localization_pybind_module.AHRSLocHandler(vehicle_config_path, control_config_path)
        
        # Create Vec3d objects for the IMU data
        acc = localization_pybind_module.Vec3d()
        acc.x = 0.1
        acc.y = 0.2
        acc.z = 0.3
        
        acc_b = localization_pybind_module.Vec3d()
        acc_b.x = 0.4
        acc_b.y = 0.5
        acc_b.z = 0.6
        
        gyro = localization_pybind_module.Vec3d()
        gyro.x = 0.01
        gyro.y = 0.02
        gyro.z = 0.03
        
        gyro_b = localization_pybind_module.Vec3d()
        gyro_b.x = 0.04
        gyro_b.y = 0.05
        gyro_b.z = 0.06
        
        mag = localization_pybind_module.Vec3d()
        mag.x = 10.0
        mag.y = 20.0
        mag.z = 30.0
        
        # Create an IMU sample
        imu = localization_pybind_module.ImuSample()
        imu.time_stamp = time.time()
        imu.acc_ = acc
        imu.acc_b_ = acc_b
        imu.gyro_ = gyro
        imu.gyro_b_ = gyro_b
        imu.mag_ = mag
        imu.pitch_ = 0.01
        imu.roll_ = 0.02
        imu.yaw_ = 0.03
        
        # Call UpdateIMU
        current_time = time.time()
        handler.UpdateIMU(imu, current_time)
        
        print("UpdateIMU call succeeded!")
        
        # Now try to get position to verify the handler is working
        position = handler.GetPosition()
        print(f"Current position after IMU update: {position}")
        
        # Get heading to verify the handler is working
        heading = handler.GetVehicleHeading()
        print(f"Current heading after IMU update: {heading}")
        
        return True
    except Exception as e:
        print(f"UpdateIMU test failed: {e}")
        return False

def test_update_wheel_speeds():
    """Test the UpdateRearRightSpeed and UpdateRearLeftSpeed methods"""
    print("\nTesting wheel speed update methods...")
    
    # Get paths to config files
    vehicle_config_path, control_config_path = get_config_paths()
    
    try:
        # Initialize AHRSLocHandler
        handler = localization_pybind_module.AHRSLocHandler(vehicle_config_path, control_config_path)
        
        # Call UpdateRearRightSpeed
        right_speed = 5.2  # 5.2 m/s
        current_time = time.time()
        handler.UpdateRearRightSpeed(right_speed, current_time)
        print("UpdateRearRightSpeed call succeeded!")
        
        # Call UpdateRearLeftSpeed
        left_speed = 4.8  # 4.8 m/s
        handler.UpdateRearLeftSpeed(left_speed, current_time)
        print("UpdateRearLeftSpeed call succeeded!")
        
        # Get position after updating wheel speeds
        position = handler.GetPosition()
        print(f"Current position after wheel speed updates: {position}")
        
        return True
    except Exception as e:
        print(f"Wheel speed update test failed: {e}")
        return False

def test_reset_vehicle_state():
    """Test the ResetVehicleState method"""
    print("\nTesting ResetVehicleState method...")
    
    # Get paths to config files
    vehicle_config_path, control_config_path = get_config_paths()
    
    try:
        # Initialize AHRSLocHandler
        handler = localization_pybind_module.AHRSLocHandler(vehicle_config_path, control_config_path)
        
        # Get initial position
        initial_position = handler.GetPosition()
        print(f"Initial position: {initial_position}")
        
        # Create a state vector with a different position [10.0, 20.0]
        # The exact format may vary based on the implementation, but typically it's
        # [x, y, heading, ...]
        new_state = [10.0, 20.0, 0.5, 0.0]
        current_time = time.time()
        
        # Reset the vehicle state
        handler.ResetVehicleState(current_time, new_state)
        print(f"Vehicle state reset with new values: {new_state}")
        
        # Get the position after reset
        new_position = handler.GetPosition()
        print(f"Position after reset: {new_position}")
        
        # Get heading after reset
        new_heading = handler.GetVehicleHeading()
        print(f"Heading after reset: {new_heading}")
        
        return True
    except Exception as e:
        print(f"ResetVehicleState test failed: {e}")
        return False

def main():
    print("="*80)
    print("LOCALIZATION PYTHON BINDINGS BASIC TEST")
    print("="*80)
    
    tests = [
        ("Module initialization", test_init),
        ("UpdateIMU method", test_update_imu),
        ("Update wheel speeds", test_update_wheel_speeds),
        ("Reset vehicle state", test_reset_vehicle_state)
    ]
    
    all_success = True
    
    for test_name, test_func in tests:
        print(f"\nRunning test: {test_name}")
        start_time = time.time()
        success = test_func()
        elapsed_time = time.time() - start_time
        
        if success:
            print(f"{test_name}: SUCCESS ({elapsed_time:.6f}s)")
        else:
            print(f"{test_name}: FAILURE ({elapsed_time:.6f}s)")
            all_success = False
    
    print("\n" + "="*80)
    if all_success:
        print("OVERALL TEST RESULT: SUCCESS")
    else:
        print("OVERALL TEST RESULT: FAILURE (some tests failed)")
    print("="*80)
    
    return 0 if all_success else 1

if __name__ == "__main__":
    sys.exit(main())
