import copy
from pathlib import Path
import os
import sys
import matplotlib.pyplot as plt
import json
import numpy as np
import argparse
import Classes

# Add the path to the Python bindings
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'python_binding/build'))
try:
    import control_module  # This is our localization Python binding
except ImportError:
    print("ERROR: Failed to import control_module. Make sure the Python bindings are built properly.")
    print("Run the rebuild.sh script in the Tests/python/python_binding directory first.")
    sys.exit(1)

def find_closest_sample(data, timestamp, timestamp_col="time_seconds", max_diff=0.1):
    """
    Find the closest data sample to the given timestamp.
    
    Args:
        data: DataFrame containing the data
        timestamp: Target timestamp to find
        timestamp_col: Column name for timestamps in the data
        max_diff: Maximum allowed time difference (in seconds)
    
    Returns:
        DataFrame row or None if no sample is found within max_diff
    """
    if data.empty:
        return None
    
    # Find the closest index
    time_diffs = np.abs(data[timestamp_col] - timestamp)
    closest_idx = np.argmin(time_diffs)
    min_diff = time_diffs.iloc[closest_idx]
    
    if min_diff <= max_diff:
        return data.iloc[closest_idx]
    else:
        return None

def interpolate_data(data, timestamp, timestamp_col="time_seconds", value_col="data_value"):
    """
    Interpolate data at the given timestamp.
    
    Args:
        data: DataFrame containing the data
        timestamp: Target timestamp for interpolation
        timestamp_col: Column name for timestamps in the data
        value_col: Column name for values in the data
    
    Returns:
        Interpolated value or None if timestamp is out of bounds
    """
    if data.empty:
        return None
    
    # Check if timestamp is within data range
    if timestamp < data[timestamp_col].iloc[0] or timestamp > data[timestamp_col].iloc[-1]:
        return None
    
    # Find the samples before and after the timestamp
    next_idx = data[data[timestamp_col] > timestamp].index[0]
    if next_idx == 0:
        return data[value_col].iloc[0]
    
    prev_idx = next_idx - 1
    
    # Linear interpolation
    t1 = data[timestamp_col].iloc[prev_idx]
    t2 = data[timestamp_col].iloc[next_idx]
    v1 = data[value_col].iloc[prev_idx]
    v2 = data[value_col].iloc[next_idx]
    
    return v1 + (v2 - v1) * (timestamp - t1) / (t2 - t1)

def save_car_pose_to_file(car_pose, output_path):
    """
    Save calculated car pose to a CSV file.
    
    Args:
        car_pose: Dictionary with timestamp, x, y, yaw arrays
        output_path: Path to save the CSV file
    """
    with open(output_path, "w") as f:
        f.write("timestamp,x,y,yaw\n")
        for i in range(len(car_pose["timestamp"])):
            f.write(f"{car_pose['timestamp'][i]},{car_pose['x'][i]},{car_pose['y'][i]},{car_pose['yaw'][i]}\n")
    
    print(f"Saved calculated car pose to {output_path}")

def main():
    """
    Main function for offline car pose calculation using Python bindings.
    This script implements B7 & B8 items from the localization separation plan:
    B7: Create Python test script for loading AI-driver trip data
    B8: Implement synchronous sensor data processing using timestamps
    """
    # Setup argument parser - same as analyze_localization.py
    parser = argparse.ArgumentParser(description='Calculate car pose using localization module and compare with GPS data.')
    parser.add_argument('--trip_path', type=str, 
                        default='/home/eranvertz/imagry/trips/RTK_Haifa/2024-10-28T16_02_54​/',
                        help='Path to the trip data directory')
    parser.add_argument('--metric_type', type=str, choices=['Driving', 'Time', 'Turning'], 
                        default='Driving',
                        help='Type of metric to use for comparison (default: Driving)')
    parser.add_argument('--config_path', type=str,
                        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../'),
                        help='Path to configuration files (vehicle_config.json and control_config.json)')
    parser.add_argument('--visualize', action='store_true',
                        help='Enable visualization of results')
    args = parser.parse_args()
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Set up script configuration from command-line arguments
    script_config = {
        "aidriver_trip_path": args.trip_path,
        "metric_type": args.metric_type,
        "config_path": args.config_path,
        "visualize": args.visualize
    }
    
    # Initialize the Trip object to load data from experiment
    aidriver_trip_path = Path(script_config["aidriver_trip_path"])
    print('Loading trip data from ' + str(aidriver_trip_path))
    trip_obj = Classes.Trip(aidriver_trip_path)
    print('Trip data loaded successfully')
    
    # Print some basic trip information
    print(f"Trip duration: {trip_obj.common_time[-1]:.2f} seconds")
    print(f"Number of samples: {len(trip_obj.common_time)}")
    print(f"Sample rate: {len(trip_obj.common_time) / trip_obj.common_time[-1]:.2f} Hz")
    
    # Initialize the AHRSLocHandler with configuration files
    vehicle_config_path = os.path.join(script_config["config_path"], "vehicle_config.json")
    control_config_path = os.path.join(script_config["config_path"], "control_config.json")
    
    if not os.path.exists(vehicle_config_path) or not os.path.exists(control_config_path):
        print(f"ERROR: Configuration files not found in {script_config['config_path']}")
        print(f"Looking for: {vehicle_config_path} and {control_config_path}")
        print("Please provide the correct path using --config_path")
        sys.exit(1)
    
    print(f"Initializing AHRSLocHandler with config files from {script_config['config_path']}")
    loc_handler = control_module.AHRSLocHandler(vehicle_config_path, control_config_path)
    
    # Prepare data structures for storing car pose
    car_pose = {
        "timestamp": [],
        "x": [],
        "y": [],
        "yaw": []
    }
    
    print("Processing sensor data chronologically...")
    
    # B8: Implement synchronous sensor data processing using timestamps
    # Process all sensor data in chronological order based on timestamps
    
    # Prepare data structures for all sensor streams and their timestamps
    sensor_streams = {
        "imu": {"data": trip_obj.imu, "timestamp": "time_stamp", "processed_idx": 0},
        "speed": {"data": trip_obj.speed, "timestamp": "time_stamp", "processed_idx": 0},
        "steering": {"data": trip_obj.steering, "timestamp": "time_stamp", "processed_idx": 0}
    }
    
    # Convert all timestamps to seconds from the start of the trip
    start_time = min([sensor_streams[stream]["data"][sensor_streams[stream]["timestamp"]].iloc[0] 
                       for stream in sensor_streams])
    
    for stream in sensor_streams:
        sensor_streams[stream]["data"]["time_seconds"] = sensor_streams[stream]["data"][sensor_streams[stream]["timestamp"]] - start_time
    
    # Process all sensor data in chronological order
    current_time = 0
    time_step = 0.01  # 10ms processing interval
    max_time = max([sensor_streams[stream]["data"]["time_seconds"].iloc[-1] for stream in sensor_streams])
    
    print(f"Processing data from 0 to {max_time:.2f} seconds with {time_step*1000:.1f}ms steps")
    
    # Create ImuSample object for use with UpdateIMU
    def create_imu_sample(row):
        imu_sample = control_module.ImuSample()
        imu_sample.acc = control_module.Vec3d(row["acc_x"], row["acc_y"], row["acc_z"])
        imu_sample.gyro = control_module.Vec3d(row["gyro_x"], row["gyro_y"], row["gyro_z"])
        return imu_sample
    
    try:
        # Process the sensor data chronologically
        while current_time <= max_time:
            # Process IMU data
            imu_data = sensor_streams["imu"]["data"]
            imu_idx = sensor_streams["imu"]["processed_idx"]
            
            while (imu_idx < len(imu_data) and 
                   imu_data["time_seconds"].iloc[imu_idx] <= current_time):
                # Process this IMU sample
                row = imu_data.iloc[imu_idx]
                imu_sample = create_imu_sample(row)
                loc_handler.UpdateIMU(imu_sample, row["time_seconds"])
                imu_idx += 1
            
            sensor_streams["imu"]["processed_idx"] = imu_idx
            
            # Process speed data
            speed_data = sensor_streams["speed"]["data"]
            speed_idx = sensor_streams["speed"]["processed_idx"]
            
            while (speed_idx < len(speed_data) and 
                   speed_data["time_seconds"].iloc[speed_idx] <= current_time):
                # Process this speed sample (convert from km/h to m/s)
                row = speed_data.iloc[speed_idx]
                speed_mps = row["data_value"] / 3.6
                loc_handler.UpdateSpeed(speed_mps, row["time_seconds"])
                speed_idx += 1
            
            sensor_streams["speed"]["processed_idx"] = speed_idx
            
            # Process steering data
            steering_data = sensor_streams["steering"]["data"]
            steering_idx = sensor_streams["steering"]["processed_idx"]
            
            while (steering_idx < len(steering_data) and 
                   steering_data["time_seconds"].iloc[steering_idx] <= current_time):
                # Process this steering sample
                row = steering_data.iloc[steering_idx]
                steering_rad = row["data_value"] * np.pi / 180  # Convert from degrees to radians if needed
                loc_handler.UpdateSteeringWheel(steering_rad, row["time_seconds"])
                steering_idx += 1
            
            sensor_streams["steering"]["processed_idx"] = steering_idx
            
            # Update position calculation
            try:
                loc_handler.UpdatePosition(current_time)
                
                # Store the current pose
                position = loc_handler.GetPosition()
                heading = loc_handler.GetVehicleHeading()
                
                car_pose["timestamp"].append(current_time)
                car_pose["x"].append(position[0])
                car_pose["y"].append(position[1])
                car_pose["yaw"].append(heading)
            except Exception as e:
                print(f"Warning: Failed to update position at time {current_time}s: {e}")
            
            # Move to the next time step
            current_time += time_step
            
            # Print progress every second
            if int(current_time * 100) % 100 == 0:
                print(f"Processed {current_time:.2f}s / {max_time:.2f}s ({(current_time/max_time*100):.1f}%)")
                # Also print the current vehicle state
                if len(car_pose["x"]) > 0:
                    last_idx = len(car_pose["x"]) - 1
                    print(f"  Position: ({car_pose['x'][last_idx]:.2f}, {car_pose['y'][last_idx]:.2f}), " + 
                          f"Heading: {car_pose['yaw'][last_idx] * 180 / np.pi:.2f}°")
        
        print("Finished processing all sensor data")
        
        # Save the calculated car pose to a CSV file
        output_dir = Path(script_config["aidriver_trip_path"]) / "calculated_pose"
        output_dir.mkdir(exist_ok=True)
        
        output_file = output_dir / "calculated_car_pose.csv"
        save_car_pose_to_file(car_pose, output_file)
        
        # Create a README file with information about the calculation
        readme_file = output_dir / "README.txt"
        with open(readme_file, "w") as f:
            f.write("Car Pose Calculation Information\n")
            f.write("===============================\n\n")
            f.write(f"Trip data source: {script_config['aidriver_trip_path']}\n")
            f.write(f"Calculation timestamp: {np.datetime64('now')}\n")
            f.write(f"Total processed time: {max_time:.2f} seconds\n")
            f.write(f"Number of data points: {len(car_pose['timestamp'])}\n")
            f.write(f"Processing interval: {time_step*1000:.1f} ms\n\n")
            f.write("File format: timestamp,x,y,yaw\n")
            f.write("- timestamp: seconds from start of recording\n")
            f.write("- x,y: position in meters\n")
            f.write("- yaw: heading in radians\n")
        
        print(f"Saved calculation metadata to {readme_file}")
        
        # Visualize the results if requested
        if script_config["visualize"]:
            plt.figure(figsize=(10, 8))
            plt.plot(car_pose["x"], car_pose["y"], 'b-', label='Calculated Path')
            plt.plot(trip_obj.N, trip_obj.E, 'r--', label='GPS Reference')
            plt.grid(True)
            plt.axis('equal')
            plt.xlabel('X [m]')
            plt.ylabel('Y [m]')
            plt.title('Calculated Car Pose vs GPS Reference')
            plt.legend()
            plt.savefig(output_dir / "car_pose_comparison.png")
            plt.show()
    
    except Exception as e:
        print(f"Error during processing: {e}")
        import traceback
        traceback.print_exc()
    
    print("Script completed. Ready for further instructions.")
    
if __name__ == '__main__':
    main()
