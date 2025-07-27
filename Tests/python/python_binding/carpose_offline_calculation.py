import os
import sys
import argparse
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from tqdm import tqdm

# Add the path to import Classes module
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import Classes

# Add the path to import control_module (Python bindings)
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'build'))
try:
    import control_module
    print("Successfully imported control_module")
except ImportError as e:
    print(f"Failed to import control_module: {e}")
    print("Make sure you have built the Python bindings properly")
    print("You may need to run the build script in Tests/python/python_binding/")
    sys.exit(1)

def main():
    """
    Main function for loading AI-driver trip data.
    This script implements part of B7 from the localization separation plan:
    B7: Create Python test script for loading AI-driver trip data
    """
    # Setup argument parser with only necessary arguments
    parser = argparse.ArgumentParser(description='Load AI-driver trip data using Classes.Trip.')
    parser.add_argument('--trip_path', type=str, 
                        default='/home/eranvertz/imagry/trips/NAHARIA/2025-05-21T11_52_50/',
                        help='Path to the trip data directory')
    parser.add_argument('--visualize', action='store_true',
                        help='Visualize the car trajectory after processing')
    parser.add_argument('--output_dir', type=str, default='./results',
                        help='Directory to save results and plots')
    args = parser.parse_args()
    
    # Initialize the Trip object to load data from experiment
    aidriver_trip_path = Path(args.trip_path)
    print('Loading trip data from ' + str(aidriver_trip_path))
    trip_obj = Classes.Trip(aidriver_trip_path)
    print('Trip data loaded successfully')
    
    # Print some basic trip information
    print(f"Trip duration: {trip_obj.common_time[-1]:.2f} seconds")
    print(f"Number of samples: {len(trip_obj.common_time)}")
    print(f"Sample rate: {len(trip_obj.common_time) / trip_obj.common_time[-1]:.2f} Hz")
    
    # Sort localization inputs chronologically
    trip_obj.sort_localization_inputs(print_results=True)
    
    # Set up configuration files for the localization module
    def get_config_paths():
        """Get the absolute paths to the configuration files."""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        repo_root = os.path.abspath(os.path.join(script_dir, "../../../"))
        vehicle_config_path = os.path.join(repo_root, "vehicle_config.json")
        control_config_path = os.path.join(repo_root, "control_config.json")
        return vehicle_config_path, control_config_path

    # Get the paths to the configuration files
    vehicle_config_path, control_config_path = get_config_paths()
    
    # Check if configuration files exist
    if not os.path.exists(vehicle_config_path) or not os.path.exists(control_config_path):
        print(f"ERROR: Configuration files not found")
        print(f"Looking for: {vehicle_config_path} and {control_config_path}")
        sys.exit(1)
    
    print(f"Initializing AHRSLocHandler with configuration files:")
    print(f"  - {vehicle_config_path}")
    print(f"  - {control_config_path}")
    
    # Initialize the AHRSLocHandler with the config files
    try:
        loc_handler = control_module.AHRSLocHandler(vehicle_config_path, control_config_path)
    except Exception as e:
        print(f"Error initializing AHRSLocHandler: {e}")
        sys.exit(1)
    
    # Initialize vehicle state (position at origin, zero heading)
    initial_time = 0.0  # Start at time 0
    initial_state = [0.0, 0.0, 0.0, 0.0]  # [x, y, heading, velocity]
    loc_handler.ResetVehicleState(initial_time, initial_state)
    
    # Prepare for tracking estimated car pose
    car_pose = {
        "timestamp": [],
        "x": [],
        "y": [],
        "yaw": []
    }
    
    print("Initial vehicle state reset to origin (0,0,0)")
    print("Starting to process sensor readings...")
    
    # Iterate through sorted sensor data and process each reading
    print("\nProcessing sensor readings chronologically...")
    for i, reading in tqdm(enumerate(trip_obj.sorted_localization_inputs), total=len(trip_obj.sorted_localization_inputs), desc="Processing sensors"):
        timestamp = reading['timestamp']
        sensor_id = reading['sensor_id']
        data = reading['data']
        
        # Call the appropriate method based on sensor type
        if sensor_id == "IMU":
            # Create IMU sample and update the localization
            imu_sample = control_module.ImuSample()
            imu_sample.time_stamp = timestamp
            # Get acceleration data from IMU row
            # Create Vec3d objects and set their properties separately
            acc_vec = control_module.Vec3d()
            # Use exact field names from the header
            acc_vec.x = float(data["x_acc"])
            acc_vec.y = float(data["y_acc"])
            acc_vec.z = float(data["z_acc"])
            imu_sample.acc_ = acc_vec
            
            # Create acceleration body frame vector with bias values
            acc_b_vec = control_module.Vec3d()
            # Use bias values if available, otherwise use the same values
            acc_b_vec.x = float(data["x_acc_bias"]) 
            acc_b_vec.y = float(data["y_acc_bias"]) 
            acc_b_vec.z = float(data["z_acc_bias"]) 
            imu_sample.acc_b_ = acc_b_vec
            
            # Create gyro vector
            gyro_vec = control_module.Vec3d()
            # Use correct field names
            gyro_vec.x = float(data["x_gyro"]) * np.pi/180  # Convert to radians
            gyro_vec.y = float(data["y_gyro"]) * np.pi/180  # Convert to radians
            gyro_vec.z = float(data["z_gyro"]) * np.pi/180  # Convert to radians
            imu_sample.gyro_ = gyro_vec
            
            # Create gyro body frame vector with bias values
            gyro_b_vec = control_module.Vec3d()
            # Use bias values if available, otherwise use the same values
            gyro_b_vec.x = float(data["x_gyro_bias"])  * np.pi/180  # Convert to radians
            gyro_b_vec.y = float(data["y_gyro_bias"])  * np.pi/180  # Convert to radians
            gyro_b_vec.z = float(data["z_gyro_bias"])  * np.pi/180  # Convert to radians
            imu_sample.gyro_b_ = gyro_b_vec
            
            # Create magnetometer vector
            mag_vec = control_module.Vec3d()
            # Use magnetometer fields if available, otherwise set to zero
            mag_vec.x = float(data["x_mag"]) 
            mag_vec.y = float(data["y_mag"]) 
            mag_vec.z = float(data["z_mag"]) 
            imu_sample.mag_ = mag_vec
            
            # Set Euler angles - using the exact field names from the header
            imu_sample.pitch_ = float(data["pitch"]) * np.pi/180  # Convert to radians
            imu_sample.roll_ = float(data["roll"]) * np.pi/180  # Convert to radians
            imu_sample.yaw_ = float(data["yaw"]) * np.pi/180  # Convert to radians
            
            # Update localization with IMU data - pass both sample and timestamp
            loc_handler.UpdateIMU(imu_sample, timestamp)
            
        elif sensor_id == "speed":
            # Convert speed from km/h to m/s
            speed_mps = data["data_value"] / 3.6
            loc_handler.UpdateSpeed(speed_mps, timestamp)
            
        elif sensor_id == "steering":
            # Steering angle in radians
            steering_rad = data["data_value"] * np.pi / 180  # Convert from degrees to radians
            loc_handler.UpdateSteeringWheel(steering_rad, timestamp)
            
        elif sensor_id == "left_rear_wheel_speed":
            # Update left rear wheel speed
            wheel_speed_mps = data["data_value"] / 3.6  # Convert from km/h to m/s
            loc_handler.UpdateRearLeftSpeed(wheel_speed_mps, timestamp)
            
        elif sensor_id == "right_rear_wheel_speed":
            # Update right rear wheel speed
            wheel_speed_mps = data["data_value"] / 3.6  # Convert from km/h to m/s
            loc_handler.UpdateRearRightSpeed(wheel_speed_mps, timestamp)
            
        # Store vehicle pose estimate (do this periodically to reduce computation overhead)
        position = loc_handler.GetPosition()  # Use correct method name from ahrs_loc_handler.hpp
        yaw = loc_handler.GetVehicleHeading()  # Use correct method name from ahrs_loc_handler.hpp
        
        car_pose["timestamp"].append(timestamp)
        car_pose["x"].append(position[0])
        car_pose["y"].append(position[1])
        car_pose["yaw"].append(yaw)
    
    print("Sensor data processing completed.")
    print(f"Generated {len(car_pose['timestamp'])} position estimates")
    print(f"Final position estimate: x={car_pose['x'][-1]:.2f}, y={car_pose['y'][-1]:.2f}, yaw={car_pose['yaw'][-1]:.2f}")
    
    # Define helper functions for car pose file parsing
    def parse_car_pose_file(car_pose_file_path):
        """Parse car pose file and return data dictionary."""
        with open(car_pose_file_path, "r") as f:
            lines = f.readlines()
        data = {"x":[], "y":[], "psi":[], "timestamp":[]}
        for line in lines[1:]:  # Skip header line
            splited = line.split(',')
            data["timestamp"].append(float(splited[0]))
            data["x"].append(float(splited[1]))
            data["y"].append(-float(splited[2]))  # Note: negating y
            data["psi"].append(-float(splited[3]) * np.pi / 180)  # Convert to radians
        return data

    def car_pose_path(trip_path):
        """Return path to car pose file."""
        for file in os.listdir(trip_path):
            if "car_pose" in file:
                return os.path.join(trip_path, file)
        raise FileNotFoundError(f"No car_pose file found in {trip_path}")

    def continuous_angle(angles, period):
        """Convert angles in a specific range to a sequence without boundary discontinuities."""
        angles = np.asarray(angles)
        
        # Determine points where the angle jumps due to periodicity
        jumps_up = (np.diff(angles) > 0.8 * period).astype(int)
        jumps_down = (np.diff(angles) < -0.8 * period).astype(int)
        jumps = jumps_up - jumps_down

        # Calculate the total accumulated shift
        steps = np.cumsum(jumps) * period
        continuous_seq = angles.copy()
        continuous_seq[1:] -= steps
        
        return continuous_seq
        
    # Try to load external car pose data if available
    external_car_pose = None
    try:
        car_pose_file = car_pose_path(args.trip_path)
        print(f"Found car pose file: {car_pose_file}")
        external_car_pose = parse_car_pose_file(car_pose_file)
    except FileNotFoundError as e:
        print(f"Warning: {e}")
    
    # Visualize the results if requested
    if args.visualize:
        # Create output directory if it doesn't exist
        os.makedirs(args.output_dir, exist_ok=True)
        
        # Plot the trajectory with GPS and our localization
        plt.figure(figsize=(12, 10))
        if False:# plot GPS
            plt.plot(trip_obj.N, trip_obj.E, 'g-', linewidth=2, label='GPS Track')
        plt.plot(car_pose["x"], car_pose["y"], 'b-', linewidth=2, label='Our Localization')
        
        # Also plot external car pose if available
        if external_car_pose is not None:
            carpose_x = np.array(external_car_pose['x'])
            carpose_y = np.array(external_car_pose['y'])
            # Adjust starting position to match others
            carpose_x -= carpose_x[0]
            carpose_y -= carpose_y[0]
            plt.plot(carpose_x, carpose_y, 'r-', linewidth=2, label='External Localization')
        plt.scatter(car_pose["x"][0], car_pose["y"][0], c='g', s=100, label='Start')
        plt.scatter(car_pose["x"][-1], car_pose["y"][-1], c='r', s=100, label='End')
        plt.grid(True)
        plt.axis('equal')
        plt.title('Vehicle Trajectory')
        plt.xlabel('East (m)')
        plt.ylabel('North (m)')
        plt.legend()
        
        # Save the trajectory plot
        plot_path = os.path.join(args.output_dir, 'vehicle_trajectory.png')
        plt.savefig(plot_path)
        print(f"Trajectory plot saved to {plot_path}")
        
        # Create a plot of the heading over time
        plt.figure(figsize=(12, 6))
        plt.plot(car_pose['timestamp'], np.array(car_pose['yaw']) * 180 / np.pi, 'b-', 
                label='Our Heading')
        
        # Add external car pose heading if available
        if external_car_pose is not None:
            # Convert to degrees for plotting
            ext_heading = np.array(external_car_pose['psi']) * 180 / np.pi
            plt.plot(external_car_pose['timestamp'], ext_heading, 'r-', 
                    label='External Heading')
        
        plt.title('Vehicle Heading')
        plt.xlabel('Time [s]')
        plt.ylabel('Heading [deg]')
        plt.legend()
        plt.grid(True)
        
        # Save the heading plot
        heading_path = os.path.join(args.output_dir, 'heading_comparison.png')
        plt.savefig(heading_path)
        print(f"Heading plot saved to {heading_path}")
        
        # Show all plots
        plt.show()
    
    print("Script completed. Trip data loaded and processed.")
    
    return trip_obj, loc_handler, car_pose
    
if __name__ == '__main__':
    trip_obj, loc_handler, car_pose = main()
