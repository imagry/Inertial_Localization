import os
import sys
import argparse
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from tqdm import tqdm
from scipy.spatial.transform import Rotation

# Add the path to import Classes module
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import Classes
import Functions
import json
# Add the path to import localization_pybind_module (Python bindings)
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'build'))
try:
    import localization_pybind_module
    print("Successfully imported localization_pybind_module")
except ImportError as e:
    print(f"Failed to import localization_pybind_module: {e}")
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
                        default='data/backed_data_files/2025-05-21T11_52_50',
                        help='Path to the trip data directory')
    parser.add_argument('--visualize', action='store_true', default=False,
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
    
    # Try to load external car pose data if available
    external_car_pose = None
    try:
        car_pose_file = car_pose_path(args.trip_path)
        print(f"Found car pose file: {car_pose_file}")
        external_car_pose = parse_car_pose_file(car_pose_file)
    except FileNotFoundError as e:
        print(f"Warning: {e}")
    # zero the initial carpose heading 
    carpose_psi0 = external_car_pose["psi"][0]
    carpose_x0 = external_car_pose["x"][0]
    carpose_y0 = external_car_pose["y"][0]
    # carpose_speed_angle0 = np.arctan2(carpose_y0,
    carpose_t0 = external_car_pose["timestamp"][0]
    def rotate_trajectory(pts, angle):
        assert pts.shape[1] == 3, "path must be arranged as column (each point X,Y,psi is a row)"
        rot = Rotation.from_euler('ZYX', [angle, 0, 0], degrees=False).as_matrix()  # 3 X 3
        rot = rot[0:2, 0:2]
        rot_traj = pts[:, :2] @ rot.T  # N*2 = N*2 @ 2*2
        rot_headings = pts[:, 2] + angle
        rot_traj = np.hstack((rot_traj, rot_headings[:, np.newaxis]))  # N*2 + N*1
        return rot_traj
    carpose_path = np.array([external_car_pose["x"], external_car_pose["y"], external_car_pose["psi"]]).T
    carpose_path_rot = rotate_trajectory(carpose_path, -carpose_psi0)
    
    external_car_pose["x"] = (carpose_path_rot[:, 0] - carpose_path_rot[0, 0]).tolist()
    external_car_pose["y"] = (carpose_path_rot[:, 1] - carpose_path_rot[0, 1]).tolist()
    external_car_pose["psi"] = Functions.continuous_angle(carpose_path_rot[:, 2]).tolist()
    # Set up configuration files for the localization module
    def get_config_paths():
        """Get the absolute paths to the configuration files."""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        repo_root = os.path.abspath(os.path.join(script_dir, "../../../"))
        vehicle_config_path = os.path.join(repo_root, "vehicle_config.json")
        localization_config_path = os.path.join(repo_root, "localization_config.json")
        return vehicle_config_path, localization_config_path

    # Get the paths to the configuration files
    vehicle_config_path, localization_config_path = get_config_paths()
    with open(localization_config_path, 'r') as f:
        localization_config = json.load(f)
    # Check if configuration files exist
    if not os.path.exists(vehicle_config_path) or not os.path.exists(localization_config_path):
        print(f"ERROR: Configuration files not found")
        print(f"Looking for: {vehicle_config_path} and {localization_config_path}")
        sys.exit(1)
    
    print(f"Initializing AHRSLocHandler with configuration files:")
    print(f"  - {vehicle_config_path}")
    print(f"  - {localization_config_path}")
    
    # Initialize the AHRSLocHandler with the config files
    try:
        loc_handler = localization_pybind_module.AHRSLocHandler(vehicle_config_path, localization_config_path)
    except Exception as e:
        print(f"Error initializing AHRSLocHandler: {e}")
        sys.exit(1)
    
    # Initialize vehicle state (position at origin, zero heading)
    initial_time = -1.0  # Start at time 0
    initial_state = [0.0, 0.0, external_car_pose["psi"][0], 0.0]  # [x, y, heading, velocity]
    loc_handler.UpdateVehicleState(initial_time, initial_state)
    # localization is initialized to zero. 
    # clock is initialize to -1 so dt will be calculated only from 2nd sensor input.
    # Prepare for tracking estimated car pose
    car_pose = {
        "timestamp": [],
        "x": [],
        "y": [],
        "yaw": []
    }
    
    print(f"Initial vehicle state reset to origin (0,0,{external_car_pose['psi'][0]:.4f})")
    print("Starting to process sensor readings...")
    
    # Iterate through sorted sensor data and process each reading
    print("\nProcessing sensor readings chronologically...")
    for i, reading in tqdm(enumerate(trip_obj.sorted_localization_inputs), total=len(trip_obj.sorted_localization_inputs), desc="Processing sensors"):
        timestamp = reading['timestamp']
        sensor_id = reading['sensor_id']
        data = reading['data']
        
        # Call the appropriate method based on sensor type
        if timestamp >= carpose_t0: 
            # attempt to make this offline calculation like ai driver, 
            # but still ai driver is slightly different because of the unsynchronous way it works
            # and also because of the timestamps it is saved which are not sensors timestamps.
            if sensor_id == "IMU":
                # Create IMU sample and update the localization
                imu_sample = localization_pybind_module.ImuSample()
                imu_sample.time_stamp = timestamp
                # Get acceleration data from IMU row
                # Create Vec3d objects and set their properties separately
                acc_vec = localization_pybind_module.Vec3d()
                # Use exact field names from the header
                acc_vec.x = float(data["x_acc"])
                acc_vec.y = float(data["y_acc"])
                acc_vec.z = float(data["z_acc"])
                imu_sample.acc_ = acc_vec
                
                # Create acceleration body frame vector with bias values
                acc_b_vec = localization_pybind_module.Vec3d()
                # Use bias values if available, otherwise use the same values
                acc_b_vec.x = float(data["x_acc_bias"]) 
                acc_b_vec.y = float(data["y_acc_bias"]) 
                acc_b_vec.z = float(data["z_acc_bias"]) 
                imu_sample.acc_b_ = acc_b_vec
                
                # Create gyro vector
                gyro_vec = localization_pybind_module.Vec3d()
                # Use correct field names
                gyro_vec.x = float(data["x_gyro"]) * np.pi/180  # Convert to radians
                gyro_vec.y = float(data["y_gyro"]) * np.pi/180  # Convert to radians
                gyro_vec.z = float(data["z_gyro"]) * np.pi/180  # Convert to radians
                imu_sample.gyro_ = gyro_vec
                
                # Create gyro body frame vector with bias values
                gyro_b_vec = localization_pybind_module.Vec3d()
                # Use bias values if available, otherwise use the same values
                gyro_b_vec.x = float(data["x_gyro_bias"])  * np.pi/180  # Convert to radians
                gyro_b_vec.y = float(data["y_gyro_bias"])  * np.pi/180  # Convert to radians
                gyro_b_vec.z = float(data["z_gyro_bias"])  * np.pi/180  # Convert to radians
                imu_sample.gyro_b_ = gyro_b_vec
                
                # Create magnetometer vector
                mag_vec = localization_pybind_module.Vec3d()
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

            elif sensor_id == "speed" and localization_config["vehicle_speed_estimation_mode"] == "default":
                # Convert speed from km/h to m/s
                speed_mps = data["data_value"] / 3.6
                loc_handler.UpdateSpeed(speed_mps, timestamp)
                
            elif sensor_id == "steering":
                # Steering angle in radians
                steering_rad = data["data_value"] * np.pi / 180  # Convert from degrees to radians
                loc_handler.UpdateSteeringWheel(steering_rad, timestamp)
                
            elif sensor_id == "left_rear_wheel_speed" and localization_config["vehicle_speed_estimation_mode"] == "rear_average":
                # Update left rear wheel speed
                wheel_speed_mps = data["data_value"] / 3.6  # Convert from km/h to m/s
                loc_handler.UpdateRearLeftSpeed(wheel_speed_mps, timestamp)
                
            elif sensor_id == "right_rear_wheel_speed" and localization_config["vehicle_speed_estimation_mode"] == "rear_average":
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
    
    # Format final position for both human readability and machine parsing
    final_x = car_pose['x'][-1]
    final_y = car_pose['y'][-1]
    final_yaw = car_pose['yaw'][-1]
    final_yaw_deg = final_yaw * 180 / np.pi
    
    print(f"Final position estimate: x={final_x:.6f}, y={final_y:.6f}, yaw={final_yaw:.6f} rad ({final_yaw_deg:.2f} deg)")
    
    # Output in a format that can be easily parsed by scripts
    print("TESTOUTPUT_START")
    print(f"final_pose_x={final_x:.6f}")
    print(f"final_pose_y={final_y:.6f}")
    print(f"final_pose_yaw={final_yaw:.6f}")
    print("TESTOUTPUT_END")
    
    # Visualize the results if requested
    if args.visualize:
        # Create output directory if it doesn't exist
        os.makedirs(args.output_dir, exist_ok=True)
        
        # Create a figure with two columns of subplots
        fig = plt.figure(figsize=(18, 12))
        
        # Define the grid layout: 2 columns, 3 rows for right column
        gs = fig.add_gridspec(3, 2, width_ratios=[1, 1])
        
        # Left column - Trajectory plot (occupying all vertical space)
        ax_traj = fig.add_subplot(gs[:, 0])
        
        # Plot the trajectory
        if False:  # plot GPS
            ax_traj.plot(trip_obj.N, trip_obj.E, 'g-', linewidth=2, label='GPS Track')
        ax_traj.plot(car_pose["x"], car_pose["y"], 'b-', linewidth=2, label='Our Localization')
        
        # Also plot external car pose if available
        if external_car_pose is not None:
            carpose_x = np.array(external_car_pose['x'])
            carpose_y = np.array(external_car_pose['y'])
            ax_traj.plot(carpose_x, carpose_y, 'r-', linewidth=2, label='External Localization')
        
        ax_traj.scatter(car_pose["x"][0], car_pose["y"][0], c='g', s=100, label='Start')
        ax_traj.scatter(car_pose["x"][-1], car_pose["y"][-1], c='r', s=100, label='End')
        ax_traj.grid(True)
        ax_traj.set_aspect('equal')
        ax_traj.set_title('Vehicle Trajectory', fontsize=14)
        ax_traj.set_xlabel('East (m)', fontsize=12)
        ax_traj.set_ylabel('North (m)', fontsize=12)
        ax_traj.legend(loc='best', fontsize=12)
        
        # Right column, top plot - Heading comparison
        ax_heading = fig.add_subplot(gs[0, 1])
        ax_heading.plot(car_pose['timestamp'], np.array(car_pose['yaw']) * 180 / np.pi, 'b-', 
                        linewidth=2, label='Our Heading')
        
        # Add external car pose heading if available
        if external_car_pose is not None:
            # Convert to degrees for plotting
            ext_heading = np.array(external_car_pose['psi']) * 180 / np.pi
            ax_heading.plot(external_car_pose['timestamp'], ext_heading, 'r-', 
                           linewidth=2, label='External Heading')
        
        ax_heading.set_title('Vehicle Heading Comparison', fontsize=14)
        ax_heading.set_xlabel('Time (s)', fontsize=12)
        ax_heading.set_ylabel('Heading (deg)', fontsize=12)
        ax_heading.grid(True)
        ax_heading.legend(loc='best', fontsize=12)
        
        # Right column, middle plot - X position comparison
        ax_x = fig.add_subplot(gs[1, 1], sharex=ax_heading)
        ax_x.plot(car_pose['timestamp'], car_pose['x'], 'b-', 
                  linewidth=2, label='Our X')
        
        if external_car_pose is not None:
            ax_x.plot(external_car_pose['timestamp'], external_car_pose['x'], 'r-', 
                     linewidth=2, label='External X')
        
        ax_x.set_title('X Position Comparison', fontsize=14)
        ax_x.set_xlabel('Time (s)', fontsize=12)
        ax_x.set_ylabel('X Position (m)', fontsize=12)
        ax_x.grid(True)
        ax_x.legend(loc='best', fontsize=12)
        
        # Right column, bottom plot - Y position comparison
        ax_y = fig.add_subplot(gs[2, 1], sharex=ax_heading)
        ax_y.plot(car_pose['timestamp'], car_pose['y'], 'b-', 
                  linewidth=2, label='Our Y')
        
        if external_car_pose is not None:
            ax_y.plot(external_car_pose['timestamp'], external_car_pose['y'], 'r-', 
                     linewidth=2, label='External Y')
        
        ax_y.set_title('Y Position Comparison', fontsize=14)
        ax_y.set_xlabel('Time (s)', fontsize=12)
        ax_y.set_ylabel('Y Position (m)', fontsize=12)
        ax_y.grid(True)
        ax_y.legend(loc='best', fontsize=12)
        
        # Adjust layout for better spacing
        plt.tight_layout()
        
        # Save the combined plots
        combined_plot_path = os.path.join(args.output_dir, 'combined_comparison.png')
        fig.savefig(combined_plot_path, dpi=150, bbox_inches='tight')
        print(f"Combined comparison plots saved to {combined_plot_path}")
        
        # Show all plots
        plt.show()
    
    print("Script completed. Trip data loaded and processed.")
    
    return trip_obj, loc_handler, car_pose
    
if __name__ == '__main__':
    trip_obj, loc_handler, car_pose = main()
