import os
import sys
import argparse
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from enum import IntEnum
from tqdm import tqdm
import json

# Make project python utilities available
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import Classes  # noqa: E402

# Make the built pybind module importable
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'build'))
try:
    import localization_pybind_module  # noqa: E402
    print("Successfully imported localization_pybind_module")
except ImportError as e:
    print(f"Failed to import localization_pybind_module: {e}")
    print("Ensure you activated the venv and ran ./rebuild.sh in Tests/python/python_binding")
    sys.exit(1)


class StaticDynamicState(IntEnum):
    NOT_INITIALIZED = 0
    STATIC = 1
    DYNAMIC = 2


def get_config_paths():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(script_dir, "../../../"))
    vehicle_config_path = os.path.join(repo_root, "vehicle_config.json")
    localization_config_path = os.path.join(repo_root, "localization_config.json")
    return vehicle_config_path, localization_config_path


def main():
    parser = argparse.ArgumentParser(description='Visualize Static/Dynamic test inputs and state')
    parser.add_argument('--trip_path', type=str,
                        default='data/backed_data_files/2025-05-21T11_52_50',
                        help='Path to the trip data directory')
    parser.add_argument('--output_dir', type=str, default='./results',
                        help='Directory to save plots')
    parser.add_argument('--show', action='store_true', default=False,
                        help='Show the plot window in addition to saving')
    args = parser.parse_args()

    # Load trip
    trip_path = Path(args.trip_path)
    print(f'Loading trip data from {trip_path}')
    trip_obj = Classes.Trip(trip_path)
    trip_obj.sort_localization_inputs(print_results=True)

    # Config paths and config
    vehicle_config_path, localization_config_path = get_config_paths()
    if not os.path.exists(vehicle_config_path) or not os.path.exists(localization_config_path):
        print(f"ERROR: Missing configuration files: {vehicle_config_path} or {localization_config_path}")
        sys.exit(1)
    with open(localization_config_path, 'r') as f:
        localization_config = json.load(f)

    # Extract thresholds for plotting (fail fast if key missing)
    acc_std_th = float(localization_config["SD_test_acc_std_th"])
    gyro_std_th = float(localization_config["SD_test_gyro_std_th"])
    speed_mean_th = float(localization_config["SD_test_carspeed_mean_th"])
    speed_max_th = float(localization_config["SD_test_carspeed_max_th"])

    # Initialize handler
    print("Initializing AHRSLocHandler")
    loc_handler = localization_pybind_module.AHRSLocHandler(vehicle_config_path, localization_config_path)
    # Initialize vehicle state at origin. Use -1.0 clock like other scripts
    loc_handler.UpdateVehicleState(-1.0, [0.0, 0.0, 0.0, 0.0])

    # Records (step-wise, synchronized on sorted event times)
    times = []
    acc_x = []
    acc_y = []
    acc_z = []
    gyro_x = []
    gyro_y = []
    gyro_z = []
    speed_rr = []
    speed_rl = []
    sd_state = []

    # Feature records
    feat_acc_std = []
    feat_gyro_std = []
    feat_speed_mean = []
    feat_speed_max = []

    # Last known values for step traces
    last_acc = np.array([0.0, 0.0, 0.0])
    last_gyro = np.array([0.0, 0.0, 0.0])
    last_rr = 0.0
    last_rl = 0.0

    # Stream through sorted sensors
    for reading in tqdm(trip_obj.sorted_localization_inputs, desc="Streaming sensors"):
        t = reading['timestamp']
        sensor_id = reading['sensor_id']
        data = reading['data']

        if sensor_id == "IMU":
            imu = localization_pybind_module.ImuSample()
            imu.time_stamp = float(t)
            acc_vec = localization_pybind_module.Vec3d()
            acc_vec.x = float(data["x_acc"]) ; acc_vec.y = float(data["y_acc"]) ; acc_vec.z = float(data["z_acc"])
            gyro_vec = localization_pybind_module.Vec3d()
            # Gyro in rad/s
            gyro_vec.x = float(data["x_gyro"]) * np.pi/180
            gyro_vec.y = float(data["y_gyro"]) * np.pi/180
            gyro_vec.z = float(data["z_gyro"]) * np.pi/180
            imu.acc_ = acc_vec
            imu.gyro_ = gyro_vec
            # Minimal required fields for UpdateIMU path
            imu.roll_ = float(data.get("roll", 0.0)) * np.pi/180
            imu.pitch_ = float(data.get("pitch", 0.0)) * np.pi/180
            imu.yaw_ = float(data.get("yaw", 0.0)) * np.pi/180
            loc_handler.UpdateIMU(imu, float(t))
            last_acc = np.array([acc_vec.x, acc_vec.y, acc_vec.z])
            last_gyro = np.array([gyro_vec.x, gyro_vec.y, gyro_vec.z])

        elif sensor_id == "left_rear_wheel_speed" and localization_config["vehicle_speed_estimation_mode"] == "rear_average":
            last_rl = float(data["data_value"]) / 3.6
            loc_handler.UpdateRearLeftSpeed(last_rl, float(t))

        elif sensor_id == "right_rear_wheel_speed" and localization_config["vehicle_speed_estimation_mode"] == "rear_average":
            last_rr = float(data["data_value"]) / 3.6
            loc_handler.UpdateRearRightSpeed(last_rr, float(t))

        elif sensor_id == "speed" and localization_config["vehicle_speed_estimation_mode"] == "default":
            # Only a single channel available; mirror to both for plotting
            spd = float(data["data_value"]) / 3.6
            loc_handler.UpdateSpeed(spd, float(t))
            last_rl = spd
            last_rr = spd

        elif sensor_id == "steering":
            # feed steering to keep handler close to real path (optional for this viz)
            steer_rad = float(data["data_value"]) * np.pi/180
            loc_handler.UpdateSteeringWheel(steer_rad, float(t))

        # Sample the state and features and push step-wise rows
        s = int(loc_handler.GetStaticDynamicTestState())
        # features: (acc_std, gyro_std, speed_mean, speed_max)
        acc_std_v, gyro_std_v, speed_mean_v, speed_max_v = loc_handler.GetStaticDynamicTestSensorsFeatures()

        times.append(float(t))
        acc_x.append(last_acc[0]); acc_y.append(last_acc[1]); acc_z.append(last_acc[2])
        gyro_x.append(last_gyro[0]); gyro_y.append(last_gyro[1]); gyro_z.append(last_gyro[2])
        speed_rr.append(last_rr); speed_rl.append(last_rl)
        sd_state.append(s)

        feat_acc_std.append(acc_std_v)
        feat_gyro_std.append(gyro_std_v)
        feat_speed_mean.append(speed_mean_v)
        feat_speed_max.append(speed_max_v)

    # Plot
    os.makedirs(args.output_dir, exist_ok=True)
    fig, axs = plt.subplots(4, 1, figsize=(14, 10), sharex=True)

    axs[0].plot(times, acc_x, label='acc_x')
    axs[0].plot(times, acc_y, label='acc_y')
    axs[0].plot(times, acc_z, label='acc_z')
    axs[0].plot(times, feat_acc_std, 'k--', linewidth=1.5, label='acc_std')
    axs[0].axhline(acc_std_th, linestyle='--', color='r', alpha=0.7, label='acc_std_th')
    axs[0].set_ylabel('Acc (m/s^2)')
    axs[0].set_title('Accelerometer')
    axs[0].grid(True)
    axs[0].legend(loc='best')

    axs[1].plot(times, gyro_x, label='gyro_x')
    axs[1].plot(times, gyro_y, label='gyro_y')
    axs[1].plot(times, gyro_z, label='gyro_z')
    axs[1].plot(times, feat_gyro_std, 'k--', linewidth=1.5, label='gyro_std')
    axs[1].axhline(gyro_std_th, linestyle='--', color='r', alpha=0.7, label='gyro_std_th')
    axs[1].set_ylabel('Gyro (rad/s)')
    axs[1].set_title('Gyroscope')
    axs[1].grid(True)
    axs[1].legend(loc='best')

    axs[2].plot(times, speed_rl, label='rear_left_speed')
    axs[2].plot(times, speed_rr, label='rear_right_speed')
    axs[2].plot(times, feat_speed_mean, 'k--', linewidth=1.5, label='speed_mean')
    axs[2].plot(times, feat_speed_max, 'k:', linewidth=1.5, label='speed_max')
    axs[2].axhline(speed_mean_th, linestyle='--', color='r', alpha=0.7, label='speed_mean_th')
    axs[2].axhline(speed_max_th, linestyle='--', color='m', alpha=0.7, label='speed_max_th')
    axs[2].set_ylabel('Wheel speed (m/s)')
    axs[2].set_title('Rear Wheel Speeds / Features')
    axs[2].grid(True)
    axs[2].legend(loc='best')

    axs[3].step(times, sd_state, where='post', label='StaticDynamicState')
    axs[3].set_ylabel('SD State')
    axs[3].set_xlabel('Time (s)')
    axs[3].set_yticks([StaticDynamicState.NOT_INITIALIZED, StaticDynamicState.STATIC, StaticDynamicState.DYNAMIC])
    axs[3].set_yticklabels(['NOT_INITIALIZED', 'STATIC', 'DYNAMIC'])
    axs[3].set_title('Static/Dynamic Test State')
    axs[3].grid(True)

    plt.tight_layout()
    out_path = os.path.join(args.output_dir, 'static_dynamic_test_visualization.png')
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f"Saved visualization to {out_path}")

    if args.show:
        plt.show()


if __name__ == '__main__':
    main()