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
    parser = argparse.ArgumentParser(description='Visualize SD test inputs/state and gyro bias estimation')
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

    # Config
    vehicle_config_path, localization_config_path = get_config_paths()
    if not os.path.exists(vehicle_config_path) or not os.path.exists(localization_config_path):
        print(f"ERROR: Missing configuration files: {vehicle_config_path} or {localization_config_path}")
        sys.exit(1)
    with open(localization_config_path, 'r') as f:
        localization_config = json.load(f)

    # Thresholds (fail if missing)
    acc_std_th = float(localization_config["SD_test_acc_std_th"])
    gyro_std_th = float(localization_config["SD_test_gyro_std_th"])
    speed_mean_th = float(localization_config["SD_test_carspeed_mean_th"])
    speed_max_th = float(localization_config["SD_test_carspeed_max_th"])

    # Handler
    print("Initializing AHRSLocHandler")
    loc_handler = localization_pybind_module.AHRSLocHandler(vehicle_config_path, localization_config_path)
    loc_handler.UpdateVehicleState(-1.0, [0.0, 0.0, 0.0, 0.0])

    # Records
    times = []
    acc_x = []; acc_y = []; acc_z = []
    gyro_x = []; gyro_y = []; gyro_z = []
    gyro_x_bias = []; gyro_y_bias = []; gyro_z_bias = []
    speed_rr = []; speed_rl = []
    sd_state = []
    feat_acc_std = []; feat_gyro_std = []; feat_speed_mean = []; feat_speed_max = []

    # Bias records
    bias_x_est = []; bias_y_est = []; bias_z_est = []

    last_acc = np.array([0.0, 0.0, 0.0])
    last_gyro = np.array([0.0, 0.0, 0.0])
    last_rr = 0.0; last_rl = 0.0

    for reading in tqdm(trip_obj.sorted_localization_inputs, desc="Streaming sensors"):
        t = reading['timestamp']
        sensor_id = reading['sensor_id']
        data = reading['data']

        if sensor_id == "IMU":
            imu = localization_pybind_module.ImuSample()
            imu.time_stamp = float(t)
            acc_vec = localization_pybind_module.Vec3d(); gyro_vec = localization_pybind_module.Vec3d()
            acc_vec.x = float(data["x_acc"]) ; acc_vec.y = float(data["y_acc"]) ; acc_vec.z = float(data["z_acc"])
            gyro_vec.x = float(data["x_gyro"]) * np.pi/180
            gyro_vec.y = float(data["y_gyro"]) * np.pi/180
            gyro_vec.z = float(data["z_gyro"]) * np.pi/180
            imu.acc_ = acc_vec; imu.gyro_ = gyro_vec
            imu.roll_ = float(data.get("roll", 0.0)) * np.pi/180
            imu.pitch_ = float(data.get("pitch", 0.0)) * np.pi/180
            imu.yaw_ = float(data.get("yaw", 0.0)) * np.pi/180
            loc_handler.UpdateIMU(imu, float(t))
            last_acc = np.array([acc_vec.x, acc_vec.y, acc_vec.z])
            last_gyro = np.array([gyro_vec.x, gyro_vec.y, gyro_vec.z])
            gbx = float(data["x_gyro_bias"]) * np.pi/180
            gby = float(data["y_gyro_bias"]) * np.pi/180
            gbz = float(data["z_gyro_bias"]) * np.pi/180
            last_gyro_bias = np.array([gbx, gby, gbz])

        elif sensor_id == "left_rear_wheel_speed" and localization_config["vehicle_speed_estimation_mode"] == "rear_average":
            last_rl = float(data["data_value"]) / 3.6
            loc_handler.UpdateRearLeftSpeed(last_rl, float(t))
        elif sensor_id == "right_rear_wheel_speed" and localization_config["vehicle_speed_estimation_mode"] == "rear_average":
            last_rr = float(data["data_value"]) / 3.6
            loc_handler.UpdateRearRightSpeed(last_rr, float(t))
        elif sensor_id == "speed" and localization_config["vehicle_speed_estimation_mode"] == "default":
            spd = float(data["data_value"]) / 3.6
            loc_handler.UpdateSpeed(spd, float(t))
            last_rl = spd; last_rr = spd
        elif sensor_id == "steering":
            steer_rad = float(data["data_value"]) * np.pi/180
            loc_handler.UpdateSteeringWheel(steer_rad, float(t))

        # Read state and features
        s = int(loc_handler.GetStaticDynamicTestState())
        acc_std_v, gyro_std_v, speed_mean_v, speed_max_v = loc_handler.GetStaticDynamicTestSensorsFeatures()
        bx, by, bz = loc_handler.GetGyroBiases()

        times.append(float(t))
        acc_x.append(last_acc[0]); acc_y.append(last_acc[1]); acc_z.append(last_acc[2])
        gyro_x.append(last_gyro[0]); gyro_y.append(last_gyro[1]); gyro_z.append(last_gyro[2])
        gyro_x_bias.append(last_gyro_bias[0]); gyro_y_bias.append(last_gyro_bias[1]); gyro_z_bias.append(last_gyro_bias[2])
        speed_rr.append(last_rr); speed_rl.append(last_rl)
        sd_state.append(s)
        feat_acc_std.append(acc_std_v); feat_gyro_std.append(gyro_std_v)
        feat_speed_mean.append(speed_mean_v); feat_speed_max.append(speed_max_v)
        bias_x_est.append(bx); bias_y_est.append(by); bias_z_est.append(bz)

    # Plot: left col = 4 SD plots, right col = 3 bias plots (share x within each col)
    os.makedirs(args.output_dir, exist_ok=True)
    fig = plt.figure(figsize=(18, 10))
    gs = fig.add_gridspec(4, 2, width_ratios=[1, 1])

    ax0 = fig.add_subplot(gs[0, 0])
    ax1 = fig.add_subplot(gs[1, 0], sharex=ax0)
    ax2 = fig.add_subplot(gs[2, 0], sharex=ax0)
    ax3 = fig.add_subplot(gs[3, 0], sharex=ax0)

    # Left column plots
    ax0.plot(times, acc_x, label='acc_x'); ax0.plot(times, acc_y, label='acc_y'); ax0.plot(times, acc_z, label='acc_z')
    ax0.plot(times, feat_acc_std, 'k--', linewidth=1.5, label='acc_std')
    ax0.axhline(acc_std_th, linestyle='--', color='r', alpha=0.7, label='acc_std_th')
    ax0.set_ylabel('Acc (m/s^2)'); ax0.set_title('Accelerometer'); ax0.grid(True); ax0.legend(loc='best')

    ax1.plot(times, gyro_x, label='gyro_x'); ax1.plot(times, gyro_y, label='gyro_y'); ax1.plot(times, gyro_z, label='gyro_z')
    ax1.plot(times, feat_gyro_std, 'k--', linewidth=1.5, label='gyro_std')
    ax1.axhline(gyro_std_th, linestyle='--', color='r', alpha=0.7, label='gyro_std_th')
    ax1.set_ylabel('Gyro (rad/s)'); ax1.set_title('Gyroscope'); ax1.grid(True); ax1.legend(loc='best')

    ax2.plot(times, speed_rl, label='rear_left_speed'); ax2.plot(times, speed_rr, label='rear_right_speed')
    ax2.plot(times, feat_speed_mean, 'k--', linewidth=1.5, label='speed_mean')
    ax2.plot(times, feat_speed_max, 'k:', linewidth=1.5, label='speed_max')
    ax2.axhline(speed_mean_th, linestyle='--', color='r', alpha=0.7, label='speed_mean_th')
    ax2.axhline(speed_max_th, linestyle='--', color='m', alpha=0.7, label='speed_max_th')
    ax2.set_ylabel('Wheel speed (m/s)'); ax2.set_title('Rear Wheel Speeds / Features'); ax2.grid(True); ax2.legend(loc='best')

    ax3.step(times, sd_state, where='post', label='StaticDynamicState')
    ax3.set_ylabel('SD State'); ax3.set_xlabel('Time (s)')
    ax3.set_yticks([StaticDynamicState.NOT_INITIALIZED, StaticDynamicState.STATIC, StaticDynamicState.DYNAMIC])
    ax3.set_yticklabels(['NOT_INITIALIZED', 'STATIC', 'DYNAMIC'])
    ax3.set_title('Static/Dynamic Test State'); ax3.grid(True)

    # Right column plots (per-axis gyro vs bias)
    bx0 = fig.add_subplot(gs[0, 1], sharex=ax0)
    bx1 = fig.add_subplot(gs[1, 1], sharex=ax0)
    bx2 = fig.add_subplot(gs[2, 1], sharex=ax0)

    bx0.plot(times, gyro_x, label='gyro_x')
    bx0.plot(times, bias_x_est, 'k--', label='bias_x_est')
    bx0.plot(times, gyro_x_bias, 'r--', label='bias_x_IMU')
    bx0.set_title('Gyro X and Bias'); bx0.grid(True); bx0.legend(loc='best')

    bx1.plot(times, gyro_y, label='gyro_y')
    bx1.plot(times, bias_y_est, 'k--', label='bias_y_est')
    bx1.plot(times, gyro_y_bias, 'r--', label='bias_y_IMU')
    bx1.set_title('Gyro Y and Bias'); bx1.grid(True); bx1.legend(loc='best')

    bx2.plot(times, gyro_z, label='gyro_z')
    bx2.plot(times, bias_z_est, 'k--', label='bias_z_est')
    bx2.plot(times, gyro_z_bias, 'r--', label='bias_z_IMU')
    bx2.set_title('Gyro Z and Bias'); bx2.grid(True); bx2.legend(loc='best')

    plt.tight_layout()
    out_path = os.path.join(args.output_dir, 'gyro_bias_estimation_visualization.png')
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f"Saved visualization to {out_path}")

    if args.show:
        plt.show()


if __name__ == '__main__':
    main()