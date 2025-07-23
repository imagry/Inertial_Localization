import os
import sys
import numpy as np
np.random.seed(0)
import matplotlib.pyplot as plt
import json
from tqdm import tqdm
from copy import copy
print("CWD = " + str(os.getcwd()) + "\n")
tests_python_path = "./Tests/python"
if tests_python_path not in sys.path:
    sys.path.append(tests_python_path)
from SteeringControlPlotter import SteeringControlPlotter
from pathlib import Path
import pandas as pd
import Functions
import time
import matplotlib
print(matplotlib.__version__)
import sys
print(sys.executable)
if __name__ == '__main__':
    current_dir_path = os.path.dirname(os.path.abspath(__file__))
    control_config_relative_path = "../../control_config.json"
    control_config_path = os.path.join(current_dir_path, control_config_relative_path)
    with open(control_config_path, "r") as f:
        control_config = json.loads(f.read())
    vehicle_config_relative_path = "../../vehicle_config.json"
    vehicle_config_path = os.path.join(current_dir_path, vehicle_config_relative_path)
    with open(vehicle_config_path, "r") as f:
        vehicle_config = json.loads(f.read())
    visualization_data_path = Path(control_config["visualization_data_path"])
    ref_traj_file_path = visualization_data_path / "reference_traj.csv"
    vehicle_state_file_path = visualization_data_path / "vehicle_state.csv"
    steering_file_path = visualization_data_path / "steering.csv"
    plotter_obj = SteeringControlPlotter(car_params=vehicle_config, flip_xy=True)
    i = 0
    convert_to_nav = False
    while True:
        try:
            if vehicle_state_file_path.is_file() and ref_traj_file_path.is_file() and steering_file_path.is_file():
                vehicle_state = pd.read_csv(vehicle_state_file_path, index_col=False)
                traj = pd.read_csv(ref_traj_file_path, index_col=False)
                steering_data = pd.read_csv(steering_file_path, index_col=False)
                ref_traj = np.vstack([np.array(traj['x']), np.array(traj['y'])]).T
                Phat = [vehicle_state['x'][0], vehicle_state['y'][0]]
                psi_hat = vehicle_state['psi'][0]
                if convert_to_nav:
                    Tnav_hat2ego = Functions.affine_transformation_matrix_2D(Phat[0], Phat[1], psi_hat)
                    Tego2navhat = Functions.inv_affine_transformation_matrix_2D(Tnav_hat2ego)
                    reference_traj_in_nav_hat = Functions.project_points_2D(Tego2navhat, ref_traj)
                else:
                    reference_traj_in_nav_hat = ref_traj
                plotter_obj.update_plot(current_car_position_x=Phat[0],
                                        current_car_position_y=Phat[1],
                                        current_car_heading=psi_hat,
                                        steering=steering_data["steering_measured"][0], # the recorded value is the steering wheel / steering ratio
                                        steering_cmd_raw=steering_data["steering_wheel_cmd_raw"][0] /
                                                         vehicle_config["steering_ratio"], # the recorded value is the delta command
                                        steering_cmd_filtered=steering_data["steering_wheel_cmd_filtered"][0] /
                                                         vehicle_config["steering_ratio"],
                                        motion_planning_traj_raw=reference_traj_in_nav_hat,
                                        motion_planning_traj_processed=reference_traj_in_nav_hat,
                                        lateral_error=steering_data["lateral_error"][0],
                                        heading_error=steering_data["heading_error"][0],
                                        heading_reference_raw=steering_data["heading_reference_raw"][0],
                                        heading_reference_filtered=steering_data["heading_reference_filtered"][0],
                                        t_i=time.time(),
                                        control_target_point=[steering_data['target_point_x'], steering_data['target_point_y']]
                                        )

        except:
            i += 1
        time.sleep(0.1)