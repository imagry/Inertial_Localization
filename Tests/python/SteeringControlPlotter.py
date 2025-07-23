import os
import matplotlib
# matplotlib.use("Qt5Agg") 
matplotlib.use('TkAgg')
from matplotlib.widgets import Slider, Button, CheckButtons
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import sys
import atexit
import signal
import numpy as np
from scipy.interpolate import interp1d
from pathlib import Path
from PIL import Image
import pandas as pd
import csv
import json
from tqdm import tqdm
tests_python_path = "./Tests/python"
if tests_python_path not in sys.path:
    sys.path.append(tests_python_path)
from datetime import datetime
import Functions
import Classes
from tqdm import tqdm
def on_click(event):
    print(f'You clicked at {event.xdata}, {event.ydata}')
def mypause(interval):
    backend = plt.rcParams['backend']
    if backend in matplotlib.rcsetup.interactive_bk:
        figManager = matplotlib._pylab_helpers.Gcf.get_active()
        if figManager is not None:
            canvas = figManager.canvas
            if canvas.figure.stale:
                canvas.draw()
            canvas.start_event_loop(interval)
            return
class SteeringControlPlotter:
    def __init__(self, car_params, desired_traj_x=None, desired_traj_y=None, flip_xy=True, initialize_plot=True):
        self.BEV_range_x = 50.0
        self.BEV_range_y = 50.0
        self.minimal_margin_x = 5.0
        self.minimal_margin_y = 5.0
        self.traveled_path_x = []
        self.traveled_path_y = []
        self.car_params = car_params
        self.time_line = []
        self.lateral_error = []
        self.heading_reference_raw = []
        self.heading_reference_filtered = []
        self.vehicle_heading = []
        self.heading_error = []
        self.heading_error = []
        self.steering = []
        self.steering_cmd_raw = []
        self.steering_cmd_filtered = []
        self.time_line_visible_frame = 60
        self.flip_xy = flip_xy
        self.aggregated_control_path = None
        if flip_xy:
            self.desired_traj_x = desired_traj_y
            self.desired_traj_y = desired_traj_x
            self.steering_sf_in_BEV = -1.0
        else:
            self.desired_traj_x = desired_traj_x
            self.desired_traj_y = desired_traj_y
            self.steering_sf_in_BEV = 1.0
        if initialize_plot:
            self.initialize_plot()
            self.plot_initialized = True
        else:
            self.plot_initialized = False
    def initialize_plot(self, mode='animation'):
        if mode == 'animation':
            plt.ion()
        self.animation_figure = plt.figure('animation', figsize=(20, 10))
        atexit.register(self.close_figure)
        signal.signal(signal.SIGTERM, self.handle_signal)
        signal.signal(signal.SIGINT, self.handle_signal)
        self.vehicle_animation_axis = plt.subplot(1, 2, 1)
        self.vehicle_animation_axis.grid(True)
        self.vehicle_animation_axis.set_xlim(- self.BEV_range_x, + self.BEV_range_x)
        self.vehicle_animation_axis.set_ylim(- self.BEV_range_y, + self.BEV_range_y)
        self.lateral_error_axis = plt.subplot(3, 2, 2)
        self.heading_control_axis = plt.subplot(3, 2, 4, sharex=self.lateral_error_axis)
        self.steering_axis = plt.subplot(3, 2, 6, sharex=self.lateral_error_axis)
        if mode != 'animation':
            # Adjust the main plot to make room for the slider
            self.animation_figure.subplots_adjust(bottom=0.25)
            # Create a horizontal slider to control the position
            self.slider_axis = self.animation_figure.add_axes([0.25, 0.1, 0.65, 0.03])
            self.slider = Slider(
                ax=self.slider_axis,
                label='Time [sec]',
                valmin=self.common_time_line[0],
                valmax=self.common_time_line[-1],
                valinit=self.common_time_line[0],
            )
            # Register the update function with the slider
            self.slider.on_changed(self.slider_update)
        self.plot_initialized = True
    def update_plot(self, current_car_position_x, current_car_position_y, current_car_heading, steering,
                    motion_planning_traj_raw , motion_planning_traj_processed, t_i, lateral_error,
                    steering_cmd_raw=0.0,steering_cmd_filtered=0.0, heading_error=0.0, heading_reference_raw=0.0,
                    heading_reference_filtered=0.0, control_target_point = None):
        '''
        position in meters, angles in radians, steering is in the wheels relative to car and not steering wheel.
        '''
        if self.flip_xy:
            # x,y = y,x swaps variables
            current_car_position_x, current_car_position_y = current_car_position_y, current_car_position_x
            # np.roll is used for swapping the axes
            motion_planning_traj_raw = np.roll(motion_planning_traj_raw, 1, axis=1)
            motion_planning_traj_processed = np.roll(motion_planning_traj_processed, 1, axis=1)
            current_car_heading_for_BEV = np.pi/2 - current_car_heading
            if control_target_point is not None:
                control_target_point[0], control_target_point[1] = control_target_point[1], control_target_point[0]
        self.vehicle_heading.append(current_car_heading)
        self.heading_reference_raw.append(heading_reference_raw)
        self.heading_reference_filtered.append(heading_reference_filtered)
        self.traveled_path_x.append(current_car_position_x)
        self.traveled_path_y.append(current_car_position_y)
        self.time_line.append(t_i)
        self.lateral_error.append(lateral_error)
        self.heading_error.append(heading_error)
        self.steering.append(steering)
        self.steering_cmd_raw.append(steering_cmd_raw)
        self.steering_cmd_filtered.append(steering_cmd_filtered)
        if self.aggregated_control_path is None and control_target_point is not None:
            self.aggregated_control_path = [control_target_point]
        elif control_target_point is not None:
            self.aggregated_control_path.append(control_target_point)
        self.vehicle_animation_axis.clear()
        if self.desired_traj_x is not None and self.desired_traj_y is not None:
            self.vehicle_animation_axis.plot(self.desired_traj_x, self.desired_traj_y,
                                             color='gray', linewidth=0.5, linestyle='--',
                                             label='GT reference path')
        self.vehicle_animation_axis.set_title('t = ' + str("%.2f" % t_i))
        self.vehicle_traj_line = self.vehicle_animation_axis.plot(self.traveled_path_x, self.traveled_path_y,
                                                                  linewidth=2.0, color='darkviolet')
        self.vehicle_animation_axis.scatter(motion_planning_traj_raw[:, 0], motion_planning_traj_raw[:, 1], s=10, color='green',
                                            label='observation')
        self.vehicle_animation_axis.scatter(motion_planning_traj_processed[:, 0], motion_planning_traj_processed[:, 1], s=10, color='blue',
                                            label='ref path in nav frame')
        if self.aggregated_control_path is not None:
            aggregated_path = np.array(self.aggregated_control_path)
            self.vehicle_animation_axis.scatter(aggregated_path[:,0],aggregated_path[:,1], s=10, marker='x', color='black',label='aggregated control path')
            self.vehicle_animation_axis.scatter(control_target_point[0], control_target_point[1],s=50, color='red', marker='x',
                                                label='target point')
        self.vehicle_animation_axis.legend()
        self.vehicle_line = Functions.draw_car(current_car_position_x, current_car_position_y, current_car_heading_for_BEV,
                                               steer=steering * self.steering_sf_in_BEV,
                                               car_params=self.car_params,
                                               ax=self.vehicle_animation_axis)
        self.vehicle_animation_axis.grid(True)
        xmin = max(min(- 0.5 * self.BEV_range_x, current_car_position_x - self.minimal_margin_x),
                   current_car_position_x + self.minimal_margin_x - self.BEV_range_x)
        xmax = min(max(0.5 * self.BEV_range_x, current_car_position_x + self.minimal_margin_x),
                   current_car_position_x - self.minimal_margin_x + self.BEV_range_x)
        ymin = max(min(- 0.5 * self.BEV_range_y, current_car_position_y - self.minimal_margin_y),
                   current_car_position_y + self.minimal_margin_y - self.BEV_range_y)
        ymax = min(max(0.5 * self.BEV_range_y, current_car_position_y + self.minimal_margin_y),
                   current_car_position_y - self.minimal_margin_y + self.BEV_range_y)
        self.vehicle_animation_axis.set_xlim(xmin, xmax)
        self.vehicle_animation_axis.set_ylim(ymin, ymax)
        if self.flip_xy:
            self.vehicle_animation_axis.set_xlabel('y [m]'), self.vehicle_animation_axis.set_ylabel('x [m]')
        else:
            self.vehicle_animation_axis.set_xlabel('x [m]'), self.vehicle_animation_axis.set_ylabel('y [m]')
        self.lateral_error_axis.clear()
        self.lateral_error_axis.plot(self.time_line, self.lateral_error)
        self.lateral_error_axis.grid(True)
        self.lateral_error_axis.set_ylabel('$lateral error [m]$')
        self.lateral_error_axis.set_xlim(max(0, t_i - self.time_line_visible_frame), t_i)

        self.heading_control_axis.clear()
        self.heading_control_axis.plot(self.time_line, self.vehicle_heading, label='vehicle_heading')
        self.heading_control_axis.plot(self.time_line, self.heading_reference_raw, label='heading_reference_raw')
        self.heading_control_axis.plot(self.time_line, self.heading_reference_filtered, label='heading_reference_filtered')
        self.heading_control_axis.legend()
        self.heading_control_axis.grid(True), self.heading_control_axis.set_ylabel('[rad]')
        self.heading_control_axis.set_xlim(max(0, t_i - self.time_line_visible_frame), t_i)

        self.steering_axis.clear()
        self.steering_axis.grid(True), self.steering_axis.set_xlabel('t [sec]'), self.steering_axis.set_ylabel('steering angle [rad]')
        self.steering_axis.plot(self.time_line, self.steering, label="steering")
        self.steering_axis.plot(self.time_line, self.steering_cmd_raw, label="steering_cmd_raw", linestyle='--', linewidth=3)
        self.steering_axis.plot(self.time_line, self.steering_cmd_filtered, label="steering_cmd_filtered", linestyle='--',
                                linewidth=3)
        self.steering_axis.plot(self.time_line, self.heading_error, label="heading_error")
        self.steering_axis.set_xlim(max(t_i - self.time_line_visible_frame, 0), t_i)
        self.steering_axis.legend()
        mypause(0.01)
    def close_figure(self):
        if self.plot_initialized:
            plt.close(self.animation_figure)
            print("SteeringControlPlotter.close_figure: figure terminated")

    def __del__(self):
        if self.plot_initialized:
            plt.close(self.animation_figure)
            print("SteeringControlPlotter.__del__: figure terminated")

    def handle_signal(self, signum, frame):
        if self.plot_initialized:
            print("SteeringControlPlotter.handle_signal reached")
            self.close_figure()
            sys.exit(0)
class MotionPlanningDebug:
    def __init__(self, path_files_container, parse_motion_planning_files=True, parse_planner_path=True):
        path_files_container = Path(path_files_container)
        if parse_motion_planning_files:
            print("reading motion planning files")
            self.path_trajectory = self.parse_path_file(path_files_container / "path_trajectory.csv")
            self.path_adjustment = self.parse_path_file(path_files_container / "path_adjustment.csv")
            self.path_extraction = self.parse_path_file(path_files_container / "path_extraction.csv")
        if parse_planner_path:
            print("reading planner path file: {}".format(path_files_container / "planner_path.csv"))
            self.planner_path = self.parse_path_file(path_files_container / "planner_path.csv")
        self.planner_path_in_world_frame = None
        self.car_pose = None# self.parse_car_pose_file(path_files_container)
        self.space_net = None
        self.road_layer = None
        self.vehicles = None
        self.MPC_solution = None
        self.continuous_lane_dividers = None
        self.dashed_lane_dividers = None
        self.kmtx = 5.6 # meters to pixels in space net
        self.road_layer_image_height = 420
        self.road_layer_image_width = 336
        self.road_layer_x_range_meters = self.pixels_to_meters_space_net(self.road_layer_image_width)
        self.road_layer_y_range_meters = self.pixels_to_meters_space_net(self.road_layer_image_height)
        self.space_net_image_height = 420
        self.space_net_image_width = 336
        self.space_net_x_range_meters = self.pixels_to_meters_space_net( self.space_net_image_width)
        self.space_net_y_range_meters = self.pixels_to_meters_space_net(self.space_net_image_height)
        self.gui_initialized = False
        self.control_localization = None
        script_dir = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(script_dir,'../../vehicle_config.json'), "r") as f:
            self.vehicle_params = json.loads(f.read())
        self.data_timestamp = None
    def parse_path_file(self, file_path):
        def read_line(line):
            line_split = line.split(',')
            ti = float(line_split[0])
            path_xy = [float(str_num) for str_num in line_split[idx_of_first_path_x:]]
            path_x = path_xy[::2] # even indices start from 0
            path_y = path_xy[1::2] # odd indices
            if len(path_x) > len(path_y):
                path_x.pop()
            return ti, path_x, path_y
        with open(file_path, "r") as f:
            lines = f.readlines()
        data = {}
        n_lines = len(lines)
        header = lines[0]
        header = header.split(',')
        idx_of_first_path_x = header.index(' path_x_0')
        line_idx = 1
        while line_idx < n_lines:
            t_i, px, py = read_line(lines[line_idx])
            path_frame = {"x":px, "y":py}
            data[t_i] = path_frame
            line_idx += 1
        return data
    def parse_car_pose_file(self, file_container):
        
        def find_car_pose_file():
            for file in os.listdir(file_container):
                if "car_pose" in file:
                    res = file
            return res
        car_pose_path = os.path.join(file_container, find_car_pose_file())
        print(f"parsing the carpose file: {car_pose_path}")
        with open(car_pose_path, "r") as f:
            lines = f.readlines()
        self.car_pose = {}
        # x_vec = []
        # y_vec = []
        # t_vec = []
        # psi_vec = []
        for line in tqdm(lines[1:], desc="Parsing car pose file"):
            splited = line.split(',')
            timestamp = float(splited[0])
            x = float(splited[1])
            y = - float(splited[2])
            psi = - float(splited[3]) * np.pi / 180
            # x_vec.append(x)
            # y_vec.append(y)
            # psi_vec.append(psi)
            # t_vec.append(timestamp)
            self.car_pose.__setitem__(timestamp, {"x": x, "y":y, "psi":psi})
        # plt.figure()
        # plt.plot(y_vec,x_vec)
        # plt.figure()
        # plt.plot((np.array(t_vec) - t_vec[0]) , psi_vec)
        # plt.show()
        # return data
    def parse_control_localization(self, control_localization):
        """
        The data type is pandas.dataframe.
        Structure is time [sec], vel [mps], steering [rad], x [m], y [m], psi [rad]
        Vectorized implementation for better performance
        """
        print("Parsing control localization data...")
        
        # Convert all data to numpy arrays first (if not already)
        times = np.array(control_localization["time [sec]"])
        x_values = np.array(control_localization["x [m]"])
        y_values = np.array(control_localization["y [m]"])
        psi_values = np.array(control_localization["psi [rad]"])
        
        # Create dictionary in one go
        self.control_localization = {
            times[i]: {
                "x": x_values[i],
                "y": y_values[i],
                "psi": psi_values[i]
            } for i in range(len(times))
        }        
    # def parse_control_localization(self, control_localization):
    #     """
    #     the data type is pandas.dataframe.
    #     structure is time [sec], vel [mps], steering [rad], x [m], y [m], psi [rad]
    #     """
    #     self.control_localization = {}
    #     for i in tqdm(range(len(list(control_localization["time [sec]"]))), desc="Parsing control localization data"):
    #         self.control_localization.__setitem__(np.array(control_localization["time [sec]"])[i],
    #                          {"x":np.array(control_localization["x [m]"])[i],
    #                           "y":np.array(control_localization["y [m]"])[i],
    #                           "psi":np.array(control_localization["psi [rad]"])[i]})
    def compare_car_pose_to_control_localization(self):
        plt.figure()
        t_carpose = np.array(list(self.car_pose.keys()))
        x_carpose = np.array([self.car_pose[key]['x'] for key in self.car_pose])
        x_carpose -= x_carpose[0]
        y_carpose = np.array([self.car_pose[key]['y'] for key in self.car_pose])
        y_carpose -= y_carpose[0]
        psi_carpose = np.array([self.car_pose[key]['psi'] for key in self.car_pose])
        t_control_localization = np.array(list(self.control_localization.keys()))
        x_control_localization = [self.control_localization[key]['x'] for key in self.control_localization]
        y_control_localization = [self.control_localization[key]['y'] for key in self.control_localization]
        psi_control_localization = np.array([self.control_localization[key]['psi'] for key in self.control_localization])
        plt.plot(- y_carpose, x_carpose)
        plt.plot(y_control_localization, x_control_localization)
        plt.xlabel('y [m]'), plt.ylabel('x [m]'), plt.grid(True)
        plt.legend(['car pose','control localization'])

        plt.figure()
        plt.plot(t_carpose, - psi_carpose * np.pi / 180 )
        plt.plot(t_control_localization, psi_control_localization)
        plt.xlabel('time [sec]'), plt.ylabel('[rad]'),plt.grid(True)
        plt.legend(['car pose heading', 'control localization heading'])
        plt.show()
    def get_car_pose_item(self, ti, side):
        car_pose_times = np.array(list(self.car_pose.keys()))
        idx = Functions.search_time_vector(car_pose_times, ti, side)
        key = list(self.car_pose.keys())[idx]
        return self.car_pose[key]
    def get_control_localization_item(self, ti, side):
        car_pose_times = np.array(list(self.control_localization.keys()))
        idx = Functions.search_time_vector(car_pose_times, ti, side)
        key = list(self.control_localization.keys())[idx]
        return self.control_localization[key]
    def get_path_adjustment_item(self, ti, side):
        path_adjustment_times = np.array(list(self.path_adjustment.keys()))
        idx = Functions.search_time_vector(path_adjustment_times, ti, side)
        key = list(self.path_adjustment.keys())[idx]
        return self.path_adjustment[key]
    def get_path_extraction_item(self, ti, side):
        path_extraction_times = np.array(list(self.path_extraction.keys()))
        idx = Functions.search_time_vector(path_extraction_times, ti, side)
        key = list(self.path_extraction.keys())[idx]
        print("path_extraction time: " + str(key))
        return self.path_extraction[key]
    def get_path_trajectory_item(self, ti, side):
        path_trajectory_times = np.array(list(self.path_trajectory.keys()))
        idx = Functions.search_time_vector(path_trajectory_times, ti, side)
        # print("closest image time found = " + str(path_trajectory_times[idx]))
        key = list(self.path_trajectory.keys())[idx]
        print("path_trajectory time :" + str(key))
        return self.path_trajectory[key]
    def get_planner_path_in_world_frame_item(self, ti, side):
        planner_path_in_world_frame_times = np.array(list(self.planner_path_in_world_frame.keys()))
        idx = Functions.search_time_vector(planner_path_in_world_frame_times, ti, side)
        # print("closest image time found = " + str(path_trajectory_times[idx]))
        key = list(self.planner_path_in_world_frame.keys())[idx]
        return self.planner_path_in_world_frame[key]
    def calculate_planner_path_in_world_frame(self):
        assert self.planner_path is not None, "path_trajectory is None, please parse the path files first"
        assert self.car_pose is not None, "car_pose is None, please parse the car pose files first"
        '''
        1. iterate over self.path_trajectory and convert the path points from the car frame to the world frame
        2. at each path_trajectory item, use the key as the time stamp to find the closest car pose that was earlier.
           to do that use get_car_pose_item().
        3. conversion of path to world frame is done by Functions.project_points_2D() and saved in 
            self.path_trajectory_world_frame in the same data structure as self.path_trajectory (key->time, x, y, psi)

        '''
        # Initialize the world frame path trajectory dictionary
        self.planner_path_in_world_frame = {}
        
        # Iterate through each entry in the path_trajectory dictionary
        for timestamp, path_data in tqdm(self.planner_path.items(), desc="Calculating planner path in world frame"):
            # Get the car pose at this timestamp (or the closest earlier one)
            car_pose = self.get_car_pose_item(timestamp, "left")
            
            # Extract car position and orientation
            car_x = car_pose["x"]
            car_y = car_pose["y"]
            car_psi = car_pose["psi"]
            
            # Convert path points from car frame to world frame
            # First, create a numpy array of points from the path data
            # minus sign is because in motion planning frame left is postitive and in carpose right is positive
            path_points = np.vstack((np.array(path_data["x"]), - np.array(path_data["y"]))).T
            
            T_nav2ego = Functions.affine_transformation_matrix_2D(car_x, car_y, car_psi)
            T_ego2nav = Functions.inv_affine_transformation_matrix_2D(T_nav2ego)
            world_points = Functions.project_points_2D(T_ego2nav, path_points)
            
            # Save the result in the same format as path_trajectory
            self.planner_path_in_world_frame[timestamp] = {
                "x": world_points[:, 0].tolist(),
                "y": world_points[:, 1].tolist(),
                "carpose": car_pose, 
                "carpose timestamp": timestamp
            }
    def calculate_erros_relative_to_planner_path(self):
        assert self.planner_path_in_world_frame is not None, "planner_path_in_world_frame is None, please parse the path files first"
        assert self.car_pose is not None, "car_pose is None, please parse the car pose files first"
        
        # Initialize dictionaries with lists to store timestamps and error values
        self.heading_error_relative_to_planner_path = {
            'timestamp': [],
            'values': []
        }
        self.lateral_error_relative_to_planner_path = {
            'timestamp': [],
            'values': []
        }
        
        # Initialize new members for storing reference points and headings
        self.aggregated_reference_points = {
            'timestamp': [],
            'x': [],
            'y': []
        }
        self.aggregated_reference_headings = {
            'timestamp': [],
            'values': []
        }
        
        # Iterate through each car pose
        for timestamp in tqdm(self.car_pose.keys(), desc="Calculating errors relative to planner path"):
            # Get current car pose
            current_car_pose = self.car_pose[timestamp]
            
            # Get the relevant reference path at this timestamp
            reference_path = self.get_planner_path_in_world_frame_item(timestamp, "left")
            
            # Extract reference path coordinates
            ref_path_x = reference_path['x']
            ref_path_y = reference_path['y']
            
            # Calculate reference heading using Functions.calc_path_features
            _, _, ref_heading = Functions.calc_path_features(ref_path_x, ref_path_y)
            
            # Create a Stanley controller object to calculate errors
            stanley_controller = Classes.StanleyController(
                Ks=1.0,  # Ks value doesn't matter here as we're only using it to calculate errors
                desired_traj_x=ref_path_x,
                desired_traj_y=ref_path_y,
                desired_traj_psi=ref_heading
            )
            
            # Calculate lateral and heading errors using the list overload
            # vehicle_state format is [x, y, psi, v]
            vehicle_state = [current_car_pose['x'], current_car_pose['y'], current_car_pose['psi'], 0.0]
            stanley_controller.calc_theta_e_and_ef(vehicle_state)
            
            # Append timestamp and error values to the respective lists
            self.lateral_error_relative_to_planner_path['timestamp'].append(timestamp)
            self.lateral_error_relative_to_planner_path['values'].append(stanley_controller.ef)
            
            self.heading_error_relative_to_planner_path['timestamp'].append(timestamp)
            self.heading_error_relative_to_planner_path['values'].append(stanley_controller.psi_e)
            
            # Store the reference point used by the controller (using target_index)
            if stanley_controller.target_index is not None and stanley_controller.target_index < len(ref_path_x):
                self.aggregated_reference_points['timestamp'].append(timestamp)
                self.aggregated_reference_points['x'].append(ref_path_x[stanley_controller.target_index])
                self.aggregated_reference_points['y'].append(ref_path_y[stanley_controller.target_index])
                
                # Store the reference heading at the target point
                self.aggregated_reference_headings['timestamp'].append(timestamp)
                self.aggregated_reference_headings['values'].append(ref_heading[stanley_controller.target_index])
    def create_SN_data_structure(self, file_container):
        self.SN_files_container = file_container
        file_names = sorted(os.listdir(file_container))
        space_net_file_names = [file_name for file_name in file_names if 'batch_index' in file_name]
        self.space_net = {}
        for file_name in space_net_file_names:
            file_name_no_extension = file_name.split('.')[0]
            batch_index = file_name_no_extension.split('batch_index_')[1][0]
            timestamp_str = file_name_no_extension.split('_')[1]
            timestamp_str = timestamp_str.split('batch')[0]
            time_stamp = float(timestamp_str) / 1000 # conversion from miliseconds to seconds
            camera = file_name_no_extension.split('_')[-1]
            if time_stamp not in self.space_net:
                self.space_net[time_stamp] = {}
            if "batch_index_" + batch_index not in self.space_net[time_stamp]:
                self.space_net[time_stamp].__setitem__("batch_index_" + batch_index, {camera: {"file_name": file_name}})
            else:
                self.space_net[time_stamp]["batch_index_" + batch_index][camera] = {"file_name": file_name}
    def create_road_layer_data_structure(self, file_container):
        self.road_layer_files_container = file_container
        file_names = sorted(os.listdir(file_container))
        road_layer_file_names = [file_name for file_name in file_names if 'road' in file_name]
        self.road_layer = {}
        for file_name in road_layer_file_names:
            file_name_no_extension = file_name.split('.')[0]
            timestamp_str = file_name_no_extension.split('_')[-1]
            time_stamp = float(timestamp_str) / 1000
            self.road_layer.__setitem__(time_stamp, {"file_name": file_name})
    def create_lane_dividers_data_structure(self, file_container):
        self.lane_divider_files_container = file_container
        file_names = sorted(os.listdir(file_container))
        dashed_lane_divider_file_names = [file_name for file_name in file_names if
                                              'dashed_dividers' in file_name]
        self.dashed_lane_dividers = {}
        for file_name in dashed_lane_divider_file_names:
            file_name_no_extension = file_name.split('.')[0]
            timestamp_str = file_name_no_extension.split('_')[-1]
            time_stamp = float(timestamp_str) / 1000
            self.dashed_lane_dividers.__setitem__(time_stamp, {"file_name": file_name})
        continuous_lane_divider_file_names = [file_name for file_name in file_names if
                                              'continuous_dividers' in file_name]
        self.continuous_lane_dividers = {}
        for file_name in continuous_lane_divider_file_names:
            file_name_no_extension = file_name.split('.')[0]
            timestamp_str = file_name_no_extension.split('_')[-1]
            time_stamp = float(timestamp_str) / 1000
            self.continuous_lane_dividers.__setitem__(time_stamp, {"file_name": file_name})
    def create_vehicles_data_structure(self, file_container):
        self.vehicles_files_container = file_container
        file_names = sorted(os.listdir(file_container))
        vehicles_file_names = [file_name for file_name in file_names if 'vehicles' in file_name]
        self.vehicles = {}
        for file_name in vehicles_file_names:
            file_name_no_extension = file_name.split('.')[0]
            timestamp_str = file_name_no_extension.split('_')[-1]
            timestamp_str = timestamp_str.split('cars')[0]
            time_stamp = float(timestamp_str) / 1000
            self.vehicles.__setitem__(time_stamp, {"file_name": file_name})
    def create_MPC_data_structure(self, file_container):
        def find_MPC_solution_file():
            def latest_file(files):
                # Initialize the latest date to the smallest possible date
                latest_date = datetime.min
                latest_dir = None

                # Iterate over all items in the given path
                for file in files:
                    date_string = file.split('mpc_solution_')[1].split('.csv')[0]
                    # Try to parse as a date
                    date = datetime.strptime(date_string, '%Y-%m-%dT%H_%M_%S')

                    # If this directory's date is later than the latest date we've seen so far
                    if date > latest_date:
                        # Update the latest date and directory
                        latest_date = date
                        latest_file = file

                # Return the name of the directory with the latest date
                return latest_file
            files=[]
            for file in os.listdir(file_container):
                if "mpc_solution" in file:
                    files.append(file)
            if len(files) == 1:
                res = files[0]
            else:
                res = latest_file(files)
            return res
        def find_MPC_log_file():
            def latest_file(files):
                # Initialize the latest date to the smallest possible date
                latest_date = datetime.min
                latest_dir = None

                # Iterate over all items in the given path
                for file in files:
                    date_string = file.split('mpc_')[1].split('.csv')[0]
                    # Try to parse as a date
                    date = datetime.strptime(date_string, '%Y-%m-%dT%H_%M_%S')

                    # If this directory's date is later than the latest date we've seen so far
                    if date > latest_date:
                        # Update the latest date and directory
                        latest_date = date
                        latest_file = file

                # Return the name of the directory with the latest date
                return latest_file

            files = []
            for file in os.listdir(file_container):
                if ("mpc" in file) and not ("mpc_solution" in file):
                    files.append(file)
            if len(files) == 0:
                return None
            res = latest_file(files)
            return res
        self.MPC_solution_file_path = os.path.join(file_container, find_MPC_solution_file())
        self.MPC_log_file_path = None
        if find_MPC_log_file() is not None:
            self.MPC_log_file_path = os.path.join(file_container, find_MPC_log_file())
        # data_frame = pd.read_csv(self.MPC_file_path)
        self.MPC_solution= {}
        with open(self.MPC_solution_file_path, 'r') as file:
            csv_reader = csv.DictReader(file)
            for row in csv_reader:
                time_stamp = float(row.pop('timestamp'))
                self.MPC_solution[time_stamp] = {key: value for key, value in row.items()}
                for key, value in self.MPC_solution[time_stamp].items():
                    if key in ['velocity','velocity_error','acceleration','acceleration_derivative','steering','steering_derivative','heading','heading_error','cte', 'poly_vec']:
                        # remove parentheses
                        value = value[1:-1]
                        # split
                        value = value.split(' ')
                        value = [float(item) for item in value]
                        self.MPC_solution[time_stamp][key] = value
                for key, value in self.MPC_solution[time_stamp].items(): # we need to get the heading before the path
                    if key == 'path':
                        # remove parentheses
                        value = value[1:-1]# [0.000000 0.000000] [1.250000 0.000000] [2.324420 -0.017727] [3.256116 -0.041260] ...
                        # splited to coordinates
                        list_of_coords_strings = value.split('[') [1:]
                        path_coords_rear_axle_rear_frame = []
                        assert 'heading' in self.MPC_solution[time_stamp].keys()
                        heading_rad = np.array(self.MPC_solution[time_stamp]['heading'])
                        # heading_rad = np.pi / 180 * heading_deg
                        for cord_s in list_of_coords_strings:
                            cord_s = cord_s.split(']')[0]
                            cord_s = cord_s.split(' ')
                            path_coords_rear_axle_rear_frame.append([float(xy) for xy in cord_s])
                        path_coords_rear_axle_rear_frame = np.array(path_coords_rear_axle_rear_frame)
                        # path_coords_front_axle_front_frame = self.convert_path_from_MPC_to_control(path_coords_rear_axle_rear_frame, heading_rad)
                        self.MPC_solution[time_stamp][key] = np.array(path_coords_rear_axle_rear_frame)
                    if key == 'input_path':
                        # remove parentheses
                        value = value[1:-1]# [0.000000 0.000000] [1.250000 0.000000] [2.324420 -0.017727] [3.256116 -0.041260] ...
                        if len(value) == 0:
                            self.MPC_solution[time_stamp][key] = np.array([])
                            continue
                        # splited to coordinates
                        list_of_coords_strings = value.split('[') [1:]
                        path_coords_rear_axle_rear_frame = []
                        for cord_s in list_of_coords_strings:
                            cord_s = cord_s.split(']')[0]
                            cord_s = cord_s.split(' ')
                            path_coords_rear_axle_rear_frame.append([float(xy) for xy in cord_s])
                        path_coords_rear_axle_rear_frame = np.array(path_coords_rear_axle_rear_frame)
                        # path_coords_front_axle_front_frame = self.convert_path_from_MPC_to_control(path_coords_rear_axle_rear_frame)
                        path_coords_front_axle_front_frame = path_coords_rear_axle_rear_frame
                        self.MPC_solution[time_stamp][key] = np.array(path_coords_front_axle_front_frame)
        print('finished parsing MPC solution')
        if self.MPC_log_file_path is not None:
            self.MPC_log = {}
            with open(self.MPC_log_file_path, 'r') as file:
                csv_reader = csv.DictReader(file)
                for row in csv_reader:
                    time_stamp = float(row.pop('timestamp'))
                    self.MPC_log[time_stamp] = {key: value for key, value in row.items()}
                    for key, value in self.MPC_log[time_stamp].items():
                        if key in ['x0', 'x1', 'x2', 'x3']:
                            self.MPC_log[time_stamp][key] = float(value)
            print('finished parsing MPC log')
    def convert_path_from_MPC_to_control(self, path_points_rear_axle_in_rear_frame: np.array, psi_vec=None):
        """assuming path_points_rear_axle_in_rear_frame is NX2
        psi_vec in radians"""
        if psi_vec is None:
            _, _, psi_vec = Functions.calc_path_features(path_points_rear_axle_in_rear_frame[:,0],
                                                         path_points_rear_axle_in_rear_frame[:,1])
        WB = self.vehicle_params["WB"]
        front_axle_rear_frame = []
        static_vector_in_ego_frame = np.array([WB, 0]).reshape([2,1])
        for i in range(len(list(psi_vec))):
            R = np.array([[np.cos(psi_vec[i]), np.sin(psi_vec[i])], [-np.sin(psi_vec[i]), np.cos(psi_vec[i])]])
            front_axle_point = (path_points_rear_axle_in_rear_frame[i] + R.T.dot(static_vector_in_ego_frame).reshape([1, 2])).squeeze()
            front_axle_rear_frame.append(front_axle_point)
        front_axle_rear_frame = np.array(front_axle_rear_frame)
        front_axle_front_frame = front_axle_rear_frame - static_vector_in_ego_frame.reshape([2,])
        return front_axle_front_frame
    def convert_path_from_control_to_MPC(self, path_points_front_axle_in_front_frame: np.array, psi_vec=None):
        """assuming path_points_front_axle_in_front_frame is NX2
        psi_vec in radians"""
        if psi_vec is None:
            s, _, tangent_angles_vec = Functions.calc_path_features(path_points_front_axle_in_front_frame[:,0],
                                                         path_points_front_axle_in_front_frame[:,1])
        WB = self.vehicle_params["WB"]
        rear_axle_front_frame = []
        psi_i = 0
        static_vector_in_ego_frame = np.array([- WB, 0]).reshape([2,1])
        for i, (tan_ang_i, si) in enumerate(zip(tangent_angles_vec, s)): # range(len(list(tangent_angles_vec))):
            R = np.array([[np.cos(psi_i), np.sin(psi_i)], [-np.sin(psi_i), np.cos(psi_i)]])
            rear_axle_point = (path_points_front_axle_in_front_frame[i] + R.T.dot(static_vector_in_ego_frame).reshape([1, 2])).squeeze()
            rear_axle_front_frame.append(rear_axle_point)
            if i < len(s) - 1:
                ds = s[i+1] - si
                delta_i = tan_ang_i - psi_i
                psi_i += ds / WB * np.sin(delta_i)
        rear_axle_front_frame = np.array(rear_axle_front_frame)
        rear_axle_rear_frame = rear_axle_front_frame  - static_vector_in_ego_frame.reshape([2,])
        return rear_axle_rear_frame
    def get_MPC_solution_item(self, ti, side):
        MPC_times = np.array(list(self.MPC_solution.keys()))
        idx = Functions.search_time_vector(MPC_times, ti, side)
        key = list(self.MPC_solution.keys())[idx]
        print("MPC solution time = " + str(key))
        return self.MPC_solution[key]
    def get_MPC_log_item(self, ti, side):
        MPC_times = np.array(list(self.MPC_log.keys()))
        idx = Functions.search_time_vector(MPC_times, ti, side)
        key = list(self.MPC_log.keys())[idx]
        print("MPC poly time = " + str(key))
        return self.MPC_log[key]
    def get_space_net_item(self, ti, side, batch_index, camera):
        space_net_times = np.array(list(self.space_net.keys()))
        idx = Functions.search_time_vector(space_net_times, ti, side)
        key = list(self.space_net.keys())[idx]
        file_name = self.space_net[key]["batch_index_" + batch_index][camera]["file_name"]
        file_path = Path(self.SN_files_container) / file_name
        IM = Image.open(file_path)
        # Convert the image to grayscale
        img_gray = IM.convert('L')
        # Convert the grayscale image to a NumPy array
        img_array = 255 - np.rot90(np.array(img_gray), 1)
        self.space_net[key]["batch_index_" + batch_index][camera]["image"] = img_array
        return self.space_net[key]["batch_index_" + batch_index][camera]["image"]
    def get_road_layer_item(self, ti, side):
        road_layer_times = np.array(list(self.road_layer.keys()))
        idx = Functions.search_time_vector(road_layer_times, ti, side)
        key = list(self.road_layer.keys())[idx]
        file_name = self.road_layer[key]["file_name"]
        file_path = Path(self.road_layer_files_container) / file_name
        IM = Image.open(file_path)
        # Convert the image to grayscale
        img_gray = IM.convert('L')
        # Convert the grayscale image to a NumPy array
        img_array = np.rot90(np.array(img_gray), 1)
        self.road_layer[key]["image"] = img_array
        return self.road_layer[key]["image"]
    def get_continuous_lane_dividers_item(self, ti, side):
        continuous_lane_dividers_times = np.array(list(self.continuous_lane_dividers.keys()))
        idx = Functions.search_time_vector(continuous_lane_dividers_times, ti, side)
        key = list(self.continuous_lane_dividers.keys())[idx]
        file_name = self.continuous_lane_dividers[key]["file_name"]
        file_path = Path(self.road_layer_files_container) / file_name
        IM = Image.open(file_path)
        # Convert the image to grayscale
        img_gray = IM.convert('L')
        # Convert the grayscale image to a NumPy array
        img_array = np.rot90(np.array(img_gray), 1)
        self.continuous_lane_dividers[key]["image"] = img_array
        return self.continuous_lane_dividers[key]["image"]
    def get_dashed_lane_dividers_item(self, ti, side):
        dashed_lane_dividers_times = np.array(list(self.dashed_lane_dividers.keys()))
        idx = Functions.search_time_vector(dashed_lane_dividers_times, ti, side)
        key = list(self.dashed_lane_dividers.keys())[idx]
        file_name = self.dashed_lane_dividers[key]["file_name"]
        file_path = Path(self.road_layer_files_container) / file_name
        IM = Image.open(file_path)
        # Convert the image to grayscale
        img_gray = IM.convert('L')
        # Convert the grayscale image to a NumPy array
        img_array = np.rot90(np.array(img_gray), 1)
        self.dashed_lane_dividers[key]["image"] = img_array
        return self.dashed_lane_dividers[key]["image"]
    def get_vehicles_item(self, ti, side):
        vehicles_times = np.array(list(self.vehicles.keys()))
        idx = Functions.search_time_vector(vehicles_times, ti, side)
        key = list(self.vehicles.keys())[idx]
        file_name = self.vehicles[key]["file_name"]
        file_path = Path(self.vehicles_files_container) / file_name
        IM = Image.open(file_path)
        # Convert the image to grayscale
        img_gray = IM.convert('L')
        # Convert the grayscale image to a NumPy array
        img_array = 255 - np.rot90(np.array(img_gray), 1)
        self.vehicles[key]["image"] = img_array
        return self.vehicles[key]["image"]
    def plot_space_net(self, ax: plt.axis, sys_time):
        space_net_image_forward = self.get_space_net_item(
            sys_time,
            "closest_value", "0", "forward")
        space_net_image_right = self.get_space_net_item(
            sys_time,
            "closest_value", "0", "right")
        space_net_image_left = self.get_space_net_item(
            sys_time,
            "closest_value", "0", "left")
        space_net_image = np.clip(3 * 255 - space_net_image_forward.astype(np.uint16) - space_net_image_right.astype(np.uint16) - space_net_image_left.astype(np.uint16),
                                  0, 255)

        # Get the dimensions of the image
        image_height, image_width = space_net_image.shape
        # Define the extent of the image
        x_range_meters = self.pixels_to_meters_space_net(image_width)
        y_range_meters = self.pixels_to_meters_space_net(image_height)
        extent = (-x_range_meters / 2, x_range_meters / 2, 0.0, y_range_meters)
        # Plot the image with the adjusted extent
        self.space_net_image = ax.imshow(space_net_image, cmap='Reds', extent=extent, alpha=0.3)
    def plot_road_layer(self, ax: plt.axis, sys_time):
        road_layer_image = self.get_road_layer_item(
            sys_time, "closest_value")
        # Get the dimensions of the image
        image_height, image_width = road_layer_image.shape
        # Define the extent of the image
        x_range_meters = self.pixels_to_meters_space_net(image_width)
        y_range_meters = self.pixels_to_meters_space_net(image_height)
        extent = (-x_range_meters / 2, x_range_meters / 2, 0.0, y_range_meters)
        # Plot the image with the adjusted extent
        self.road_layer_image = ax.imshow(road_layer_image, cmap='Greens', extent=extent, alpha=0.3)
    def plot_continuous_lane_dividers(self, ax: plt.axis, sys_time):
        continuous_lane_dividers_image = self.get_continuous_lane_dividers_item(
            sys_time, "closest_value")
        # Get the dimensions of the image
        image_height, image_width = continuous_lane_dividers_image.shape
        # Define the extent of the image
        x_range_meters = self.pixels_to_meters_space_net(image_width)
        y_range_meters = self.pixels_to_meters_space_net(image_height)
        extent = (-x_range_meters / 2, x_range_meters / 2, 0.0, y_range_meters)
        # Plot the image with the adjusted extent
        self.continuous_lane_dividers_image = ax.imshow(continuous_lane_dividers_image, cmap='YlOrRd', extent=extent, alpha=0.3)
    def plot_dashed_lane_dividers(self, ax: plt.axis, sys_time):
        dashed_lane_dividers_image = self.get_dashed_lane_dividers_item(
            sys_time, "closest_value")
        # Get the dimensions of the image
        image_height, image_width = dashed_lane_dividers_image.shape
        # Define the extent of the image
        x_range_meters = self.pixels_to_meters_space_net(image_width)
        y_range_meters = self.pixels_to_meters_space_net(image_height)
        extent = (-x_range_meters / 2, x_range_meters / 2, 0.0, y_range_meters)
        # Plot the image with the adjusted extent
        self.dashed_lane_dividers_image = ax.imshow(dashed_lane_dividers_image, cmap='YlOrRd', extent=extent, alpha=0.3)
    def plot_path_extraction(self, ax: plt.axis, sys_time):
        path_extraction_obj = self.get_path_extraction_item(sys_time, "closest_value")
        # side = left because we are looking for time that precedes  MotionPlanningUpdate call time
        self.path_extraction_scatter = ax.scatter(-np.array(path_extraction_obj['y']), path_extraction_obj['x'], s = 10)
        return self.path_extraction_scatter
    def plot_path_adjustment(self, ax: plt.axis, sys_time):
        path_adjustment_obj = self.get_path_adjustment_item(sys_time, "closest_value")
        # side = left because we are looking for time that precedes  MotionPlanningUpdate call time
        self.path_adjustment_scatter = ax.scatter(-np.array(path_adjustment_obj['y']), path_adjustment_obj['x'], s = 10)
        return self.path_adjustment_scatter
    def plot_path_trajectory(self, ax: plt.axis, sys_time):
        path_trajectory_obj = self.get_path_trajectory_item(sys_time, "closest_value")
        # side = left because we are looking for time that precedes  MotionPlanningUpdate call time
        self.path_trajectory_scatter = ax.scatter(-np.array(path_trajectory_obj['y']), path_trajectory_obj['x'], s = 10)
        return self.path_trajectory_scatter
    def plot_path_trajectory_ENU_rear_control(self, ax: plt.axis, sys_time):
        # side = left because we are looking for time that precedes  MotionPlanningUpdate call time
        path_trajectory_obj = self.get_path_trajectory_item(sys_time, "closest_value")
        path_trajectory = np.vstack([path_trajectory_obj['x'], path_trajectory_obj['y']]).T
        # saved as front axle in front frame in ENU
        path_trajectory_obj_rear_control = self.convert_path_from_control_to_MPC(path_trajectory)# front to rear
        self.path_trajectory_scatter = ax.scatter(path_trajectory_obj_rear_control[:,0], path_trajectory_obj_rear_control[:,1], s = 10)
        return self.path_trajectory_scatter
    # def plot_path_polyfit(self, ax: plt.axis, sys_time):
    #     # side = left because we are looking for time that precedes  MotionPlanningUpdate call time
    #     path_trajectory_obj = self.get_path_trajectory_item(sys_time, "left")
    #     poly_x = path_trajectory_obj['x']
    #     MPC_log_item = self.get_MPC_log_item(sys_time, "left")
    #     poly_coefs = [MPC_log_item["x0"], MPC_log_item["x1"], MPC_log_item["x2"], MPC_log_item["x3"]]
    #     poly_y = []
    #     for x in poly_x:
    #         poly_y.append(poly_coefs[0] + poly_coefs[1] * x + poly_coefs[2] * x**2 + poly_coefs[3] * x**3)
    #     self.poly_path_scatter = ax.scatter(-np.array(poly_y), poly_x, s = 10)
    #     return self.path_trajectory_scatter
    def plot_path_polyfit(self, ax: plt.axis, MPC_solution_item):
        poly_coefs = MPC_solution_item["poly_vec"]
        poly_x = MPC_solution_item['input_path'][:, 0]
        poly_y = []
        for x in poly_x:  
            poly_y.append(poly_coefs[0] + poly_coefs[1] * x + poly_coefs[2] * x**2 + poly_coefs[3] * x**3)
        # regression is performed in rear axle rear frame
        poly_path_rear = np.vstack([poly_x, poly_y]).T
        # poly_path_front = self.convert _path_from_MPC_to_control(poly_path_rear)
        poly_path_front = poly_path_rear
        self.poly_path_scatter = ax.scatter(poly_path_rear[:,0], poly_path_rear[:,1], s = 10)
        return self.path_trajectory_scatter
    def plot_MPC_solution(self, ax: plt.axis, sys_time):
        MPC_times = np.array(list(self.MPC_solution.keys()))
        idx = Functions.search_time_vector(MPC_times, sys_time, "closest_value")
        key = list(self.MPC_solution.keys())[idx]
        print("MPC trajectory time = " + str(key))
        MPC_solution_path_x = self.MPC_solution[key]['path'][:, 0]
        MPC_solution_path_y = self.MPC_solution[key]['path'][:, 1]
        #MPC path is calculated in rear axle control point. convert to front for this visualization
        MPC_solution_path_rear = np.vstack([MPC_solution_path_x, MPC_solution_path_y]).T
        MPC_solution_path_front = self.convert_path_from_MPC_to_control(MPC_solution_path_rear)
        self.MPC_path_scatter = ax.scatter(-MPC_solution_path_front[:,1], MPC_solution_path_front[:,0], s=25, marker='o')
        # minus because ENU -> Ned
        return self.MPC_path_scatter
    def pixels_to_meters_space_net(self, pixels):
        return pixels / self.kmtx
    def plot_traveled_path(self):
        traveled_path_x = [self.control_localization[key]['x'] for key in self.control_localization]
        traveled_path_y = [self.control_localization[key]['y'] for key in self.control_localization]
        self.traveled_path_line = plt.plot(traveled_path_x, traveled_path_y)
    def plot_heading(self):
        t = [key for key in self.control_localization]
        psi = [self.control_localization[key]['psi'] for key in self.control_localization]
        plt.plot(t, psi)
    def initialize_gui(self):
        self.figure = plt.figure('Ego Frame View')
        # Register callback function to when figure is closed
        self.figure.canvas.mpl_connect('close_event', self.figure_closed)
        self.main_ax = self.figure.add_axes([0.15, 0.05, 0.8, 0.95])
        # Create a CheckButtons widget
        check_box_ax = plt.axes([0.05, 0.8, 0.1, 0.15])
        self.check_box_labels = ('road layer',
                                 'space net',
                                 'cont lane div',
                                 'dashed lane div',
                                 'path extraction',
                                 'path adjustment',
                                 'path trajectory',
                                 'MPC trajectory',
                                 'traveled path')
        self.check_box_statuses = {'road layer': True,
                                   'space net': True,
                                   'cont lane div': True,
                                   'dashed lane div': True,
                                   'path extraction': True,
                                   'path adjustment': True,
                                   'path trajectory': True,
                                   'MPC trajectory': True,
                                   'traveled path': True}
        check_box_statuses_list = [value for value in self.check_box_statuses.values()]
        self.check_boxes = CheckButtons(check_box_ax, labels=self.check_box_labels,
                                        actives=check_box_statuses_list)
        # Connect the function to the CheckButtons widget
        self.check_boxes.on_clicked(self.check_box_clicked)
        # ax for MPC button placement
        if self.MPC_solution is not None:
            mpc_button_ax = self.figure.add_axes([0.05, 0.6, 0.1, 0.15])
            self.MPC_button = Button(mpc_button_ax, 'MPC sol.')
            # Connect the button to the callback function
            self.MPC_button.on_clicked(self.MPC_button_callback)
        self.main_ax.grid(True), self.main_ax.set_xlabel('y [m]'), self.main_ax.set_ylabel('x [m]')
        self.main_ax.set_xlim([-self.space_net_x_range_meters / 2, self.space_net_x_range_meters / 2])
        self.main_ax.set_ylim([0.0, self.space_net_y_range_meters])
        self.gui_initialized = True
    def MPC_button_callback(self, val):
        print("data_timestamp = " + str(self.data_timestamp))
        MPC_solution_item = self.get_MPC_solution_item(self.data_timestamp, "closest_value")
        animation_figure = plt.figure()
        vehicle_animation_axis = plt.subplot(1, 1, 1)
        plt.title("MPC path processing in ENU (positive left)")
        self.plot_path_trajectory_ENU_rear_control(vehicle_animation_axis, self.data_timestamp)# imported from MP trajectory -> forward axle forward frame
        if self.MPC_log_file_path is not None:
            self.plot_path_polyfit(vehicle_animation_axis, MPC_solution_item) # converted to front
        # vehicle_animation_axis.axis("equal")
        vehicle_animation_axis.grid(True)
        vehicle_animation_axis.set_ylabel('x [m]')
        vehicle_animation_axis.set_xlabel('y [m]')
        MPC_solution_path_x = MPC_solution_item['path'][:, 0]# forward axle
        MPC_solution_path_y = MPC_solution_item['path'][:, 1]
        vehicle_animation_axis.scatter(MPC_solution_path_x, MPC_solution_path_y, s=25, marker='o')
        if len(MPC_solution_item['input_path'].shape) > 1:
            MPC_input_path_x = MPC_solution_item['input_path'][:, 0]
            MPC_input_path_y = MPC_solution_item['input_path'][:, 1]
            vehicle_animation_axis.scatter(MPC_input_path_x, MPC_input_path_y, s=25, marker='x')# imported from MPC solution transformed to forward axle
            legend = ['MP traj']
            if self.MPC_log_file_path is not None:
                legend.append('poly path')
            legend.extend(['MPC solution','compensated input to MPC'])
            plt.legend(legend)
        else:
            legend = ['MP traj']
            if self.MPC_log_file_path is not None:
                legend.append('poly path')
            legend.extend(['MPC solution'])
            plt.legend(legend)
        # vehicle_animation_axis.axis("equal")

        plt.figure('state')
        plt.subplot(2, 1, 1)
        plt.plot(MPC_solution_item['cte'])
        plt.ylabel("cte [m]")
        plt.grid(True)
        plt.subplot(2, 1, 2)
        plt.plot(MPC_solution_item['heading_error'])
        plt.ylabel("heading_error [rad]")
        plt.grid(True)

        plt.figure('actuation')
        plt.subplot(221)
        plt.plot(MPC_solution_item['acceleration'])
        plt.ylabel(r'$a [m/sec^2]$')
        plt.grid(True)
        plt.subplot(223)
        plt.plot(MPC_solution_item['acceleration_derivative'])
        plt.ylabel(r'$da/dt [m/sec^3]$')
        plt.grid(True)
        plt.subplot(222)
        plt.plot(MPC_solution_item['steering'])
        plt.ylabel(r'$\delta [rad]$')
        plt.grid(True)
        plt.subplot(224)
        plt.plot(MPC_solution_item['steering_derivative'])
        plt.ylabel(r'$d\delta/dt [rad/sec]$')
        plt.grid(True)
        plt.show()
    def calc_traveled_path_ego(self, sys_time_at_MotionPlanningUpdate_call):
        if self.control_localization is not None:
            traveled_path_x = [self.control_localization[key]['x'] for key in self.control_localization]
            traveled_path_y = [self.control_localization[key]['y'] for key in self.control_localization]
            traveled_path_nav = np.vstack([np.array(traveled_path_x), np.array(traveled_path_y)]).T
            current_car_pose = self.get_control_localization_item(sys_time_at_MotionPlanningUpdate_call, 'left')
        else:
            traveled_path_x = [self.car_pose[key]['x'] for key in self.car_pose]
            traveled_path_y = [self.car_pose[key]['y'] for key in self.car_pose]
            traveled_path_nav = np.vstack([np.array(traveled_path_x), np.array(traveled_path_y)]).T
            current_car_pose = self.get_car_pose_item(sys_time_at_MotionPlanningUpdate_call, 'left')

        current_car_position_x, current_car_position_y = (current_car_pose['x'], current_car_pose['y'])
        psi = current_car_pose['psi']
        Tnav2ego = Functions.affine_transformation_matrix_2D(current_car_position_x, current_car_position_y, psi)
        traveled_path_ego = Functions.project_points_2D(Tnav2ego, traveled_path_nav)
        return traveled_path_ego
    def plot_ego_frame(self, data_timestamp, control_path=None):
        self.data_timestamp = data_timestamp # save for MPC debug
        if self.gui_initialized:
            plt.close(self.figure)
        self.initialize_gui()
        if self.road_layer is not None:
            self.plot_road_layer(self.main_ax, data_timestamp)
        if self.space_net is not None:
            self.plot_space_net(self.main_ax, data_timestamp)
        if self.continuous_lane_dividers is not None:
            self.plot_continuous_lane_dividers(self.main_ax, data_timestamp)
        if self.dashed_lane_dividers is not None:
            self.plot_dashed_lane_dividers(self.main_ax, data_timestamp)
        if self.MPC_solution is not None:
            self.plot_MPC_solution(self.main_ax, data_timestamp)
        self.plot_path_extraction(self.main_ax, data_timestamp)
        self.plot_path_adjustment(self.main_ax, data_timestamp)
        self.plot_path_trajectory(self.main_ax, data_timestamp)
        traveled_path_ego = self.calc_traveled_path_ego(data_timestamp)
        self.traveled_path_ego_scatter = self.main_ax.scatter(traveled_path_ego[:, 1], traveled_path_ego[:, 0], s=1)
        # Create legend
        RL_patch = mpatches.Patch(color='green', label='road layer')
        SN_patch = mpatches.Patch(color='red', label='space net')
        dahsed_LD_patch = mpatches.Patch(color='yellow', label='space net')
        cont_LD_patch = mpatches.Patch(color='yellow', label='space net')
        handles = [RL_patch, SN_patch, dahsed_LD_patch, cont_LD_patch, self.path_extraction_scatter,
                   self.path_adjustment_scatter,
                   self.path_trajectory_scatter, self.traveled_path_ego_scatter]
        labels = ['road_layer', 'space net', 'cont lane dividers', 'dashed lane dividers', 'path extraction',
                  'path adjustment', 'trajectory path',
                  'traveled_path_ego']
        if self.MPC_solution is not None:
            handles.append(self.MPC_path_scatter)
            labels.append('MPC_path_scatter')
        if control_path is not None:
            self.control_path = self.main_ax.scatter(control_path["y"], control_path['x'], s=10, color='black')
            handles.append(self.control_path)
            labels.append('control path')
        self.figure.legend(handles=handles, labels=labels, loc=(0.01, 0.01))
    def update_check_box_status(self):
        for label, state in zip(self.check_box_labels, self.check_boxes.get_status()):
            self.check_box_statuses[label] = state
    def check_box_clicked(self, label):
        self.update_check_box_status()
        # Define a function to be called when a checkbox is clicked
        print('\n Current state:')
        for key in self.check_box_statuses:
            print(f'{key}: {self.check_box_statuses[key]}')
        if self.check_box_statuses["road layer"]:
            self.road_layer_image.set_visible(True)
        else:
            self.road_layer_image.set_visible(False)

        if self.check_box_statuses["space net"]:
            self.space_net_image.set_visible(True)
        else:
            self.space_net_image.set_visible(False)
        if self.check_box_statuses["cont lane div"]:
            self.continuous_lane_dividers_image.set_visible(True)
        else:
            self.continuous_lane_dividers_image.set_visible(False)
        if self.check_box_statuses["dashed lane div"]:
            self.dashed_lane_dividers_image.set_visible(True)
        else:
            self.dashed_lane_dividers_image.set_visible(False)
        if self.check_box_statuses["path extraction"]:
            self.path_extraction_scatter.set_visible(True)
        else:
            self.path_extraction_scatter.set_visible(False)
        if self.check_box_statuses["path adjustment"]:
            self.path_adjustment_scatter.set_visible(True)
        else:
            self.path_adjustment_scatter.set_visible(False)
        if self.check_box_statuses["path trajectory"]:
            self.path_trajectory_scatter.set_visible(True)
        else:
            self.path_trajectory_scatter.set_visible(False)
        if self.check_box_statuses["traveled path"]:
            self.traveled_path_ego_scatter.set_visible(True)
        else:
            self.traveled_path_ego_scatter.set_visible(False)
        if self.check_box_statuses["road layer"] or self.check_box_statuses["space net"]:
            self.main_ax.set_aspect('equal')
        else:
            self.main_ax.set_aspect('auto')

        plt.draw()
    def figure_closed(self, event):
        self.gui_initialized = False
        print('motion planning gui closed')
class ExperimentAnalysisTool:
    def __init__(self, car_params, desired_traj_x=None, desired_traj_y=None, flip_xy=True, initialize_plot=True):
        self.BEV_range_x = 50.0
        self.BEV_range_y = 50.0
        self.minimal_margin_x = 20.0
        self.minimal_margin_y = 20.0
        self.traveled_path_x = []
        self.traveled_path_y = []
        self.car_params = car_params
        self.time_line = []
        self.lateral_error = []
        self.heading_reference_raw = []
        self.heading_reference_filtered = []
        self.vehicle_heading = []
        self.vehicle_heading_from_control = []
        self.heading_error = []
        self.heading_error = []
        self.steering = []
        self.car_speed_mps = []
        self.steering_cmd_raw = []
        self.steering_cmd_filtered = []
        self.time_line_visible_frame = 60
        self.flip_xy = flip_xy
        self.aggregated_control_path = None
        self.time_shift = 0.1  # Default time shift in seconds for keyboard navigation
        if flip_xy:
            self.desired_traj_x = desired_traj_y
            self.desired_traj_y = desired_traj_x
            self.steering_sf_in_BEV = -1.0
        else:
            self.desired_traj_x = desired_traj_x
            self.desired_traj_y = desired_traj_y
            self.steering_sf_in_BEV = 1.0
        if initialize_plot:
            self.initialize_plot()
            self.plot_initialized = True
        else:
            self.plot_initialized = False
        self.motion_planning_debug_obj = None
        self.analyse_lateral_control = False
        self.analyse_motion_planning = False
        self.driving_mode_data = None
        # Map driving mode value to descriptive text
        self.mode_descriptions = {
            0: "Manual",
            1: "Steering",
            6: "Speed",
            7: "Full"
        }
        
        # Map driving mode value to color
        self.mode_colors = {
            0: "red",     # Manual
            1: "orange",  # Steering
            6: "yellow",  # Speed
            7: "green"    # Full control
        }
    def initialize_plot_lateral_control(self):
        self.animation_figure = plt.figure('Experiment Analysis Tool', figsize=(30, 15))
        atexit.register(self.close_figure)
        signal.signal(signal.SIGTERM, self.handle_signal)
        signal.signal(signal.SIGINT, self.handle_signal)
        
        # Connect the keyboard event handler
        self.animation_figure.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.vehicle_animation_axis = plt.subplot(1, 2, 1)
        self.vehicle_animation_axis.grid(True)
        self.vehicle_animation_axis.set_xlim(- self.BEV_range_x, + self.BEV_range_x)
        self.vehicle_animation_axis.set_ylim(- self.BEV_range_y, + self.BEV_range_y)
        self.lateral_error_axis = plt.subplot(4, 2, 2)
        self.heading_control_axis = plt.subplot(4, 2, 4, sharex=self.lateral_error_axis)
        self.steering_axis = plt.subplot(4, 2, 6, sharex=self.lateral_error_axis)
        self.car_speed_axis = plt.subplot(4, 2, 8, sharex=self.lateral_error_axis)
        # Adjust the main plot to make room for the slider and button
        self.animation_figure.subplots_adjust(bottom=0.25)
        #   SLIDER
        # Create a horizontal slider to control the position
        self.slider_axis = self.animation_figure.add_axes([0.25, 0.1, 0.65, 0.03])
        self.slider = Slider(
            ax=self.slider_axis,
            label='Time [sec]',
            valmin=self.common_time_line[0],
            valmax=self.common_time_line[-1],
            valinit=self.common_time_line[0],
        )
        # Register the update function with the slider
        self.slider.on_changed(self.slider_callback)
        # BUTTON
        # Define the button's position and size
        button_ax = self.animation_figure.add_axes([0.1, 0.1, 0.1, 0.075])
        self.button = Button(button_ax, 'Ego View')
        # Connect the button to the callback function
        self.button.on_clicked(self.button_callback)
        self.plot_initialized = True
    def initialize_plot_default(self):
        self.animation_figure = plt.figure('Experiment Analysis Tool', figsize=(30, 15))
        atexit.register(self.close_figure)
        signal.signal(signal.SIGTERM, self.handle_signal)
        signal.signal(signal.SIGINT, self.handle_signal)
        
        # Connect the keyboard event handler
        self.animation_figure.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # Create a 2x2 grid of subplots
        # Main plot takes the entire left column
        self.vehicle_animation_axis = plt.subplot(2, 2, (1, 3))
        self.vehicle_animation_axis.grid(True)
        self.vehicle_animation_axis.set_xlim(- self.BEV_range_x, + self.BEV_range_x)
        self.vehicle_animation_axis.set_ylim(- self.BEV_range_y, + self.BEV_range_y)
        self.vehicle_animation_axis.set_aspect('equal')
        
        # Add subplots on the right
        # 3-row layout to accommodate driving mode visualization
        self.lateral_error_axis = plt.subplot(3, 2, 2)
        self.lateral_error_axis.grid(True)
        self.lateral_error_axis.set_ylabel('Lateral Error [m]')
        
        self.heading_error_axis = plt.subplot(3, 2, 4, sharex=self.lateral_error_axis)
        self.heading_error_axis.grid(True)
        self.heading_error_axis.set_ylabel('Heading Error [rad]')
        
        # Add a new subplot for driving mode visualization
        self.driving_mode_axis = plt.subplot(3, 2, 6, sharex=self.lateral_error_axis)
        self.driving_mode_axis.grid(True)
        self.driving_mode_axis.set_ylabel('Driving Mode')
        self.driving_mode_axis.set_xlabel('Time [sec]')
        
        # Adjust the main plot to make room for the slider and button
        self.animation_figure.subplots_adjust(bottom=0.25)
        self.slider_axis = self.animation_figure.add_axes([0.25, 0.1, 0.65, 0.03])
        self.slider = Slider(
            ax=self.slider_axis,
            label='Time [sec]',
            valmin=self.common_time_line[0],
            valmax=self.common_time_line[-1],
            valinit=self.common_time_line[0],
        )
        # Register the update function with the slider
        self.slider.on_changed(self.slider_callback)
        button_ax = self.animation_figure.add_axes([0.1, 0.1, 0.1, 0.075])
        self.button = Button(button_ax, 'Ego View')
        # Connect the button to the callback function
        self.button.on_clicked(self.button_callback)
        self.plot_initialized = True
    def load_lateral_control_files(self, steering_control_data,
                                   localization_data,
                                   path_processing_data):
        self.steering_control_data = steering_control_data
        self.localization_data = localization_data
        self.path_processing_data = path_processing_data
        self.steering_control_time = np.array(self.steering_control_data['time [sec]'])
        self.localization_time = np.array(self.localization_data['time [sec]'])
        self.path_processing_time = np.array(list(self.path_processing_data.keys()))
        self.common_time_line = Functions.align_time_vectors([self.steering_control_time,
                                                              self.localization_time])
                                                            #   self.path_processing_time])
        self.t0 = self.common_time_line[0]
        self.common_time_line -= self.t0
        self.steering_control_time_zeroed = self.steering_control_time - self.t0
        self.localization_time_zeroed = self.localization_time - self.t0
        self.path_processing_time_zeroed = self.path_processing_time - self.t0
        self.aggregated_control_path = np.vstack([np.array(self.steering_control_data['target_point_x [m]']),
                                                  np.array(self.steering_control_data['target_point_y [m]'])]).T # n by 2
        self.heading_reference_raw = np.array(self.steering_control_data['reference_heading_raw [rad]'])
        self.heading_reference_filtered = np.array(self.steering_control_data['reference_heading_filtered [rad]'])
        self.vehicle_heading_feedback = np.array(self.steering_control_data['vehicle_heading [rad]'])
        self.heading_error = np.array(self.steering_control_data['heading_error [rad]'])
        self.lateral_error = np.array(self.steering_control_data['lateral_error [m]'])
        self.steering_cmd_raw = (np.array(self.steering_control_data['steering_wheel_cmd_raw [rad]']))
        self.steering_cmd_filtered = np.array(self.steering_control_data['steering_wheel_cmd_filtered [rad]'])
        # self.steering_cmd_raw = (np.array(self.steering_control_data['steering_wheel_cmd_raw [rad]']) /
        #                          self.car_params['steering_ratio'])
        # self.steering_cmd_filtered = np.array(self.steering_control_data['steering_wheel_cmd_filtered [rad]']/
        #                          self.car_params['steering_ratio'])
        self.traveled_path_x = np.array(self.localization_data['x [m]'])
        self.traveled_path_y = np.array(self.localization_data['y [m]'])
        self.vehicle_heading = Functions.continuous_angle(np.array(self.localization_data['psi [rad]']))  
        self.vehicle_heading_from_control = np.array(self.steering_control_data['vehicle_heading [rad]'])
        self.steering = np.array(self.localization_data['steering [rad]'])
        self.car_speed_mps = np.array(self.localization_data['vel [mps]'])
        self.analyse_lateral_control = True
    def load_motion_planning(self, motion_planning_debug_obj: MotionPlanningDebug):
        if not self.analyse_lateral_control:
            self.traveled_path_x = [motion_planning_debug_obj.car_pose[key]["x"] for key in motion_planning_debug_obj.car_pose]
            self.traveled_path_y = [motion_planning_debug_obj.car_pose[key]["y"] for key in motion_planning_debug_obj.car_pose]
            self.common_time_line = np.array(list(motion_planning_debug_obj.car_pose.keys()))
            self.t0 = self.common_time_line[0]
            self.common_time_line -= self.t0
        self.motion_planning_debug_obj = motion_planning_debug_obj
        if motion_planning_debug_obj.MPC_solution is not None:
            # get MPC steering
            steering_times = []
            self.MPC_steering_times = [ti for ti in motion_planning_debug_obj.MPC_solution.keys()]
            self.MPC_steering = [value['steering'][0] for value in list(motion_planning_debug_obj.MPC_solution.values())]
    def parse_driving_mode(self, file_path):
        """
        Parse driving mode data from a CSV file
        Args:
            file_path: path to the CSV file containing driving mode data
        Returns:
            Dictionary with parsed driving mode data containing:
                - timestamps: list of timestamps from time_stamp column
                - values: list of all data from data_value column
                - timestamp_value: dictionary with timestamp as key and driving_mode as value
        """
        print(f"Parsing driving mode data from: {file_path}")
        try:
            # Read the CSV file
            df = pd.read_csv(Path(file_path) / "driving_mode.csv")
            
            # Extract the required data
            timestamps = list(df['time_stamp'])
            values = list(df['data_value'])
            
            # Create timestamp to value dictionary
            timestamp_value = {}
            for i in range(len(timestamps)):
                timestamp_value[timestamps[i]] = values[i]
            
            # Store the data in a dictionary
            self.driving_mode_data = {
                'timestamps': timestamps,
                'values': values,
                'timestamp_value': timestamp_value
            }
            
            print(f"Successfully parsed driving mode data with {len(timestamps)} entries")
            return self.driving_mode_data
            
        except Exception as e:
            print(f"Error parsing driving mode data: {str(e)}")
            return None
    def get_driving_mode_item(self, ti, side):
        """
        Get the driving mode value at a specific time
        
        Args:
            ti: The system time to search for
            side: Which side to search on if exact time isn't found:
                  'left' - return the mode just before the requested time
                  'right' - return the mode just after the requested time
                  'closest_value' - return the mode closest to the requested time
        
        Returns:
            The driving mode value at the requested time, or None if no driving mode data exists
        """
        if self.driving_mode_data is None:
            print("Warning: No driving mode data available")
            return None
            
        driving_mode_times = np.array(self.driving_mode_data['timestamps'])
        
        # Handle edge cases where the requested time is outside the range of available data
        if len(driving_mode_times) == 0:
            return None
        if ti < driving_mode_times[0]:
            return self.driving_mode_data['values'][0]
        if ti > driving_mode_times[-1]:
            return self.driving_mode_data['values'][-1]
            
        # Find the appropriate index based on the requested side
        idx = Functions.search_time_vector(driving_mode_times, ti, side)
        if idx < 0 or idx >= len(driving_mode_times):
            print(f"Warning: Time index {idx} out of range for driving mode data")
            return None
            
        # Return the driving mode value at the found index
        return self.driving_mode_data['values'][idx]
    def slider_callback(self, val):
        if self.analyse_lateral_control:
            self.update_visualization_lateral_control()
        else:
            self.update_visualization_default()
    def update_visualization_default(self):
        t_slider = self.slider.val
        system_time = t_slider + self.t0
        current_car_pose = self.motion_planning_debug_obj.get_car_pose_item(system_time, "left")

        self.vehicle_animation_axis.clear()
        if self.flip_xy:
            # x,y = y,x swaps variables
            current_car_position_x, current_car_position_y = (current_car_pose['y'], current_car_pose['x'])
            traveled_path_x, traveled_path_y = (self.traveled_path_y, self.traveled_path_x)
            current_car_heading_for_BEV = np.pi/2 -  current_car_pose['psi']
            if self.motion_planning_debug_obj is not None and \
                self.motion_planning_debug_obj.planner_path_in_world_frame is not None:
                planner_path_in_world_frame = \
                    self.motion_planning_debug_obj.get_planner_path_in_world_frame_item(
                        system_time, "left"
                        )
                planner_path_world_y, planner_path_world_x = (
                    planner_path_in_world_frame['x'],
                    np.array(planner_path_in_world_frame['y'])
                )

        else:
            current_car_position_x, current_car_position_y = (current_car_pose['x'], current_car_pose['y'])
            traveled_path_x, traveled_path_y = (self.traveled_path_x, self.traveled_path_y)
            current_car_heading_for_BEV = current_car_pose['psi']
            if self.motion_planning_debug_obj is not None and \
                self.motion_planning_debug_obj.planner_path_in_world_frame is not None:
                planner_path_in_world_frame = \
                    self.motion_planning_debug_obj.get_planner_path_in_world_frame_item(
                        system_time, "left"
                        )
                planner_path_world_x, planner_path_world_y = (
                    planner_path_in_world_frame['x'],
                    np.array(planner_path_in_world_frame['y'])
                )
        self.vehicle_animation_axis.set_title('t = ' + str("%.2f" % t_slider))
        self.vehicle_traj_line = self.vehicle_animation_axis.plot(traveled_path_x, traveled_path_y,
                                                                  linewidth=2.0, color='darkviolet', label='traveled_path')
        self.vehicle_animation_axis.scatter(planner_path_world_x, planner_path_world_y,
                                            s=10, color='green',
                                            label='planner path')
        self.vehicle_animation_axis.legend()
        self.vehicle_line = Functions.draw_car(current_car_position_x, current_car_position_y, current_car_heading_for_BEV,
                                               steer=0 * self.steering_sf_in_BEV,
                                               car_params=self.car_params,
                                               ax=self.vehicle_animation_axis)
        self.vehicle_animation_axis.grid(True)                                               
        xmin = max(min(- 0.5 * self.BEV_range_x, current_car_position_x - self.minimal_margin_x),
                   current_car_position_x + self.minimal_margin_x - self.BEV_range_x)
        xmax = min(max(0.5 * self.BEV_range_x, current_car_position_x + self.minimal_margin_x),
                   current_car_position_x - self.minimal_margin_x + self.BEV_range_x)
        ymin = max(min(- 0.5 * self.BEV_range_y, current_car_position_y - self.minimal_margin_y),
                   current_car_position_y + self.minimal_margin_y - self.BEV_range_y)
        ymax = min(max(0.5 * self.BEV_range_y, current_car_position_y + self.minimal_margin_y),
                   current_car_position_y - self.minimal_margin_y + self.BEV_range_y)
        # Display driving mode information as a colored text box
        if self.driving_mode_data is not None:
            # Get the current driving mode value
            current_driving_mode = self.get_driving_mode_item(system_time, "left")
            if current_driving_mode is not None:
                mode_text = self.mode_descriptions.get(current_driving_mode, f"Mode {current_driving_mode}")
                mode_color = self.mode_colors.get(current_driving_mode, "gray")
                
                # Position the text box in the top-right corner of the vehicle plot
                text_x = xmin + 0.03 * (xmax - xmin)
                text_y = ymax - 0.07 * (ymax - ymin)
                
                # Add text with colored background
                self.vehicle_animation_axis.text(
                    text_x, text_y, 
                    mode_text,
                    bbox=dict(facecolor=mode_color, alpha=0.7, edgecolor='black', boxstyle='round,pad=0.5'),
                    fontsize=12, fontweight='bold', color='white'
                )
        self.vehicle_animation_axis.set_xlim(xmin, xmax)
        self.vehicle_animation_axis.set_ylim(ymin, ymax)
        if self.flip_xy:
            self.vehicle_animation_axis.set_xlabel('y [m]'), self.vehicle_animation_axis.set_ylabel('x [m]')
        else:
            self.vehicle_animation_axis.set_xlabel('x [m]'), self.vehicle_animation_axis.set_ylabel('y [m]')
        
        # Update the lateral and heading error plots if the motion planning debug object has the data
        if hasattr(self.motion_planning_debug_obj, 'lateral_error_relative_to_planner_path') and \
           hasattr(self.motion_planning_debug_obj, 'heading_error_relative_to_planner_path'):
            # Clear the error plots
            self.lateral_error_axis.clear()
            self.heading_error_axis.clear()
            
            # Get the error data
            lateral_error_timestamps = self.motion_planning_debug_obj.lateral_error_relative_to_planner_path['timestamp']
            lateral_error_values = self.motion_planning_debug_obj.lateral_error_relative_to_planner_path['values']
            
            heading_error_timestamps = self.motion_planning_debug_obj.heading_error_relative_to_planner_path['timestamp']
            heading_error_values = self.motion_planning_debug_obj.heading_error_relative_to_planner_path['values']
            
            # Convert timestamps to be relative to t0
            lateral_error_times_zeroed = np.array(lateral_error_timestamps) - self.t0
            heading_error_times_zeroed = np.array(heading_error_timestamps) - self.t0
            
            # Find the indices for data within the visible window
            visible_window_start = max(0, t_slider - 30)  # Show 30 seconds before current time
            
            # Plot the data
            self.lateral_error_axis.plot(lateral_error_times_zeroed, lateral_error_values)
            self.lateral_error_axis.grid(True)
            self.lateral_error_axis.set_ylabel('Lateral Error [m]')
            self.lateral_error_axis.set_xlim(visible_window_start, t_slider + 5)  # Show 5 seconds ahead
            
            self.heading_error_axis.plot(heading_error_times_zeroed, heading_error_values)
            self.heading_error_axis.grid(True)
            self.heading_error_axis.set_ylabel('Heading Error [rad]')
            self.heading_error_axis.set_xlabel('Time [sec]')
            self.heading_error_axis.set_xlim(visible_window_start, t_slider + 5)  # Show 5 seconds ahead
            
            # Mark the current time with a vertical line
            self.lateral_error_axis.axvline(x=t_slider, color='r', linestyle='--')
            self.heading_error_axis.axvline(x=t_slider, color='r', linestyle='--')
        else:
            # Clear the error plots and just show empty grids
            self.lateral_error_axis.clear()
            self.lateral_error_axis.grid(True)
            self.lateral_error_axis.set_ylabel('Lateral Error [m]')
            
            self.heading_error_axis.clear()
            self.heading_error_axis.grid(True)
            self.heading_error_axis.set_ylabel('Heading Error [rad]')
            self.heading_error_axis.set_xlabel('Time [sec]')
        
        # Update driving mode display
        if self.driving_mode_data is not None:
            driving_mode_times = np.array(self.driving_mode_data['timestamps']) - self.t0
            driving_mode_values = self.driving_mode_data['values']
            
            self.driving_mode_axis.clear()
            self.driving_mode_axis.step(driving_mode_times, driving_mode_values, where='post', label='Driving Mode')
            
            # Set y-axis ticks to show only the valid mode values
            valid_modes = sorted(list(set(driving_mode_values)))
            self.driving_mode_axis.set_yticks(valid_modes)
           
            self.driving_mode_axis.set_yticklabels([self.mode_descriptions.get(mode, f"Mode {mode}") for mode in valid_modes])
            
            self.driving_mode_axis.set_ylabel('Driving Mode')
            self.driving_mode_axis.set_xlabel('Time [sec]')
            self.driving_mode_axis.grid(True)
            self.driving_mode_axis.set_xlim(visible_window_start, t_slider + 5)  # Show 5 seconds ahead
            
            # Add a vertical line to mark the current time
            self.driving_mode_axis.axvline(x=t_slider, color='r', linestyle='--')
        
        self.animation_figure.canvas.draw_idle()
    def update_visualization_lateral_control(self):
        t_slider = self.slider.val
        sys_time = t_slider + self.t0 #treated as localization time
        steering_idx_end = Functions.search_time_vector(self.steering_control_time, sys_time, "left")
        steering_idx_start = Functions.search_time_vector(self.steering_control_time, sys_time - self.time_line_visible_frame, "left")
        # steering_idx_end = np.argmin(np.abs(self.steering_control_time_zeroed - t_slider))
        # steering_idx_start = np.argmin(np.abs(self.steering_control_time_zeroed - (t_slider - self.time_line_visible_frame)))
        localization_idx_end = Functions.search_time_vector(self.localization_time, sys_time, "left")
        localization_idx_start = Functions.search_time_vector(self.localization_time,
                                                          sys_time - self.time_line_visible_frame, "left")
        if self.motion_planning_debug_obj is not None:
            if self.motion_planning_debug_obj.MPC_solution is not None:
                MPC_idx_end = Functions.search_time_vector(self.MPC_steering_times, sys_time, "left")
                MPC_idx_start = Functions.search_time_vector(self.MPC_steering_times,
                                                                      sys_time - self.time_line_visible_frame, "left")
        # localization_idx_end = np.argmin(np.abs(self.localization_time_zeroed - t_slider))
        # localization_idx_start = np.argmin(np.abs(self.localization_time_zeroed - (t_slider - self.time_line_visible_frame)))
        path_processing_idx = Functions.search_time_vector(self.path_processing_time, sys_time, "left")
        # path_processing_idx = np.argmin(np.abs(self.path_processing_time_zeroed - t_slider))
        path_processing_key = self.path_processing_time[path_processing_idx]
        self.vehicle_animation_axis.clear()
        if self.desired_traj_x is not None and self.desired_traj_y is not None:
            self.vehicle_animation_axis.plot(self.desired_traj_x, self.desired_traj_y,
                                             color='gray', linewidth=0.5, linestyle='--',
                                             label='GT reference path')
        if self.flip_xy:
            # x,y = y,x swaps variables
            current_car_position_x, current_car_position_y = (self.traveled_path_y[localization_idx_end],
                                                              self.traveled_path_x[localization_idx_end])
            ref_path_raw_x, ref_path_raw_y = (self.path_processing_data[path_processing_key]['input_path_y'],
                                                                      self.path_processing_data[path_processing_key]['input_path_x'])
            ref_path_processed_x, ref_path_processed_y = (self.path_processing_data[path_processing_key]['processed_path_y'],
                                                                                    self.path_processing_data[path_processing_key]['processed_path_x'])
            traveled_path_x, traveled_path_y = (self.traveled_path_y[:localization_idx_end], self.traveled_path_x[:localization_idx_end])
            # np.roll is used for swapping the axes
            aggregated_path = np.roll(self.aggregated_control_path, 1, axis=1)
            current_car_heading_for_BEV = np.pi/2 -  self.vehicle_heading[localization_idx_end]
            if self.motion_planning_debug_obj is not None and \
                self.motion_planning_debug_obj.planner_path_in_world_frame is not None:
                planner_path_in_world_frame = \
                    self.motion_planning_debug_obj.get_planner_path_in_world_frame_item(
                        sys_time, "left"
                        )
                planner_path_world_y, planner_path_world_x = (
                    planner_path_in_world_frame['x'],
                    np.array(planner_path_in_world_frame['y'])
                )
        else:
            current_car_position_x, current_car_position_y = (self.traveled_path_x[localization_idx_end],
                                                              self.traveled_path_y[localization_idx_end])
            ref_path_raw_x, ref_path_raw_y = (self.path_processing_data[path_processing_idx]['input_path_x'],
                                                                      self.path_processing_data[path_processing_idx]['input_path_y'])
            ref_path_processed_x, ref_path_processed_y = (self.path_processing_data[path_processing_idx]['processed_path_x'],
                                                                                  self.path_processing_data[path_processing_idx]['processed_path_y'])
            current_car_heading_for_BEV = self.vehicle_heading[localization_idx_end]
            traveled_path_x, traveled_path_y = (self.traveled_path_x[:localization_idx_end],
                                                self.traveled_path_y[:localization_idx_end])
            aggregated_path = np.array(self.aggregated_control_path)
            if self.motion_planning_debug_obj is not None and \
                self.motion_planning_debug_obj.planner_path_in_world_frame is not None:
                planner_path_in_world_frame = \
                    self.motion_planning_debug_obj.get_planner_path_in_world_frame_item(
                        sys_time, "left"
                        )
                planner_path_world_x, planner_path_world_y = (
                    planner_path_in_world_frame['x'],
                    np.array(planner_path_in_world_frame['y'])
                )
        self.vehicle_animation_axis.set_title('t = ' + str("%.2f" % t_slider))
        self.vehicle_traj_line = self.vehicle_animation_axis.plot(traveled_path_x, traveled_path_y,
                                                                  linewidth=2.0, color='darkviolet', label='traveled_path')
        
        if 'planner_path_world_x' in locals() and \
              'planner_path_world_y' in locals():
                  self.vehicle_animation_axis.scatter(planner_path_world_x, planner_path_world_y,
                                                s=10, color='green',
                                                label='planner path')
        self.vehicle_animation_axis.scatter(ref_path_processed_x, ref_path_processed_y,
                                            s=30, color='blue',
                                            label='ref path in nav frame')
        self.vehicle_animation_axis.scatter(aggregated_path[:steering_idx_end,0],
                                            aggregated_path[:steering_idx_end,1], s=10, marker='x',
                                            color='black', label='aggregated control path')
        self.vehicle_animation_axis.scatter(aggregated_path[steering_idx_end,0], aggregated_path[steering_idx_end,1], s=50, color='red',
                                                marker='x',
                                                label='target point')
        self.vehicle_animation_axis.legend()
        self.vehicle_line = Functions.draw_car(current_car_position_x,current_car_position_y,
                                               current_car_heading_for_BEV,
                                               steer=self.steering[localization_idx_end] * self.steering_sf_in_BEV,
                                               car_params=self.car_params,
                                               ax=self.vehicle_animation_axis)
        self.vehicle_animation_axis.grid(True)
        xmin = max(min(- 0.5 * self.BEV_range_x, current_car_position_x - self.minimal_margin_x),
                   current_car_position_x + self.minimal_margin_x - self.BEV_range_x)
        xmax = min(max(0.5 * self.BEV_range_x, current_car_position_x + self.minimal_margin_x),
                   current_car_position_x - self.minimal_margin_x + self.BEV_range_x)
        ymin = max(min(- 0.5 * self.BEV_range_y, current_car_position_y - self.minimal_margin_y),
                   current_car_position_y + self.minimal_margin_y - self.BEV_range_y)
        ymax = min(max(0.5 * self.BEV_range_y, current_car_position_y + self.minimal_margin_y),
                   current_car_position_y - self.minimal_margin_y + self.BEV_range_y)
                   
        # Display driving mode information as a colored text box
        if self.driving_mode_data is not None:
            # Get the current driving mode value
            current_driving_mode = self.get_driving_mode_item(sys_time, "left")
            if current_driving_mode is not None:
                mode_text = self.mode_descriptions.get(current_driving_mode, f"Mode {current_driving_mode}")
                mode_color = self.mode_colors.get(current_driving_mode, "gray")
                
                # Position the text box in the top-right corner of the vehicle plot
                text_x = xmin + 0.03 * (xmax - xmin)
                text_y = ymax - 0.07 * (ymax - ymin)
                
                # Add text with colored background
                self.vehicle_animation_axis.text(
                    text_x, text_y, 
                    mode_text,
                    bbox=dict(facecolor=mode_color, alpha=0.7, edgecolor='black', boxstyle='round,pad=0.5'),
                    fontsize=12, fontweight='bold', color='white'
                )
                
        self.vehicle_animation_axis.set_xlim(xmin, xmax)
        self.vehicle_animation_axis.set_ylim(ymin, ymax)
        if self.flip_xy:
            self.vehicle_animation_axis.set_xlabel('y [m]'), self.vehicle_animation_axis.set_ylabel('x [m]')
        else:
            self.vehicle_animation_axis.set_xlabel('x [m]'), self.vehicle_animation_axis.set_ylabel('y [m]')
        # lateral error
        self.lateral_error_axis.clear()
        self.lateral_error_axis.plot(self.steering_control_time_zeroed[steering_idx_start:steering_idx_end],
                                     self.lateral_error[steering_idx_start:steering_idx_end])
        self.lateral_error_axis.grid(True)
        self.lateral_error_axis.set_ylabel('$lateral error [m]$')
        self.lateral_error_axis.set_xlim(max(0, t_slider - self.time_line_visible_frame), t_slider)
        # heading axis
        self.heading_control_axis.clear()
        self.heading_control_axis.plot(self.localization_time_zeroed[localization_idx_start:localization_idx_end],
                                       self.vehicle_heading[localization_idx_start:localization_idx_end], label='vehicle_heading')
        self.heading_control_axis.plot(self.steering_control_time_zeroed[steering_idx_start:steering_idx_end],
                                       self.vehicle_heading_from_control[steering_idx_start:steering_idx_end],
                                       label='vehicle_heading_from_controller')
        self.heading_control_axis.plot(self.steering_control_time_zeroed[steering_idx_start:steering_idx_end],
                                       self.heading_reference_raw[steering_idx_start:steering_idx_end], label='heading_reference_raw')
        self.heading_control_axis.plot(self.steering_control_time_zeroed[steering_idx_start:steering_idx_end],
                                       self.heading_reference_filtered[steering_idx_start:steering_idx_end], label='heading_reference_filtered')
        self.heading_control_axis.legend()
        self.heading_control_axis.grid(True), self.heading_control_axis.set_ylabel('[rad]')
        self.heading_control_axis.set_xlim(max(0, t_slider - self.time_line_visible_frame), t_slider)
        # steering axis
        self.steering_axis.clear()
        self.steering_axis.grid(True), self.steering_axis.set_ylabel(
            'steering angle [rad]')
        self.steering_axis.plot(self.localization_time_zeroed[localization_idx_start:localization_idx_end],
                                self.steering[localization_idx_start:localization_idx_end], label="steering")
        self.steering_axis.plot(self.steering_control_time_zeroed[steering_idx_start:steering_idx_end],
                                self.steering_cmd_raw[steering_idx_start:steering_idx_end], label="steering_cmd_raw", linestyle='--',
                                linewidth=3)
        self.steering_axis.plot(self.steering_control_time_zeroed[steering_idx_start:steering_idx_end],
                                self.steering_cmd_filtered[steering_idx_start:steering_idx_end], label="steering_cmd_filtered",
                                linestyle='--',
                                linewidth=3)
        self.steering_axis.plot(self.steering_control_time_zeroed[steering_idx_start:steering_idx_end],
                                self.heading_error[steering_idx_start:steering_idx_end], label="heading_error")
        if self.motion_planning_debug_obj is not None:
            if self.motion_planning_debug_obj.MPC_solution is not None:
                MPC_time_vector_zeroed = np.array(self.MPC_steering_times[MPC_idx_start:MPC_idx_end]) - self.t0
                self.steering_axis.step(MPC_time_vector_zeroed,
                                        - np.array(self.MPC_steering[MPC_idx_start:MPC_idx_end]), where='post', label="MPC  steering")
                                        # minus sign because MPC is solved in ENU where right is negative
        self.steering_axis.set_xlim(max(t_slider - self.time_line_visible_frame, 0), t_slider)
        self.steering_axis.legend()
        ## speed
        self.car_speed_axis.clear()
        self.car_speed_axis.grid(True), self.car_speed_axis.set_xlabel('t [sec]'), self.car_speed_axis.set_ylabel(
            'car speed [mps]')
        
        # Plot car speed on the primary Y axis
        speed_line = self.car_speed_axis.plot(self.localization_time_zeroed[localization_idx_start:localization_idx_end],
                                self.car_speed_mps[localization_idx_start:localization_idx_end], label="car speed", color='blue')
                                
        # Create secondary Y axis for driving mode on the right side
        if self.driving_mode_data is not None:
            # Set up twin axis sharing the same x-axis
            driving_mode_axis = self.car_speed_axis.twinx()
            
            # Get driving mode data
            driving_mode_times = np.array(self.driving_mode_data['timestamps']) - self.t0
            driving_mode_values = self.driving_mode_data['values']
            
            # Plot driving mode on the secondary Y axis
            mode_line = driving_mode_axis.step(driving_mode_times, driving_mode_values, where='post', 
                                              label='Driving Mode', color='red', linestyle='-')
            
            # Set y-axis ticks to show only the valid mode values
            valid_modes = sorted(list(set(driving_mode_values)))
            driving_mode_axis.set_yticks(valid_modes)
            
            # Add labels for the driving modes
            driving_mode_axis.set_yticklabels([self.mode_descriptions.get(mode, f"Mode {mode}") for mode in valid_modes])
            
            driving_mode_axis.set_ylabel('Driving Mode')
            
            # Set the y range to be slightly larger than the data range
            if valid_modes:
                driving_mode_axis.set_ylim(min(valid_modes) - 0.5, max(valid_modes) + 0.5)
            
            # Add a vertical line to mark the current time
            self.car_speed_axis.axvline(x=t_slider, color='r', linestyle='--')
            
            # Combine legends from both axes
            lines = speed_line + mode_line
            labels = [l.get_label() for l in lines]
            self.car_speed_axis.legend(lines, labels, loc='upper left')
        
        # Set time window
        self.car_speed_axis.set_xlim(max(t_slider - self.time_line_visible_frame, 0), t_slider)
                
        ## update figure
        self.animation_figure.canvas.draw_idle()
    def on_key_press(self, event):
        """
        Handle keyboard arrow key events to navigate the slider
        
        Args:
            event: The keyboard event object
        """
        if event.key == 'right' or event.key == 'left':
            # Get current slider value
            current_value = self.slider.val
            
            # Calculate new value based on arrow key
            if event.key == 'right':
                # Move forward in time
                new_value = min(current_value + self.time_shift, self.slider.valmax)
            else:  # Left arrow
                # Move backward in time
                new_value = max(current_value - self.time_shift, self.slider.valmin)
            
            # Update slider value
            self.slider.set_val(new_value)
            
            # The slider callback will be called automatically
    def button_callback(self, event):
        t_slider = self.slider.val
        print("t_slider = " + str(t_slider + self.t0) )
        if self.analyse_lateral_control:
            path_processing_idx = Functions.search_time_vector(self.path_processing_time_zeroed, t_slider, 'left')
            path_processing_key = self.path_processing_time[path_processing_idx]
            sys_time_at_MotionPlanningUpdate_call = path_processing_key
            print('sys time at control call = ' + str(sys_time_at_MotionPlanningUpdate_call))
            path_x, path_y = (self.path_processing_data[path_processing_key]['input_path_x'],
                              self.path_processing_data[path_processing_key]['input_path_y'])
            delay = self.path_processing_data[path_processing_key]["delay"]
            print("delay = " + str(delay))
            path_y = list(np.array(path_y))
            control_path = {"x": path_x, "y":path_y}
            # image_grab_time = sys_time_at_MotionPlanningUpdate_call - delay
            image_grab_time = self.path_processing_data[path_processing_key]["data_timestamp"]
        else:
            path_trajectory_times = np.array(list(self.motion_planning_debug_obj.path_trajectory.keys()))
            idx = Functions.search_time_vector(path_trajectory_times, 
                                               t_slider + self.t0, "left")
            image_grab_time = list(self.motion_planning_debug_obj.path_trajectory.keys())[idx]

        print("image_grab_time = " + str(image_grab_time))
        if self.motion_planning_debug_obj is not None:
            if self.analyse_lateral_control:
                self.motion_planning_debug_obj.plot_ego_frame(image_grab_time, control_path=control_path)
            else:
                self.motion_planning_debug_obj.plot_ego_frame(image_grab_time, control_path=None)
        else:
            Functions.calc_path_features(path_x, path_y, plot_results=True)
        plt.show()
    def close_figure(self):
        if self.plot_initialized:
            plt.close(self.animation_figure)
            print("SteeringControlPlotter.close_figure: figure terminated")
    def __del__(self):
        if self.plot_initialized:
            plt.close(self.animation_figure)
            print("SteeringControlPlotter.__del__: figure terminated")
    def handle_signal(self, signum, frame):
        if self.plot_initialized:
            print("SteeringControlPlotter.handle_signal reached")
            self.close_figure()
            sys.exit(0)
    def analyze_lateral_control_performance(self):
        """
        Analyze the lateral control performance by plotting the lateral and heading errors.
        This simplified version focuses only on basic visualization with absolute timestamps.
        """
        # Create a figure
        fig = plt.figure('Lateral Control Performance Analysis', figsize=(15, 10))
        
        # Create a 3x2 grid layout
        # Main vehicle animation plot takes the left column
        self.vehicle_animation_axis = plt.subplot(3, 2, (1, 5))
        self.vehicle_animation_axis.grid(True)
        self.vehicle_animation_axis.set_aspect('equal')
        
        # Error plots on the right
        self.lateral_error_axis = plt.subplot(3, 2, 2)
        self.lateral_error_axis.grid(True)
        self.lateral_error_axis.set_ylabel('Lateral Error [m]')
        
        self.heading_error_axis = plt.subplot(3, 2, 4, sharex=self.lateral_error_axis)
        self.heading_error_axis.grid(True)
        self.heading_error_axis.set_ylabel('Heading Error [rad]')
        
        self.driving_mode_axis = plt.subplot(3, 2, 6, sharex=self.lateral_error_axis)
        self.driving_mode_axis.grid(True)
        self.driving_mode_axis.set_ylabel('Driving Mode')
        self.driving_mode_axis.set_xlabel('Time [sec]')
        # Set up timestamp handling
        # Use self.t0 as reference time for all plots - ensure this is initialized
        if not hasattr(self, 't0') or self.t0 is None:
            if hasattr(self, 'motion_planning_debug_obj') and self.motion_planning_debug_obj is not None:
                car_pose_timestamps = sorted(list(self.motion_planning_debug_obj.car_pose.keys()))
                if car_pose_timestamps:
                    self.t0 = car_pose_timestamps[0]
                else:
                    print("Warning: No car pose data to determine t0, using 0")
                    self.t0 = 0
            else:
                print("Warning: No motion planning data to determine t0, using 0")
                self.t0 = 0
        
        # Validate required data is available
        if not hasattr(self, 'motion_planning_debug_obj') or self.motion_planning_debug_obj is None:
            print("Error: Motion planning debug data not loaded")
            return
        
        # Get vehicle path data
        car_pose_timestamps = sorted(list(self.motion_planning_debug_obj.car_pose.keys()))
        if not car_pose_timestamps:
            print("Error: No car pose data available")
            return
        
        # Plot vehicle path
        x_points = []
        y_points = []
        for timestamp in car_pose_timestamps:
            pose = self.motion_planning_debug_obj.car_pose[timestamp]
            x_points.append(pose['x'])
            y_points.append(pose['y'])
        
        # Plot the traveled path
        self.vehicle_animation_axis.plot(x_points, y_points, 
                                        linewidth=2.0, 
                                        color='darkviolet', 
                                        label='Traveled Path')
        # If reference points exist, plot them as well
        if hasattr(self.motion_planning_debug_obj, 'aggregated_reference_points') and \
           self.motion_planning_debug_obj.aggregated_reference_points:
            ref_x = self.motion_planning_debug_obj.aggregated_reference_points['x']
            ref_y = self.motion_planning_debug_obj.aggregated_reference_points['y']
            self.vehicle_animation_axis.plot(ref_x, ref_y,
                                            linewidth=1.5,
                                            color='green',
                                            linestyle='--',
                                            label='Reference Path')
        
        # Add markers for start and end
        if x_points and y_points:
            self.vehicle_animation_axis.plot(x_points[0], y_points[0], 'go', markersize=10, label='Start')
            self.vehicle_animation_axis.plot(x_points[-1], y_points[-1], 'ro', markersize=10, label='End')
        
        self.vehicle_animation_axis.legend()
        self.vehicle_animation_axis.set_title('Planner Path VS Car Pose')
        
        # Get lateral and heading error data if available
        if hasattr(self.motion_planning_debug_obj, 'lateral_error_relative_to_planner_path') and \
        hasattr(self.motion_planning_debug_obj, 'heading_error_relative_to_planner_path'):
            
            # Get timestamps and apply t0 offset for consistency
            lateral_timestamps = np.array(self.motion_planning_debug_obj.lateral_error_relative_to_planner_path['timestamp']) - self.t0
            lateral_errors = np.array(self.motion_planning_debug_obj.lateral_error_relative_to_planner_path['values'])
            
            heading_timestamps = np.array(self.motion_planning_debug_obj.heading_error_relative_to_planner_path['timestamp']) - self.t0
            heading_errors = np.array(self.motion_planning_debug_obj.heading_error_relative_to_planner_path['values'])
            
            # Plot lateral errors with proper t0 offset
            self.lateral_error_axis.plot(lateral_timestamps, lateral_errors, 'b-', linewidth=2, label='All Data')
            
            # Calculate RMS error for all data
            lateral_rms_all = np.sqrt(np.mean(lateral_errors**2))
            
            # Filter data for steering or full control modes (1 or 7)
            filtered_lateral_errors = None
            filtered_heading_errors = None
            
            if hasattr(self, 'driving_mode_data') and self.driving_mode_data is not None:
                # Create interpolation function for driving modes
                mode_timestamps = np.array(self.driving_mode_data['timestamps']) - self.t0
                mode_values = np.array(self.driving_mode_data['values'])
                
                if len(mode_timestamps) > 1 and len(mode_values) > 1:
                    # Use nearest interpolation for categorical data
                    mode_interp = interp1d(mode_timestamps, mode_values, 
                                          kind='nearest', bounds_error=False, 
                                          fill_value=(mode_values[0], mode_values[-1]))
                    
                    # Create masks for effective control periods (mode 1 or 7)
                    lateral_effective_mask = np.zeros_like(lateral_errors, dtype=bool)
                    heading_effective_mask = np.zeros_like(heading_errors, dtype=bool)
                    for i, t in enumerate(lateral_timestamps):
                        if t >= np.min(mode_timestamps) and t <= np.max(mode_timestamps):
                            mode = int(mode_interp(t))
                            if mode == 1 or mode == 7:  # Steering or Full control
                                lateral_effective_mask[i] = True
                    for i, t in enumerate(heading_timestamps):
                        if t >= np.min(mode_timestamps) and t <= np.max(mode_timestamps):
                            mode = int(mode_interp(t))
                            if mode == 1 or mode == 7:  # Steering or Full control
                                heading_effective_mask[i] = True
                    
                    # Filter errors based on masks
                    if np.any(lateral_effective_mask):
                        filtered_lateral_errors = lateral_errors[lateral_effective_mask]
                        filtered_lateral_timestamps = lateral_timestamps[lateral_effective_mask]
                        self.lateral_error_axis.plot(filtered_lateral_timestamps, filtered_lateral_errors, 
                                                   'r-', linewidth=2, label='Steering/Full Control Only')
                        self.lateral_error_axis.legend(loc='upper right')
                        
                        # Print min/max values for debugging
                        print(f"\n--- LATERAL ERROR STATISTICS (Control Modes Only) ---")
                        print(f"Min: {np.min(filtered_lateral_errors):.3f} m")
                        print(f"Max: {np.max(filtered_lateral_errors):.3f} m")
                        print(f"Abs Max: {np.max(np.abs(filtered_lateral_errors)):.3f} m")
                        print(f"RMS: {np.sqrt(np.mean(filtered_lateral_errors**2)):.3f} m")
                        print(f"Mean: {np.mean(filtered_lateral_errors):.3f} m")
                        print(f"Std Dev: {np.std(filtered_lateral_errors):.3f} m")
                    
                    if np.any(heading_effective_mask):
                        filtered_heading_errors = heading_errors[heading_effective_mask]
                        filtered_heading_timestamps = heading_timestamps[heading_effective_mask]
                        self.heading_error_axis.plot(filtered_heading_timestamps, filtered_heading_errors, 
                                                   'r-', linewidth=2, label='Steering/Full Control Only')
                        self.heading_error_axis.legend(loc='upper right')
                        
                        # Print min/max values for debugging
                        print(f"\n--- HEADING ERROR STATISTICS (Control Modes Only) ---")
                        print(f"Min: {np.min(filtered_heading_errors):.3f} rad")
                        print(f"Max: {np.max(filtered_heading_errors):.3f} rad")
                        print(f"Abs Max: {np.max(np.abs(filtered_heading_errors)):.3f} rad")
                        print(f"RMS: {np.sqrt(np.mean(filtered_heading_errors**2)):.3f} rad")
                        print(f"Mean: {np.mean(filtered_heading_errors):.3f} rad")
                        print(f"Std Dev: {np.std(filtered_heading_errors):.3f} rad")
            
            # Calculate and display RMS error and max error for filtered data if available
            if filtered_lateral_errors is not None and len(filtered_lateral_errors) > 0:
                lateral_rms = np.sqrt(np.mean(filtered_lateral_errors**2))
                lateral_max = np.max(np.abs(filtered_lateral_errors))
                self.lateral_error_axis.set_title(f'Lateral Error - RMS: {lateral_rms:.3f} m, Max: {lateral_max:.3f} m (engage only)')
                
                # Mark maximum error point
                max_error_idx = np.argmax(np.abs(filtered_lateral_errors))
                self.lateral_error_axis.plot(filtered_lateral_timestamps[max_error_idx], 
                                           filtered_lateral_errors[max_error_idx], 
                                           'ro', markersize=5, 
                                           label=f'Max Error: {lateral_max:.3f} m')
            
            # Plot heading errors with proper t0 offset
            self.heading_error_axis.plot(heading_timestamps, heading_errors, 'b-', linewidth=2, label='All Data')
            
            # Calculate and display RMS error for heading
            heading_rms_all = np.sqrt(np.mean(heading_errors**2))
            
            if filtered_heading_errors is not None and len(filtered_heading_errors) > 0:
                heading_rms = np.sqrt(np.mean(filtered_heading_errors**2))
                heading_max = np.max(np.abs(filtered_heading_errors))
                self.heading_error_axis.set_title(f'Heading Error - RMS: {heading_rms:.3f} rad, Max: {heading_max:.3f} rad (engage only)')
                
                # Mark maximum error point
                max_error_idx = np.argmax(np.abs(filtered_heading_errors))
                self.heading_error_axis.plot(filtered_heading_timestamps[max_error_idx], 
                                           filtered_heading_errors[max_error_idx], 
                                           'ro', markersize=5, 
                                           label=f'Max Error: {heading_max:.3f} rad')
        else:
            self.lateral_error_axis.set_title('Lateral Error - No data available')
            self.heading_error_axis.set_title('Heading Error - No data available')
        
        # Plot driving mode data if available
        if hasattr(self, 'driving_mode_data') and self.driving_mode_data is not None:
            # Apply t0 offset for consistency with other plots
            mode_timestamps = np.array(self.driving_mode_data['timestamps']) - self.t0
            mode_values = np.array(self.driving_mode_data['values'])
            
            # Plot the driving modes as a step function
            self.driving_mode_axis.step(mode_timestamps, mode_values, where='post', linewidth=2)
            
            # Set y-axis ticks and labels
            unique_modes = sorted(list(set(mode_values)))
            self.driving_mode_axis.set_yticks(unique_modes)
            self.driving_mode_axis.set_yticklabels([self.mode_descriptions.get(mode, f"Mode {mode}") for mode in unique_modes])
            
            # Color bands calculation removed to improve performance
            # This section previously added color spans to highlight different driving modes
            
            # Add mode legend to the driving mode plot for clarity
            handles = []
            labels = []
            for mode in unique_modes:
                color = self.mode_colors.get(mode, "gray")
                mode_text = self.mode_descriptions.get(mode, f"Mode {mode}")
                handles.append(plt.Rectangle((0,0), 1, 1, color=color, alpha=0.5))
                labels.append(mode_text)
            self.driving_mode_axis.legend(handles, labels, loc='upper right')
            self.driving_mode_axis.set_title('Driving Mode')
        else:
            self.driving_mode_axis.set_title('Driving Mode - No data available')
        
        # Set the common x range across all plots
        x_min = None
        x_max = None
        
        # Collect all available timestamps
        all_timestamps = []
        if 'lateral_timestamps' in locals() and len(lateral_timestamps) > 0:
            all_timestamps.extend(lateral_timestamps)
        if 'heading_timestamps' in locals() and len(heading_timestamps) > 0:
            all_timestamps.extend(heading_timestamps)
            
        if all_timestamps:
            x_min = min(all_timestamps)
            x_max = max(all_timestamps)
            
            # Set same x range for all plots explicitly
            self.lateral_error_axis.set_xlim(x_min, x_max)
            self.heading_error_axis.set_xlim(x_min, x_max)
            self.driving_mode_axis.set_xlim(x_min, x_max)
        
        # Connect keyboard event handler for arrow key navigation
        fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # Apply tight layout to optimize space usage
        plt.tight_layout()
        
        # Show the plot
        plt.show()
        
        return fig