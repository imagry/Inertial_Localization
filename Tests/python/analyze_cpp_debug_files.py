import copy
from pathlib import Path
import Classes
import pandas as pd
import os
import matplotlib.pyplot as plt
from SteeringControlPlotter import SteeringControlPlotter, ExperimentAnalysisTool, MotionPlanningDebug
import json
import numpy as np
import shutil
import Functions
import folium
import webbrowser
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
# import cartopy.crs as ccrs
# import cartopy.feature as cfeature
# import osmnx as ox
# import requests
# from PIL import Image
# from io import BytesIO
# import math
from datetime import datetime
from tqdm import tqdm

if __name__ == '__main__':
    script_dir = os.path.dirname(os.path.abspath(__file__))
    def plot_localization_temp():
        plt.figure()
        plt.plot(steering_control_debug_data['target_point_x [m]'], steering_control_debug_data['target_point_y [m]'])
        plt.plot(localization_debug_data['x [m]'], localization_debug_data['y [m]'])
    def get_latest_directory(path):
        # Initialize the latest date to the smallest possible date
        latest_date = datetime.min
        latest_dir = None

        # Iterate over all items in the given path
        for item in os.listdir(path):
            # Construct the full path of the item
            item_path = os.path.join(path, item)

            # Check if the item is a directory
            if os.path.isdir(item_path):
                try:
                    # Try to parse the directory name as a date
                    item_date = datetime.strptime(item, '%Y-%m-%d_%H:%M:%S')

                    # If this directory's date is later than the latest date we've seen so far
                    if item_date > latest_date:
                        # Update the latest date and directory
                        latest_date = item_date
                        latest_dir = item
                except ValueError:
                    # If the directory name couldn't be parsed as a date, ignore it
                    pass

        # Return the name of the directory with the latest date
        return latest_dir
    def load_path_processing_data():
        def read_pp_block(cur_idx):
            assert lines[cur_idx] == 'time:\n'
            t_i = float(lines[cur_idx + 1])# latest localization time
            d_i = float(lines[cur_idx + 3])
            data_block = {}
            data_block['input_path_x'] = [float(str_num) for str_num in lines[cur_idx + 5].split(',')[:-1]]
            data_block['input_path_y'] = [float(str_num) for str_num in lines[cur_idx + 7].split(',')[:-1]]
            data_block['processed_path_x'] = [float(str_num) for str_num in lines[cur_idx + 9].split(',')[:-1]]
            data_block['processed_path_y'] = [float(str_num) for str_num in lines[cur_idx + 11].split(',')[:-1]]
            data_block['processed_path_psi'] = [float(str_num) for str_num in lines[cur_idx + 13].split(',')[:-1]]
            data_block['data_timestamp'] = float(lines[cur_idx + 3][:-2])
            data_block['delay'] = t_i - data_block['data_timestamp']
            cur_idx += 14
            return cur_idx, t_i, data_block
        print("loading " + str(path_processing_debug_file_path))
        with open(path_processing_debug_file_path, "r") as f:
            lines = f.readlines()
        pp_data = {}
        n_lines = len(lines)
        line_idx = 0
        with tqdm(total=n_lines) as pbar:
            while line_idx < n_lines:
                if n_lines < line_idx + 14: # 14 lines per block
                    break
                line_idx, t_i, data_block = read_pp_block(line_idx)
                pp_data[t_i] = data_block
                pbar.update(14)
        return pp_data
    def remove_corrupted_lines(path, n_cols):
        file_path, file_name = os.path.split(path)
        file_name_temp = file_name.split('.')[0] + '_TEMP.' + file_name.split('.')[1]
        path_temp = Path(file_path, file_name_temp)
        shutil.copyfile(path, path_temp)
        try:
            with open(path_temp, 'r') as fr:
                lines = fr.readlines()
                with open(path_temp, 'w') as fw:
                    # write header
                    fw.write(lines[0])
                    for line in lines[1:]:
                        cond1 = len(line.split(',')) == n_cols + 1# +1 because includes "\n" as column
                        cond2 = True
                        for word in line.split(',')[:-1]:
                            cond2 = cond2 and check_if_string_is_number(word)
                        if cond1 and cond2:
                            fw.write(line)
                    fw.close()
                fr.close()
            shutil.copyfile(path_temp, path)
        except:
            print("remove_corupt_lines: failed")
        finally:
            os.remove(path_temp)
    def animate_data(start_time=None):
        with open('../../vehicle_config.json', "r") as f:
            vehicle_params = json.loads(f.read())
        # path_processing_times = np.array(list(path_processing_data.keys()))
        # path_processing_time = path_processing_times[-1]
        # path_processing_block = path_processing_data[path_processing_time]
        # plt.figure()
        # plt.plot(path_processing_block['processed_path_x'], path_processing_block['processed_path_y'])
        # plt.show()
        plotter_obj = SteeringControlPlotter(car_params=vehicle_params, flip_xy=True)
        # plotter_obj.BEV_range_x = 7
        # plotter_obj.BEV_range_y = 7
        # time vectors
        path_processing_times = np.array(list(path_processing_data.keys()))
        control_times = np.array(list(steering_control_debug_data['time [sec]']))
        localization_times = np.array(list(localization_debug_data['time [sec]']))
        path_processing_idx = 0
        localization_idx = 0
        if start_time is not None:
            localization_idx = np.argmin(abs(start_time-(localization_times - localization_times[0])))
        control_idx = 0
        n_localization_samples = len(localization_debug_data['time [sec]'])
        # path_processing_times = list(path_processing_data.keys())
        path_processing_time = path_processing_times[path_processing_idx]
        localization_time = localization_times[localization_idx]
        control_time = control_times[control_idx]
        # n_path_processing_samples = len(path_processing_times)
        reference_path = np.zeros([2, 2])
        lateral_error = 0.0
        heading_error = 0.0
        delta = 0.0
        control_target_point=[0,0]
        # iterate on localization data and path processing
        while localization_idx < n_localization_samples-1:
            if path_processing_time <= localization_time and path_processing_idx < len(path_processing_times) - 1:
                path_processing_block = path_processing_data[path_processing_time]
                reference_path = np.vstack(
                    [path_processing_block['processed_path_x'], path_processing_block['processed_path_y']]).T
                path_processing_idx += 1
                path_processing_time = path_processing_times[path_processing_idx]
            elif control_time <= localization_time:
                lateral_error = steering_control_debug_data['lateral_error [m]'][control_idx]
                heading_error = steering_control_debug_data['heading_error [rad]'][control_idx]
                control_target_point = [steering_control_debug_data['target_point_x [m]'][control_idx],
                                        steering_control_debug_data['target_point_y [m]'][control_idx]]
                # delta = steering_control_debug_data['delta [steering]'][control_idx]
                control_idx += 1
                control_time = control_times[control_idx]
            else:
                localization_idx += 1
                localization_time = localization_times[localization_idx]
            if np.mod(localization_idx, 50) == 0:
                # steering,
                # motion_planning_traj_raw, motion_planning_traj_processed, t_i, lateral_error,
                # steering_cmd_raw = 0.0, steering_cmd_filtered = 0.0, heading_error = 0.0, heading_reference = 0.0,
                # control_target_point = None
                plotter_obj.update_plot(current_car_position_x=localization_debug_data['x [m]'][localization_idx],
                                        current_car_position_y=localization_debug_data['y [m]'][localization_idx],
                                        current_car_heading=localization_debug_data['psi [rad]'][localization_idx],
                                        steering=localization_debug_data['steering [rad]'][localization_idx],
                                        motion_planning_traj_raw=np.zeros([2, 2]),
                                        motion_planning_traj_processed=reference_path,
                                        lateral_error=lateral_error,
                                        heading_error=heading_error,
                                        heading_reference=steering_control_debug_data['reference_heading_raw [rad]'][control_idx],
                                        steering_cmd_raw = steering_control_debug_data['steering_wheel_cmd_raw [rad]'][control_idx] /
                                                            vehicle_params['steering_ratio'],
                                        steering_cmd_filtered = steering_control_debug_data['steering_wheel_cmd_filtered [rad]'][control_idx] /
                                                            vehicle_params['steering_ratio'],
                                        t_i=localization_debug_data['time [sec]'][localization_idx] -
                                            localization_debug_data['time [sec]'][0],
                                        control_target_point=copy.copy(control_target_point))# the point is flipped in the plotter so avoid corrupting the data
    def analyze_experiment():
        with open(os.path.join(script_dir,'../../vehicle_config.json'), "r") as f:
            vehicle_params = json.loads(f.read())
        plotter_obj = ExperimentAnalysisTool(car_params=vehicle_params, flip_xy=True, initialize_plot=False)
        # iterate on localization data and path processing
        if script_config["analyze_lateral_control"]:
            plotter_obj.load_lateral_control_files(steering_control_data=steering_control_debug_data,
                                                   localization_data=localization_debug_data,
                                                   path_processing_data=path_processing_data)
        if script_config["analyze_motion_planning_dump_files"]:
            plotter_obj.load_motion_planning(motion_planning_debug_obj)

        if script_config["analyze_lateral_control"]:
            plotter_obj.initialize_plot_lateral_control()
        elif script_config["analyze_motion_planning_dump_files"]:
            plotter_obj.initialize_plot_default()
        else:
            raise "no input data"
        if script_config["analyze_driving_mode"]:
            plotter_obj.parse_driving_mode(file_path=script_config["aidriver_trip_path"])
    def analyze_lateral_control_performance(motion_planning_debug_obj=None):
        if motion_planning_debug_obj is None:
            motion_planning_debug_obj = MotionPlanningDebug(script_config["motion_planning_read_directory"], 
                                                        parse_motion_planning_files=False, 
                                                        parse_planner_path=True)    
        if motion_planning_debug_obj.car_pose is None:
            assert check_if_car_pose_exist()
            if script_config["analyze_online_run"]:
                car_pose_file_container = script_config["aidriver_trip_path"]
            elif script_config["copy_and_delete_motion_planning_files"]:
                car_pose_file_container = script_config["motion_planning_dump_directory"]
            else:
                car_pose_file_container = script_config["motion_planning_read_directory"]
            motion_planning_debug_obj.parse_car_pose_file(car_pose_file_container)
        if motion_planning_debug_obj.planner_path_in_world_frame is None:
            motion_planning_debug_obj.calculate_planner_path_in_world_frame()
        if not (hasattr(motion_planning_debug_obj, 'aggregated_reference_headings') and 
                hasattr(motion_planning_debug_obj, 'aggregated_reference_headings') and 
                hasattr(motion_planning_debug_obj, 'heading_error_relative_to_planner_path') and 
                hasattr(motion_planning_debug_obj, 'lateral_error_relative_to_planner_path')):
            motion_planning_debug_obj.calculate_erros_relative_to_planner_path()
        with open(os.path.join(script_dir,'../../vehicle_config.json'), "r") as f:
            vehicle_params = json.loads(f.read())
        plotter_obj = ExperimentAnalysisTool(car_params=vehicle_params, initialize_plot=False)
        plotter_obj.load_motion_planning(motion_planning_debug_obj)
        plotter_obj.parse_driving_mode(file_path=script_config["aidriver_trip_path"])
        plotter_obj.analyze_lateral_control_performance()
    def analyze_motion_planning_times():
        assert script_config["analyze_motion_planning_dump_files"]
        mp_times = np.array(list(motion_planning_debug_obj.path_trajectory.keys()))
        print("np diffs mean: " + str(np.diff(mp_times).mean()))
        plt.figure("mp times")
        h = plt.subplot(211)
        plt.plot(mp_times)
        plt.subplot(212)
        plt.plot(np.diff(mp_times))
    def analyze_call_times_and_delay():
        path_processing_times = np.array(list(path_processing_data.keys()))
        localization_times = np.array(localization_debug_data['time [sec]'])
        control_times = np.array(steering_control_debug_data['time [sec]'])
        IMU_times = np.array(AHRS_data['time_IMU'])
        IMU_update_OS_times = np.array(AHRS_data['time_OS'])
        t0 = min([path_processing_times[0], localization_times[0], control_times[0]])
        path_processing_times -= t0
        localization_times -= t0
        control_times -= t0
        path_processing_mean_dt = np.diff(path_processing_times).mean()
        localization_mean_dt = np.diff(localization_times).mean()
        control_mean_dt = np.diff(control_times).mean()

        delays = [item["delay"] for item in path_processing_data.values()]

        plt.figure()
        h = plt.subplot(221)
        plt.plot(path_processing_times, label='path_processing_times, dt = ' + str("%.2f" % path_processing_mean_dt))
        plt.plot(localization_times, label='localization_times, dt = ' + str("%.2f" % localization_mean_dt))
        plt.plot(control_times, label='control_times, dt = ' + str("%.2f" % control_mean_dt))
        plt.grid(True), plt.xlabel('samples'),plt.ylabel('[sec]'), plt.title('call times')
        plt.subplot(223, sharex=h)
        plt.plot(np.diff(path_processing_times), label='path_processing_times, dt = ' + str("%.3f" % path_processing_mean_dt))
        plt.plot(np.diff(localization_times), label='localization_times, dt = ' + str("%.3f" % localization_mean_dt))
        plt.plot(np.diff(control_times), label='control_times, dt = ' + str("%.3f" % control_mean_dt))
        plt.grid(True), plt.xlabel('samples'), plt.ylabel('[sec]'), plt.title('call times diffs'), plt.legend()
        plt.subplot(122)
        plt.scatter(path_processing_times, np.ones(path_processing_times.shape[0]),
                    label='path_processing_times, dt = ' + str("%.3f" % path_processing_mean_dt))
        plt.scatter(localization_times, 2 * np.ones(localization_times.shape[0]),
                    label='localization_times, dt = ' + str("%.3f" % localization_mean_dt))
        plt.scatter(control_times, 3 * np.ones(control_times.shape[0]),
                    label='control_times, dt = ' + str("%.3f" % control_mean_dt))
        plt.grid(True), plt.legend(), plt.xlabel('[sec]')
        plt.figure()
        plt.plot(delays)
        plt.title("mean = " + str("%.3f" % np.array(delays).mean()) + " std = " + str("%.3f" % np.array(delays).std()))
        plt.grid(True), plt.ylabel("delays [sec]")

        # IMU update times
        plt.figure('IMU times')
        plt.scatter(IMU_update_OS_times - IMU_update_OS_times[0], IMU_times - IMU_times[0])
        plt.grid(True)
        plt.xlabel('OS times [sec]'), plt.ylabel('IMU times [sec]')
    def plot_steering():
        # Functions.analyze_signal_statistics(np.array(steering_control_debug_data['lateral_error [m]']))
        steering_time = np.array(steering_control_debug_data['time [sec]'])
        t0 = steering_time[0]
        steering_time -= t0
        loc_time = np.array(localization_debug_data['time [sec]'])
        loc_time -= t0
        plt.figure()
        h1 = plt.subplot(311)
        plt.plot(steering_time, steering_control_debug_data['lateral_error [m]'], label='lateral_error [m]')
        plt.ylabel('lateral_error [m]'), plt.grid(True)
        h2 = plt.subplot(312, sharex=h1)
        # plt.plot(loc_time, localization_debug_data['psi [rad]'])
        ref_heading = Functions.continuous_angle(np.array(steering_control_debug_data['reference_heading_raw [rad]']))
        plt.plot(steering_time, ref_heading, label='reference_heading')
        plt.plot(steering_time, steering_control_debug_data['vehicle_heading [rad]'], label='vehicle_heading')
        plt.grid(True), plt.xlabel('time [sec]'), plt.ylabel('psi [rad]'), plt.legend()

        h3 = plt.subplot(313, sharex=h1)
        plt.scatter(loc_time, np.array(localization_debug_data['steering [rad]']),
                 label='steering sensor recorded through API [rad]', s=10, color='red')
        plt.plot(loc_time, np.array(localization_debug_data['steering [rad]']),
                    label='steering sensor recorded through API [rad]', linewidth=3)
        steering_cmd_controller = np.array(steering_control_debug_data['delta_cmd [rad]'])
        plt.plot(steering_time, steering_cmd_controller, label='steering cmd raw [rad]')
        plt.plot(steering_time,
                 np.array(steering_control_debug_data['steering_wheel_cmd_filtered [rad]']) / vehicle_params['steering_ratio'],
                 label='steering cmd filtered [rad]')
        plt.plot(steering_time, steering_control_debug_data['heading_error [rad]'], linewidth=2, linestyle='--',label='heading error [rad]')
        if script_config["compare_results_with_offline_trip_data"]:
            steering_file_path = Path(aidriver_trip_path / 'steering.csv')
            steering_data = pd.read_csv(steering_file_path, index_col=False)
            t_start = np.array(localization_debug_data['time [sec]'])[0]
            t_end = np.array(localization_debug_data['time [sec]'])[-1]
            start_index = np.argmin(abs(steering_data['time_stamp'] - t_start))
            end_index = np.argmin(abs(steering_data['time_stamp'] - t_end))
            steering_time = np.array(steering_data['time_stamp'])[start_index:end_index]
            steering_values = np.array(steering_data['data_value'])[start_index:end_index]
            plt.plot(steering_time - t0 , steering_values / float(vehicle_params['steering_ratio']) * np.pi / 180,
                     label='(steering read from trip csv file) / steering_ratio', linestyle='--')
        # plt.plot(trip_obj.common_time, trip_obj.steering_interp / float(vehicle_params['steering_ratio']) * np.pi / 180,
        #          label='(steering read from trip csv file) / steering_ratio')
        plt.grid(True), plt.legend(), plt.xlabel('time [sec]'), plt.ylabel('steering angle ($\delta$) [rad]')

        # plt.figure()
        # plt.plot(np.diff(loc_time))
    def find_localization_at_time(ti):
        localization_time = np.array(localization_debug_data['time [sec]'])
        index = np.searchsorted(localization_time,  ti, side='right')
        return index
    def debug_steering_cmd_calculation():
        path_processing_times = list(path_processing_data.keys())
        for t in path_processing_times:
        # t = path_processing_times[0]
            path_x,  path_y = (path_processing_data[t]['processed_path_x'],
                              path_processing_data[t]['processed_path_y'])
            path_psi = path_processing_data[t]['processed_path_psi']
            loc_idx = find_localization_at_time(t)
            x_v = np.array(localization_debug_data['x [m]'])[loc_idx]
            y_v = np.array(localization_debug_data['y [m]'])[loc_idx]
            psi_v = np.array(localization_debug_data['psi [rad]'])[loc_idx]
            vel = np.array(localization_debug_data['vel [mps]'])[loc_idx]
            controller = Classes.StanleyController(Ks=control_config["stanley_gain"],
                                                   desired_traj_x=path_x,
                                                   desired_traj_y=path_y,
                                                   desired_traj_psi=path_psi)
            vehicle_state = [x_v, y_v, psi_v, vel]
            plt.figure()
            plt.plot(path_x, path_y)
            Functions.draw_car(x_v, y_v, psi_v, steer=0, car_params=vehicle_params)
            plt.axis('equal'), plt.grid(True)
            plt.show()
        # controller.calc_steering_command(vehicle_state)
        print("delta = " + str(controller.delta))
    def compare_carpose_to_gps():
        def parse_car_pose_file():
            car_pose_file_path = car_pose_path()
            with open(car_pose_file_path, "r") as f:
                lines = f.readlines()
            data = {"x":[], "y":[], "psi":[], "timestamp":[]}
            for line in lines[1:]:
                splited = line.split(',')
                data["timestamp"].append(float(splited[0]))
                data["x"].append(float(splited[1]))
                data["y"].append(- float(splited[2]))
                data["psi"].append( - float(splited[3]) * np.pi / 180)
            return data
        def calc_pos_err_on_time_interval(time_interval, plot_res=False):
            dt = np.diff(trip_obj.common_time).mean()
            window_size = round(time_interval / dt)
            n = len(trip_obj.common_time)
            dP_on_time_inerval = P_est_interp[window_size:n] - P_est_interp[:n - window_size]
            N_on_time_inerval = (trip_obj.N[window_size:n] - trip_obj.N[:n - window_size]).reshape([n - window_size, 1])
            E_on_time_inerval = (trip_obj.E[window_size:n] - trip_obj.E[:n - window_size]).reshape([n - window_size, 1])
            dP_on_time_inerval_GPS = np.hstack([N_on_time_inerval, E_on_time_inerval])
            dP_error = dP_on_time_inerval - dP_on_time_inerval_GPS
            dp_error_norm = np.linalg.norm(dP_error, axis=1)
            if plot_res:
                signal_std = np.std(dp_error_norm)
                signal_mean = np.mean(dp_error_norm)
                fig, ax = plt.subplots()
                ax.plot(dp_error_norm)
                ax.axhline(y=signal_mean, color='red', linestyle='--')
                ax.axhline(y=signal_mean + signal_std, color='black', linestyle='--')
                ax.axhline(y=signal_mean - signal_std, color='black', linestyle='--')
                title = 'short time localization error, mean = ' + \
                        str("%.2f" % dp_error_norm.mean()) + ', STD = ' + \
                        str("%.2f" % dp_error_norm.std())
                # ax.set_title("mean = " + str("%.2f" % signal_mean) + ", std = " + str("%.2f" % signal_std))
                ax.set_title(title)
                plt.grid(True), plt.ylabel('[m]'), plt.title(title)
            return dp_error_norm.mean()
        carpose = parse_car_pose_file()
        carpose_x = np.array(carpose['x'])
        carpose_y = np.array(carpose['y'])
        carpose_x -= carpose_x[0]
        carpose_y -= carpose_y[0]
        gt = np.vstack([trip_obj.N, trip_obj.E])
        est = np.vstack(([carpose_x, carpose_y]))
        carpose_rotated = rotate_traj_to_match_initial_heading(gt, est, plot=False)
        plt.figure()
        """the figures y-axis is treated as "forward" and x-axis is "right" this would plot a NED configurations"""
        plt.plot(trip_obj.E, trip_obj.N, label='gps')
        plt.plot(carpose_rotated[1, :], carpose_rotated[0, :], label='localization')
        plt.grid(True), plt.legend(), plt.ylabel('N [m]'), plt.xlabel('E [m]')#, plt.axis('equal')
        # calculate estimated position error in sampled time windows
        gps_time = trip_obj.common_time - trip_obj.common_time[0]
        est_time = np.array(carpose["timestamp"]) - carpose["timestamp"][0]
        P_est_x_interp = np.interp(gps_time, est_time, carpose_rotated[0, :])
        P_est_y_interp = np.interp(gps_time, est_time, carpose_rotated[1, :])
        P_est_interp = np.vstack([P_est_x_interp, P_est_y_interp]).T
        time_intervals = list(np.arange(0.1, 1, step=0.1))
        err = []
        for time_interval in time_intervals:
            err.append(calc_pos_err_on_time_interval(time_interval, plot_res=True))
        err = np.array(err)
        time_intervals = np.array(time_intervals)
        # Compute the slopes between each consecutive pair of points
        slopes = np.diff(err) / np.diff(time_intervals)
        # Calculate the mean slope
        mean_slope = np.mean(slopes)
        plt.figure()
        plt.plot(time_intervals, err)
        plt.grid(True)
        plt.xlabel('time interval [sec]'), plt.ylabel('position estimation error')
        plt.title("Mean Slope:" + str("%.2f" % mean_slope))
    def plot_localization(time_range=None):
        plt.figure()
        # the figures y-axis is treated as "forward" and x-axis is "right" this would plot a NED configurations
        if script_config["compare_results_with_offline_trip_data"]:
            plt.plot(trip_obj.E, trip_obj.N, label='gps')
        x = np.array(localization_debug_data['x [m]'])
        # x -= x[0]
        y = np.array(localization_debug_data['y [m]'])
        # y -= y[0]
        # y = handle_array_of_strings(y)

        # t_ref = trip_obj.common_time
        # ref = np.column_stack((np.asarray(trip_obj.E), np.asarray(trip_obj.N)))
        # t_est = np.asarray(localization_debug_data['time [sec]'] - localization_debug_data['time [sec]'][0])
        # est = np.column_stack((np.asarray(x), np.asarray(y)))
        if time_range is not None:
            start_idx_in_loc_time = np.argmin(abs(localization_debug_data['time [sec]'] - localization_debug_data['time [sec]'][0] - time_range[0]))
            stop_idx_in_loc_time = np.argmin(abs(localization_debug_data['time [sec]'] - localization_debug_data['time [sec]'][0] - time_range[1]))
            x = x[start_idx_in_loc_time:stop_idx_in_loc_time]
            y = y[start_idx_in_loc_time:stop_idx_in_loc_time]

            start_idx_in_steering_time = np.argmin(
                abs(steering_control_debug_data['time [sec]'] - steering_control_debug_data['time [sec]'][0] - time_range[0]))
            stop_idx_in_steering_time = np.argmin(
                abs(steering_control_debug_data['time [sec]'] - steering_control_debug_data['time [sec]'][0] - time_range[1]))
            target_points_x = steering_control_debug_data['target_point_x [m]'][start_idx_in_steering_time:stop_idx_in_steering_time]
            target_points_y = steering_control_debug_data['target_point_y [m]'][
                              start_idx_in_steering_time:stop_idx_in_steering_time]
        else:
            target_points_x = steering_control_debug_data['target_point_x [m]']
            target_points_y = steering_control_debug_data['target_point_y [m]']
        plt.plot(y, x, label='localization')
        plt.scatter(target_points_y, target_points_x, color='orange', marker='x',
                    s=10, label='control target points')
        plt.plot(target_points_y,
                    target_points_x, label='control target points')
        plt.grid(True), plt.legend(), plt.ylabel('N [m]'), plt.xlabel('E [m]')#, plt.axis('equal')
        ###
        plt.figure()
        if time_range is not None:
            plt.plot(localization_debug_data['time [sec]'][start_idx_in_loc_time:stop_idx_in_loc_time] - 
                     localization_debug_data['time [sec]'][0],
                     localization_debug_data['psi [rad]'][start_idx_in_loc_time:stop_idx_in_loc_time])
        else:
            plt.plot(localization_debug_data['time [sec]'] - localization_debug_data['time [sec]'][0], 
                     localization_debug_data['psi [rad]'])
        plt.title("$\psi$"), plt.grid(True), plt.xlabel("time [sec]")
    def analyze_imu_dt():
        assert script_config["compare_results_with_offline_trip_data"]
        imu_data = pd.read_csv(Path(script_config["aidriver_trip_path"]) / "imu.csv")
        dt = np.diff(imu_data.time_stamp)
        plt.figure()
        plt.scatter(range(len(dt)), dt)
    def plot_AHRS():
        if script_config["compare_results_with_offline_trip_data"]:
            remove_corrupted_lines(AHRS_debug_file_path, 8)
            AHRS_data = pd.read_csv(AHRS_debug_file_path, index_col=None)
            phi_hat = Functions.continuous_angle(np.array(AHRS_data.phi_hat))
            theta_hat = Functions.continuous_angle(np.array(AHRS_data.theta_hat))
            psi_hat = Functions.continuous_angle(np.array(AHRS_data.psi_hat - AHRS_data.psi_hat[0]))
            t = np.array(AHRS_data.time_IMU)


            data_reference = pd.read_csv(Path(script_config["aidriver_trip_path"]) / "imu.csv", index_col=None)
            phi = Functions.continuous_angle(np.array(data_reference.roll * np.pi / 180))
            theta = Functions.continuous_angle(np.array(data_reference.pitch * np.pi / 180))
            psi = Functions.continuous_angle(
                np.array((data_reference.yaw - data_reference.yaw[0]) * np.pi / 180))
            t_ref = np.array(data_reference.time_stamp)

            Functions.PlotEulerAngles(phi,phi_hat, theta, theta_hat, psi, psi_hat, t_ref, t)
        else:
            remove_corrupted_lines(AHRS_debug_file_path, 7)
            AHRS_data = pd.read_csv(AHRS_debug_file_path, index_col=None)
            phi_hat = Functions.continuous_angle(np.array(AHRS_data.phi_hat))
            theta_hat = Functions.continuous_angle(np.array(AHRS_data.theta_hat))
            psi_hat = Functions.continuous_angle(np.array(AHRS_data.psi_hat - AHRS_data.psi_hat[0]))
            phiIMU = Functions.continuous_angle(np.array(AHRS_data.phiIMU))
            thetaIMU = Functions.continuous_angle(np.array(AHRS_data.thetaIMU))
            psiIMU = Functions.continuous_angle(np.array(AHRS_data.psiIMU - AHRS_data.psiIMU[0]))
            t = np.array(AHRS_data.time_IMU)



            Functions.PlotEulerAngles(phiIMU, phi_hat, thetaIMU, theta_hat, psiIMU, psi_hat, t, t)
    def handle_array_of_strings(arr):
        res = []
        for (i,element) in enumerate(arr):
            res.append(float(element))
        return res
    def check_if_string_is_number(word: str):
        word = '12.21'
        splits = word.split('.')
        cond1 = 0 < len(splits) and len(splits) <= 2
        cond2 = True
        for split in splits:
            cond2 = cond2 and split.isnumeric()
        return cond1 and cond2
    def analyze_traj_processing():
        path_processing_times = list(path_processing_data.keys())
        # plt.figure()
        for t in path_processing_times:
            # path_x, path_y = (path_processing_data[t]['input_path_x'], path_processing_data[t]['input_path_y'])
            path_x, path_y = (path_processing_data[t]['processed_path_x'], path_processing_data[t]['processed_path_y'])
            # plt.scatter(path_x, path_y)
            # plt.grid(True), plt.xlabel('y [m]'), plt.ylabel('x [m]')
            # plt.axis('equal')
            Functions.calc_path_features(path_x, path_y, plot_results=True)
            plt.show()
    def trip_view_on_map(trail_coordinates, mapLabel='map'):
        # ""Trail coordinates - Nx2 array of [lat,long]
        m_mean = np.mean(trail_coordinates, axis=0)
        m = folium.Map(location=[m_mean[0], m_mean[1]], zoom_start=50)
        folium.PolyLine(trail_coordinates, tooltip=None).add_to(m)
        downsamp_trail_coordinates = trail_coordinates[::100, :]
        for index in range(downsamp_trail_coordinates.shape[0]):
            point = downsamp_trail_coordinates[index, :]
            folium.CircleMarker(point, radius=1, color="red").add_to(m)
        # folium.PolyLine(downsamp_trail_coordinates, tooltip=None, color="red", fill="none", **{"stroke": True, "stroke-dasharray":4}).add_to(m)
        # folium.Marker(m_mean, popup=None).add_to(m)
        # save the map as an HTML file

        #  Add clear marking for beginning and end points of the trip
        path_start = trail_coordinates[0, :]
        path_end = trail_coordinates[-1, :]
        folium.CircleMarker(path_start, radius=3, color="green").add_to(m)
        folium.CircleMarker(path_end, radius=3, color="black").add_to(m)

        os.makedirs('Results/', exist_ok=True)
        mapName = 'Results/' + mapLabel + '.html'
        m.save(mapName)
        return m, mapName
    def plot_trip_map(X_gps_lng_lat, trip_name):
        trip_latlong = np.fliplr(X_gps_lng_lat[0:2, :].T)
        map_name = 'map_' + trip_name
        m, map_nampe = trip_view_on_map(trip_latlong, map_name)
        webbrowser.open_new_tab(map_nampe)
    def plot_control_target_points():
        plt.figure()
        plt.scatter(steering_control_debug_data['target_point_x [m]'], steering_control_debug_data['target_point_y [m]'],
                    color='black', marker='x')
        plt.grid(True)
    def create_timestamped_directory(container_dir):
        # Get the current date and time
        now = datetime.now()
        # Format the date and time as a string
        dir_name = now.strftime("%Y-%m-%d_%H:%M:%S")
        # Create the directory
        os.makedirs(Path(container_dir) / dir_name)
        return Path(container_dir) / dir_name
    def achieve_user_permission(path):
        path = Path(path)
        # achieve user permission
        cmd = "sudo chown -R $USER " + str(os.path.abspath(path))
        returned_value = os.system(cmd)  # returns the exit code in unix
        print('achieved user permission to ' + str(path))
        print('returned value:', returned_value)
    def check_if_car_pose_exist():
        if script_config["analyze_online_run"]:
            car_pose_file_container = script_config["aidriver_trip_path"]
        elif script_config["copy_and_delete_motion_planning_files"]:
            car_pose_file_container = script_config["motion_planning_dump_directory"]
        else:
            car_pose_file_container = script_config["motion_planning_read_directory"]
        res = False
        for file in os.listdir(car_pose_file_container):
            if "car_pose" in file:
                res = True
        return res
    def find_car_pose_file():
        "return the file name"
        if script_config["analyze_online_run"]:
            car_pose_file_container = script_config["aidriver_trip_path"]
        elif script_config["copy_and_delete_motion_planning_files"]:
            car_pose_file_container = script_config["motion_planning_dump_directory"]
        else:
            car_pose_file_container = script_config["motion_planning_read_directory"]
        for file in os.listdir(car_pose_file_container):
            if "car_pose" in file:
                res = file
        return res
    def car_pose_path():
        "return container"
        if script_config["analyze_online_run"]:
            car_pose_file_container = script_config["aidriver_trip_path"]
        elif script_config["copy_and_delete_motion_planning_files"]:
            car_pose_file_container = script_config["motion_planning_dump_directory"]
        else:
            car_pose_file_container = script_config["motion_planning_read_directory"]
        for file in os.listdir(car_pose_file_container):
            if "car_pose" in file:
                res = file
        return Path(car_pose_file_container) / res
    def copy_MP_files_to_data_folder(path_files_container, destination_folder):
        if not check_if_car_pose_exist():
            raise "car pose file not found"
        path_files_container = Path(path_files_container)
        destination_folder = Path(destination_folder)
        shutil.copyfile(path_files_container / "path_trajectory.csv", destination_folder / "path_trajectory.csv")
        print("path_trajectory.csv copied from" + str(path_files_container) + " to " + str(destination_folder))
        shutil.copyfile(path_files_container / "path_adjustment.csv", destination_folder / "path_adjustment.csv")
        print("path_adjustment.csv copied from" + str(path_files_container) + " to " + str(destination_folder))
        shutil.copyfile(path_files_container / "path_extraction.csv", destination_folder / "path_extraction.csv")
        print("path_extraction.csv copied from" + str(path_files_container) + " to " + str(destination_folder))
        shutil.copyfile(path_files_container / "planner_path.csv", destination_folder / "planner_path.csv")
        print("planner_path.csv copied from" + str(path_files_container) + " to " + str(destination_folder))
        car_pose_file_name = find_car_pose_file()
        shutil.copyfile(path_files_container / car_pose_file_name, destination_folder / car_pose_file_name)
        print(car_pose_file_name + " copied from" + str(path_files_container) + " to " + str(destination_folder))
    def check_if_MP_files_exist(path_files_container):
        if script_config["analyze_online_run"]:
            car_pose_file_container = script_config["aidriver_trip_path"]
        else:
            car_pose_file_container = path_files_container
        def check_if_car_pose_exist():
            res = False
            for file in os.listdir(car_pose_file_container):
                if "car_pose" in file:
                    res = True
            return res
        path_files_container = Path(path_files_container)
        file_1 = path_files_container / "path_trajectory.csv"
        file_2 = path_files_container / "path_adjustment.csv"
        file_3 = path_files_container / "path_extraction.csv"
        file_4 = path_files_container / "planner_path.csv"
        car_pose_condition = (not script_config["analyze_carpose"]) or check_if_car_pose_exist()
        return file_1.is_file() and file_2.is_file() and file_3.is_file() and file_4.is_file() and car_pose_condition
    def delete_MP_files(path_files_container):
        if script_config["analyze_online_run"]:
            car_pose_file_container = script_config["aidriver_trip_path"]
        else:
            car_pose_file_container = path_files_container
        def find_car_pose_files():
            car_pose_files = []
            for file in os.listdir(path_files_container):
                if "car_pose" in file:
                    car_pose_files.append(file)
            return car_pose_files
        path_files_container = Path(path_files_container)
        os.remove(path_files_container / "path_trajectory.csv")
        print("path_trajectory.csv deleted from" + str(path_files_container))
        os.remove(path_files_container / "path_adjustment.csv")
        print("path_adjustment.csv deleted from" + str(path_files_container))
        os.remove(path_files_container / "path_extraction.csv")
        print("path_extraction.csv deleted from" + str(path_files_container))
        os.remove(path_files_container / "planner_path.csv")
        print("planner_path.csv deleted from" + str(path_files_container))
        if not script_config["analyze_online_run"]:
            for file in find_car_pose_files():
                os.remove(path_files_container / file)
                print(file + " deleted from" + str(path_files_container))
    def copy_space_net_files():
        # # Ensure the target directory exists
        # os.makedirs(script_config["space_net_read_directory"], exist_ok=True)
        # # Iterate over all the items in the source directory
        # for item in os.listdir(script_config["space_net_dump_directory"]):
        #     source_item = os.path.join(script_config["space_net_dump_directory"], item)
        #     target_item = os.path.join(script_config["space_net_read_directory"], item)
        #     # If the item is a directory, use copytree
        #     if os.path.isdir(source_item):
        #         shutil.copytree(source_item, target_item, dirs_exist_ok=True)
        #     else:
        #         # If the item is a file, use copy2
        #         shutil.copy2(source_item, target_item)
        shutil.copytree(script_config["space_net_dump_directory"], script_config["space_net_read_directory"], dirs_exist_ok=True)
        print("space net files copied from " + script_config["space_net_dump_directory"] + " to " + script_config["space_net_read_directory"] )
    def delete_space_net_files(container):
        # Remove all files and subdirectories in the directory
        for filename in os.listdir(container):
            file_path = os.path.join(container, filename)
            if os.path.isfile(file_path):
                os.unlink(file_path)  # remove file or symlink
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)  # remove directory
        print("space net files deleted from " + container)
    def create_space_net_read_directory():
        os.makedirs(script_config["space_net_read_directory"])
        print("created "  + script_config["space_net_read_directory"])
    def plot_map_desired_boundaries():
        def calculate_bounds(center_lat, center_lon, zoom, map_width, map_height):
            # Calculate the scale factor
            scale = 2 ** zoom

            # Calculate the number of pixels per degree
            pixels_per_degree = scale * 256.0 / 360.0

            # Calculate the size of the map in degrees
            map_width_degrees = map_width / pixels_per_degree
            map_height_degrees = map_height / pixels_per_degree

            # Calculate the coordinates of the corners
            min_lon = center_lon - map_width_degrees / 2.0
            max_lon = center_lon + map_width_degrees / 2.0
            min_lat = center_lat - map_height_degrees / 2.0
            max_lat = center_lat + map_height_degrees / 2.0

            return min_lat, min_lon, max_lat, max_lon
        # Specify the coordinates of the desired location
        latitude = 32.781118
        longitude = 34.967301
        # Specify the desired zoom level
        space_net_image_height = 420
        space_net_image_width = 336
        zoom = 19
        # Specify your Google Maps API key

        api_key = 'AIzaSyCAY2mp9huBY1Pc38yf-GlKWvHaaB0Vkks'
        # Specify the URL of the Google Maps Static API

        width = 640
        height = 640

        url = f"https://maps.googleapis.com/maps/api/staticmap?center={latitude},{longitude}&zoom={zoom}&size={width}x{height}&key={api_key}"
        min_lat, min_lon, max_lat, max_lon = calculate_bounds(center_lat=latitude, center_lon=longitude, zoom=zoom, map_width=width, map_height=height)
        print("min_lat, min_lon, max_lat, max_lon = " + str("%.5f" % min_lat) + ", " + str("%.5f" % min_lon) + ", " + str("%.5f" % max_lat)  + ", " + str("%.5f" % max_lon))
        # Retrieve the image from the Mapbox Static Images API
        if True:
            response = requests.get(url)
            # Save the image to a file
            with open("map.png", "wb") as f:
                f.write(response.content)
        image = Image.open("map.png")
        # Convert the image to a numpy array
        image_array = np.array(image)
        plt.figure()
        plt.imshow(image)
        # image = Image.open(BytesIO(response.content))
        # image.show()
    def rotate_traj_to_match_initial_heading(gt, est, plot=False):
        def interpolate_gt_to_est(est, gt):
            """
            assume trajectories are numpy arrays with shape 2 X N
            1. segment gt to match trajectory length
            2. interpolate
            """
            s_est, _, _ = Functions.calc_path_features(est[0, :], est[1, :])
            s_gt, _, _ = Functions.calc_path_features(gt[0, :], gt[1, :])
            gt_last_idx = Functions.search_time_vector(s_gt, s_est[-1],"left")
            gt, s_gt = (gt[:, :gt_last_idx], s_gt[:gt_last_idx])
            gt_interp_x = np.interp(x=s_est, xp=s_gt, fp=gt[0, :])
            gt_interp_y = np.interp(x=s_est, xp=s_gt, fp=gt[1, :])
            return np.vstack([gt_interp_x, gt_interp_y])
        def traj_error(est, gt, weights=None):
            err = np.linalg.norm(est - gt, axis=0)
            if weights is not None:
                assert weights.shape == err.shape
                err = (err * weights).sum() / weights.sum()
            else:
                err = err.mean()

            return err
        def transform_r(est, gt, weights=None):
            def minimization_function(angle, traj_gt):
                gt_trans = rotate_trajectory(traj_gt, angle[0])
                return traj_error(est=est, gt=gt_trans, weights=weights)

            x0 = np.array([0.0])  # angle
            res = minimize(minimization_function, x0, method='nelder-mead', args=(gt,),
                           options={'xatol': 1e-8, 'disp': False})

            angle = res.x[0]
            gt_trans = rotate_trajectory(gt, angle)
            err = traj_error(est=est, gt=gt_trans, weights=weights)

            return gt_trans, err, angle
        def rotate_trajectory(pts, angle):
            assert pts.shape[0] == 2
            rot = Rotation.from_euler('ZYX', [angle, 0, 0], degrees=False).as_matrix()  # 3 X 3
            rot = rot[0:2, 0:2]
            rot_traj = (rot @ pts)  # (2X2 @ 2Xn).rot = (2 X n).rot = n X 2
            return rot_traj
        # gt = np.column_stack((np.asarray(df['gt_x']), np.asarray(df['gt_y'])))
        # est = np.column_stack((np.asarray(df['est_x']), np.asarray(df['est_y'])))
        gt_interp = interpolate_gt_to_est(est, gt)
        weights = (gt_interp.shape[1] - np.arange(gt_interp.shape[1])) / gt_interp.shape[1]
        power = 4
        weights = weights ** power
        err = traj_error(est=est, gt=gt_interp, weights=weights)

        gt_r, err_r, angle_r = transform_r(est, gt_interp, weights=weights)

        out_dict = {'err_r': err_r, 'angle_r': angle_r}
        est_r = rotate_trajectory(est, - angle_r)
        if plot:
            plt.figure()
            plt.plot(est[0, :], est[1, :], label='est')
            plt.plot(gt[0, :], gt[1, :], '--', label=f'gt {err:.1f}')
            plt.plot(gt_r[0, :], gt_r[1, :], '--', label=f'gt R {err_r:.1f}')
            plt.plot([0], [0], '*k')
            plt.legend()
            plt.show()
        return est_r
    script_config = {"analyze_online_run":False,
                     "use_latest_control_folder":True,
                     "compare_results_with_offline_trip_data":False,
                     "analyze_motion_planning_dump_files":True,
                     "analyze_MPC_solution": True,
                     "analyze_driving_mode": True,
                     "analyze_carpose": True,
                     "motion_planning_dump_directory": "/opt/imagry/aidriver_new/logs/", 
                     "motion_planning_read_directory": "/opt/imagry/aidriver_new/logs/",# /opt/imagry/aidriver_new/logs/, /media/eranvertz/DATA_DISK_A1/aidriver_data/trips/2025-07-03T11_19_37/logs/motion_planning/
                    #  "motion_planning_read_directory": "/media/eranvertz/DATA_DISK_A1/aidriver_data/trips/2025-07-03T11_19_37/logs/motion_planning/",# /opt/imagry/aidriver_new/logs/, /media/eranvertz/DATA_DISK_A1/aidriver_data/trips/2025-07-03T11_19_37/logs/motion_planning/
                     "copy_and_delete_motion_planning_files": False,
                     "use_latest_motion_planning_read_folder": False,
                     "analyze_lateral_control":True,
                     "analyze_AHRS_file": False,
                     "analyze_space_net": False,
                     "analyze_road_layer": False,
                     "analyze_lane_dividers": False,
                     "analyze_vehicles": False,
                    #  "aidriver_trip_path":"/media/eranvertz/DATA_DISK_A1/aidriver_data/trips/2025-07-03T11_19_37/",
                     "aidriver_trip_path": "/home/eranvertz/imagry/trips/NAHARIA/2025-05-21T11_52_50/",
                     "control_module_log_path": "/media/eranvertz/DATA_DISK_A1/aidriver_data/trips/2025-07-03T11_19_37/2025-07-03_11:18:30/",
                     "space_net_dump_directory": "/opt/imagry/aidriver_new/tmp/",# by default '/opt/imagry/aidriver_new/tmp/'
                     "space_net_read_directory":  "/opt/imagry/aidriver_new/tmp/", # by default '/opt/imagry/aidriver_new/tmp/'
                     "copy_and_delete_space_net_files": False
                     }
    if script_config["compare_results_with_offline_trip_data"]:
        aidriver_trip_path = Path(script_config["aidriver_trip_path"])
        print('recorded trip files from ' + str(aidriver_trip_path))
        trip_obj = Classes.Trip(aidriver_trip_path)
        X_gps_lng_lat = np.vstack([trip_obj.lon_interp, trip_obj.lat_interp])
        trip_name = Path(script_config["aidriver_trip_path"]).name
    if script_config["analyze_lateral_control"]:
        with open(Path(os.path.join(script_dir, '../../vehicle_config.json')), "r") as f:
            vehicle_params = json.loads(f.read())
        with open(Path(os.path.join(script_dir,'../../control_config.json')), "r") as f:
            control_config = json.loads(f.read())
        if script_config["use_latest_control_folder"]:
            temp_results_folder = Path(os.path.join(script_dir,'../../data/temp_results'))
            debug_folder = Path(get_latest_directory(temp_results_folder))
            data_folder = temp_results_folder / debug_folder
        else:
            data_folder = Path(script_config["control_module_log_path"])
        print('lateral control debug folder: ' + str(data_folder))
        achieve_user_permission(data_folder)
        # # achieve user permission
        # cmd = "sudo chown -R $USER " + str(os.path.abspath(data_folder))
        # returned_value = os.system(cmd)  # returns the exit code in unix
        # print('returned value:', returned_value)
        #####
        localization_debug_file_path = data_folder / 'debug_localization.csv'
        steering_control_file_path = data_folder / 'debug_steering_control.csv'
        path_processing_debug_file_path = data_folder / 'debug_path_processing.csv'
        if script_config["analyze_AHRS_file"]:
            AHRS_debug_file_path = data_folder / 'debug_AHRS.csv'
            remove_corrupted_lines(AHRS_debug_file_path, 8)
            AHRS_data = pd.read_csv(AHRS_debug_file_path, index_col=None)
        path_processing_data = load_path_processing_data()
        remove_corrupted_lines(localization_debug_file_path, 6)
        localization_debug_data = pd.read_csv(localization_debug_file_path, index_col=False)
        with open(steering_control_file_path) as f:
            lines = f.readlines()
            if len(lines) > 0:
                # time [sec],target_point_x [m],target_point_y [m],reference_heading_raw [rad],reference_heading_filtered [rad],vehicle_heading [rad],heading_error [rad],lateral_error [m],delta_cmd [rad],steering_wheel_cmd_raw [rad],steering_wheel_cmd_filtered [rad],
                remove_corrupted_lines(steering_control_file_path, 11)
                steering_control_debug_data = pd.read_csv(steering_control_file_path)
            else:
                print("no line to read in steering file")
        t_start = 0
        # localization_debug_data['time [sec]'] = handle_array_of_strings(np.array(localization_debug_data['time [sec]']))
        t_end = np.array(localization_debug_data['time [sec]'])[-1] - np.array(localization_debug_data['time [sec]'])[0]
        if script_config["compare_results_with_offline_trip_data"]:
            trip_obj.segment_trip(t_start, t_end)
    if script_config["analyze_motion_planning_dump_files"]:
        if script_config["use_latest_motion_planning_read_folder"] and not script_config["copy_and_delete_motion_planning_files"]:
            latest_dir = get_latest_directory(script_config["motion_planning_read_directory"])
            script_config["motion_planning_read_directory"] = Path(script_config["motion_planning_read_directory"]) / latest_dir
        if (check_if_MP_files_exist(script_config["motion_planning_read_directory"]) and
                not script_config["copy_and_delete_motion_planning_files"]):
            print("motion planning files found in data folder")
        elif (script_config["copy_and_delete_motion_planning_files"] and
              check_if_MP_files_exist(script_config["motion_planning_dump_directory"])):
            script_config["motion_planning_read_directory"] = create_timestamped_directory(script_config["motion_planning_read_directory"])
            copy_MP_files_to_data_folder(script_config["motion_planning_dump_directory"], script_config["motion_planning_read_directory"])
            delete_MP_files(script_config["motion_planning_dump_directory"])
        else:
            print("path files are not found in either the read folder nor the dump folder. continuing without analyzing path dump files")
            script_config["analyze_motion_planning_dump_files"] = False
            if not script_config["analyze_lateral_control"]:
                raise "no data to debug: control is disabled and no motion planning files"

        if script_config["analyze_motion_planning_dump_files"]:
            if script_config["analyze_lateral_control"]:
                input_path_to_control = {key: {"x": path_processing_data[key]['input_path_x'],
                                           "y": path_processing_data[key]['input_path_y']} for key in path_processing_data.keys()}
            motion_planning_debug_obj = MotionPlanningDebug(script_config["motion_planning_read_directory"])
            if script_config["analyze_lateral_control"]:
                motion_planning_debug_obj.parse_control_localization(localization_debug_data)
            if script_config["analyze_space_net"]:
                achieve_user_permission(script_config["space_net_read_directory"])
                if script_config["copy_and_delete_space_net_files"]:
                    achieve_user_permission(script_config["space_net_dump_directory"])
                    if Path(script_config["space_net_read_directory"]).exists():
                        delete_space_net_files(script_config["space_net_read_directory"])
                    copy_space_net_files()
                    delete_space_net_files(script_config["space_net_dump_directory"])
                motion_planning_debug_obj.create_SN_data_structure(script_config["space_net_read_directory"])
                print('space net files read from ' + script_config["space_net_read_directory"])
            if script_config["analyze_carpose"]:
                assert check_if_car_pose_exist()
                if script_config["analyze_online_run"]:
                    car_pose_file_container = script_config["aidriver_trip_path"]
                elif script_config["copy_and_delete_motion_planning_files"]:
                    car_pose_file_container = script_config["motion_planning_dump_directory"]
                else:
                    car_pose_file_container = script_config["motion_planning_read_directory"]
                motion_planning_debug_obj.parse_car_pose_file(car_pose_file_container)
            if script_config["analyze_road_layer"]:
                motion_planning_debug_obj.create_road_layer_data_structure(script_config["space_net_read_directory"])
                print('road layer files read from ' + script_config["space_net_read_directory"])
            if script_config["analyze_lane_dividers"]:
                motion_planning_debug_obj.create_lane_dividers_data_structure(script_config["space_net_read_directory"])
                print('lane divider files read from ' + script_config["space_net_read_directory"])
            if script_config["analyze_vehicles"]:
                motion_planning_debug_obj.create_vehicles_data_structure(script_config["space_net_read_directory"])
                print('vehicles files read from ' + script_config["space_net_read_directory"])
            if script_config["analyze_MPC_solution"]:
                """
                    when running with data collection in ai-driver, the MPC solution 
                    file is saved in the trip folder.
                """
                if script_config["analyze_online_run"]:
                    mpc_solution_container = Path(script_config["aidriver_trip_path"])
                else:
                    mpc_solution_container = Path(script_config["motion_planning_read_directory"])
                motion_planning_debug_obj.create_MPC_data_structure(mpc_solution_container)
            if script_config["analyze_carpose"] and script_config["analyze_motion_planning_dump_files"]:
                motion_planning_debug_obj.calculate_planner_path_in_world_frame()
                motion_planning_debug_obj.calculate_erros_relative_to_planner_path()
    else:
        motion_planning_debug_obj = None
    # plot_map_desired_boundries()
    # compare_carpose_to_gps()
    analyze_experiment()
    # analyze_lateral_control_performance(motion_planning_debug_obj)
    # animate_data(0)
    # analyze_call_times_and_delay()
    # analyze_motion_planning_times()
    # trip_obj.plot_lat_lon()////
    # trip_obj.plot_heading()
    # plot_steering()
    # plot_localization()
    # analyze_imu_dt()
    # plot_AHRS()
    # analyze_traj_processing()
    # plot_trip_map(X_gps_lng_lat, trip_name=trip_name)
    # debug_steering_cmd_calculation()
    # plot_control_target_points()
    plt.show()