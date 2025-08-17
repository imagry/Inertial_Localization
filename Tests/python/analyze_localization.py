import copy
from pathlib import Path
import os
import matplotlib.pyplot as plt
import json
import numpy as np
from functools import partial
from itertools import pairwise
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
import Classes
import argparse
from tqdm import tqdm


if __name__ == '__main__':
    # Setup argument parser
    parser = argparse.ArgumentParser(description='Analyze car pose localization compared to GPS data.')
    parser.add_argument('--trip_path', type=str, 
                        default='/home/eranvertz/imagry/trips/RTK_Haifa/2024-10-28T16_02_54​/',
                        help='Path to the trip data directory')
    parser.add_argument('--metric_type', type=str, choices=['Driving', 'Time', 'Turning'], 
                        default='Driving',
                        help='Type of metric to use for comparison (default: Driving)')
    parser.add_argument('--carpose_path', type=str, default=None,
                        help='Optional path to car pose CSV file; if not provided, discovered from trip_path')
    args = parser.parse_args()
    script_dir = os.path.dirname(os.path.abspath(__file__))
    def compare_carpose_to_gps(metric_type="Driving"):
        """
        Compare car pose to GPS data to evaluate localization quality.
        
        Args:
            metric_type (str): The type of metric to use. Can be "Driving", "Time", or "Turning".
            
        Returns:
            bool: True if function completes successfully.
        """
        def parse_car_pose_file(car_pose_file_path):
            with open(car_pose_file_path, "r") as f:
                lines = f.readlines()
            data = {"x":[], "y":[], "psi":[], "timestamp":[]}
            for line in lines[1:]:
                splited = line.split(',')
                data["timestamp"].append(float(splited[0]))
                data["x"].append(float(splited[1]))
                data["y"].append(- float(splited[2]))
                data["psi"].append( - float(splited[3]) * np.pi / 180)# assume psi is in degrees
                # minus signes are because the carpose is saved in ENU frame 
            return data
        
        def time_interval_indexes(gt_timestamps, time_interval):
            dt = np.diff(gt_timestamps).mean()
            window_size = round(time_interval[0] / dt)
            n = len(gt_timestamps)

            return (range(0, n - window_size), range(window_size,n))

        def calc_pos_err_on_time_interval(time, gt, est, time_interval):
            indexes = time_interval_indexes(time, time_interval)
            dp_error_norm = kuemmerle_error(gt, est, indexes,
                                        lambda ref, est: np.linalg.norm(ref[:, :2]-est[:, :2], axis=1),
                                        lambda x: np.zeros_like(x[:, 0]))
            return dp_error_norm
            
        def plot_error(dp_error_norm):
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

        def indices_of_driving_intervals(ref_poses, window, driving_interval):
            '''
            Find the indices of windows where the driven distance is withing driving_interval.
            Args:
                ref_poses: array of shape (n, 3) with the reference poses
                window: size of the window in ref_poses (window+1 poses are used)
                driving_interval: the driving interval to fit into 
            '''
            distances = np.linalg.norm(np.diff(ref_poses[:, :2], axis=0), axis=1)
            cum_distances = np.cumsum(distances)
            cum_distances = np.concatenate(([0], cum_distances))
            distances_over_window = cum_distances[window:] - cum_distances[:-window]

            windows_within_tolerance = (driving_interval[0] < distances_over_window) & (distances_over_window < driving_interval[1])
            indices = np.where(windows_within_tolerance)[0]

            return np.column_stack((indices, indices+window))

        def indexes_of_turning_intervals(ref_poses, window, turning_interval):
            '''
            Find the indices of windows where the total turn is within turning_interval.
            Args:
                ref_poses: array of shape (n, 3) with the reference poses
                window: size of the window in ref_poses (window+1 poses are used)
                turning_interval: the turning interval to fit into 
            '''
            rotations_over_window = (ref_poses[:, 2][window:] - ref_poses[:, 2][:-window]) / np.pi * 180 # convert to degrees

            windows_within_tolerance = (turning_interval[0] < rotations_over_window) & (rotations_over_window < turning_interval[1])
            indices = np.where(windows_within_tolerance)[0]

            return np.column_stack((indices, indices+window))

        def calc_pos_err_VS_driving_interval_(time, gt, est, driving_interval):
            ''' 
            calculate the position error vs the driving interval
            Args:
                time: common time of bothgt and est after interpolation
                gt: reference poses
                est: estimated poses
                driving_interval: set of driving intervals ni X 2 where ni is number of intervals and each row is [start, end]
            Returns:
                dp_error_norm: TODO: fill this 
            '''
            dt = np.diff(time).mean()
            window_size = 200 # 2 seconds 
            n = len(time)
            indices = indices_of_driving_intervals(gt, window_size, driving_interval)# nw X 2 where nw is number of windows
            # rot_angles = gt[:, 2] - est[:, 2] # n X 1
            def calculate_rot_angles_vec():
                # iterate over indices
                # for each index pair, call rotate_traj_to_match_initial_heading() 
                # output a vector or rotation angles of size nw. 
                for i in range(indices.shape[0]):
                    start_idx = indices[i, 0]
                    end_idx = indices[i, 1]
                    ref_poses = gt[start_idx:end_idx]
                    est_poses = est[start_idx:end_idx]
                    rot_angle = rotate_traj_to_match_initial_heading(est_poses, ref_poses)
                    rot_angles.append(rot_angle)
                return np.array(rot_angles)
                
            def segment_end_position_error(ref_poses, est_poses, rot_angle):
                '''
                rotate (and translate?) the est_poses by rot_angle and calculate the eucledean
                distance between end positions.
                
                '''
                rot_matrices = create_rotation_matrices(rot_angle)
                # apply each rotation to its respective point
                transformed_points = np.einsum('ijk,ik->ij', rot_matrices, est_poses[:, :2])
                # calculate the translation error
                return np.linalg.norm(ref_poses[:, :2] - transformed_points, axis=1)
            dp_error_norm = kuemmerle_error(gt, est, indices,
                                        partial(segment_end_position_error, delta_headings=rot_angles[indices[:, 0]]),
                                        lambda x: np.zeros_like(x[:, 0]))
            return dp_error_norm

        def calc_pos_err_VS_driving_interval(time, gt, est, driving_interval):
            ''' 
            calculate the position error vs the driving interval. 
            1. Interpolate paths based on arc length. this opreration will remove zero diffs.
            2. calculate the indices of driving interval.
            3. for each interval, rotate the est to match the initial heading of the ref.
            4. calculate the end position error.
            5. return the end position error.
            Args:
                time: common time of bothgt and est after interpolation
                gt: reference poses
                est: estimated poses
                driving_interval: set of driving intervals ni X 2 where ni is number of intervals and each row is [start, end]
            Returns:
                dp_error_norm: TODO: fill this 
            '''
            dt = np.diff(time).mean()
            window_size = 200 # 2 seconds 
            n = len(time)
            est_no_zero_diff, est_new_idx, gt_interp, s_common = interpolate_path_by_arc_length(est, gt) # est is rewrited without zero diffs
            indices = indices_of_driving_intervals(gt_interp, window_size, driving_interval)# nw X 2 where nw is number of windows
            # rot_angles = gt[:, 2] - est[:, 2] # n X 1
            # iterate over indices
            # for each index pair, call rotate_traj_to_match_initial_heading() 
            # plots to debug this function 
            # plt.figure('calc_pos_err_VS_driving_interval')
            # plt.plot(gt[:, 0], gt[:, 1], label='gt org')
            # plt.scatter(gt_interp[:, 0], gt_interp[:, 1], label='gt_interp')
            # plt.plot(est[:, 0], est[:, 1], label='est org')
            # plt.scatter(est_no_zero_diff[:, 0], est_no_zero_diff[:, 1], label='est_no_zero_diff')
            
            # plt.title('XY Trajectory')
            # plt.xlabel('X')
            # plt.ylabel('Y')
            # plt.axis('equal')
            # plt.legend()
            # ax = plt.gca()
            segment_end_position_errors = []
            tqdm_message = "calculating errors for interval = " + str(driving_interval)
            for i in tqdm(range(indices.shape[0]), desc=tqdm_message):
                start_idx = indices[i, 0]
                end_idx = indices[i, 1]
                ref_sample = gt_interp[start_idx:end_idx]
                # ax.plot(ref_sample[:, 0], ref_sample[:, 1], label='ref_sample')
                ref_sample = ref_sample - ref_sample[0]
                est_sample = est_no_zero_diff[start_idx:end_idx]
                # ax.plot(est_sample[:, 0], est_sample[:, 1], label='est_sample')
                est_sample = est_sample - est_sample[0]
                # plt.legend()
                # plt.show()
                if driving_interval[0] > 2 and False:
                    est_sample_rot, _ = rotate_traj_to_match_initial_heading(ref_sample, est_sample, plot=True)    
                else:
                    est_sample_rot, _ = rotate_traj_to_match_initial_heading(ref_sample, est_sample, plot=False)    
                segment_end_position_errors.append(np.linalg.norm(est_sample_rot[:2] - ref_sample[:2]))
                
            return np.array(segment_end_position_errors)
        
        def calc_rot_err_on_turning_interval(time, gt, est, turning_interval):
            window_size = 200 # 2 seconds
            indices = indexes_of_turning_intervals(gt, window_size, turning_interval)
            dp_error_norm = kuemmerle_error(gt, est, indices,
                                        lambda x, y: np.zeros_like(x[:, 2]),
                                        lambda x: x[:, 2] / np.pi * 180) # convert to degrees
            return dp_error_norm
        
        
        # Select car pose path from optional CLI arg or discover from trip_path
        if args.carpose_path is not None:
            if not os.path.exists(args.carpose_path):
                raise FileNotFoundError(f"Provided carpose_path does not exist: {args.carpose_path}")
            carpose_file = args.carpose_path
        else:
            carpose_file = car_pose_path()
        carpose = parse_car_pose_file(carpose_file)
        carpose_x = np.array(carpose['x'])
        carpose_y = np.array(carpose['y'])
        carpose_yaw = np.array(carpose['psi'])
        carpose_yaw = continuous_angle(carpose_yaw, 360)
        carpose_x -= carpose_x[0]
        carpose_y -= carpose_y[0]
        gt = np.column_stack([trip_obj.N, trip_obj.E, trip_obj.yaw_interp])
        est = np.column_stack(([carpose_x, carpose_y, carpose_yaw]))
        carpose_rotated, carpose_new_idx = rotate_traj_to_match_initial_heading(gt, est, plot=False)
        
        plt.figure('compare_carpose_to_gps initial states')
        """the figures y-axis is treated as "forward" and x-axis is "right" this would plot a NED configurations"""
        plt.plot(gt[:, 1], gt[:, 0], label='gps')
        plt.plot(carpose_rotated[:, 1], carpose_rotated[:, 0], label='localization')
        plt.grid(True), plt.legend(), plt.ylabel('N [m]'), plt.xlabel('E [m]')#, plt.axis('equal')
        # calculate estimated position error in sampled time windows
        gps_time = trip_obj.common_time - trip_obj.common_time[0]
        est_time = np.array(carpose["timestamp"]) - carpose["timestamp"][0]
        P_est_x_interp = np.interp(gps_time, est_time[carpose_new_idx], carpose_rotated[:, 0])
        P_est_y_interp = np.interp(gps_time, est_time[carpose_new_idx], carpose_rotated[:, 1])
        P_est_yaw_interp = np.interp(gps_time, est_time[carpose_new_idx], carpose_rotated[:, 2])
        
        P_est_interp = np.column_stack([P_est_x_interp, P_est_y_interp, P_est_yaw_interp])
        metrics = {"Driving": (calc_pos_err_VS_driving_interval, np.arange(0.1, 120, 1.0), # driving intervals in meters
                               "Driving range [m]", "Position estimation error [m]"),
                   "Time": (calc_pos_err_on_time_interval, np.arange(0.1, 1, step=0.1),
                            "Time range [sec]", "Position estimation error [m]"),
                   "Turning": (calc_rot_err_on_turning_interval, np.arange(-180, 180, step=1),
                               "Turning range [deg]", "Heading estimation error [deg]")}
        # Use the metric_type provided in the parameter
        if metric_type not in metrics:
            print(f"Warning: Unknown metric_type '{metric_type}', defaulting to 'Driving'")
            metric_type = "Driving"
        calc_fn, intervals, x_label, y_label = metrics[metric_type]

        err = []
        for value_range in pairwise(intervals):
            errors = calc_fn(gps_time, gt, P_est_interp, value_range)
            if False:
                plot_error(errors)
            # Check if the array is empty to avoid warnings
            if len(errors) > 0:
                err.append((errors.mean(), errors.std()))
            else:
                print(f"Warning: Empty error array for range {value_range}, skipping")
                err.append((np.nan, np.nan))  # Add NaN to keep the array length consistent
        err = np.array(err)
        intervals = np.array(intervals)[1:]
        # Compute the slopes between each consecutive pair of points
        slopes = np.diff(err[:, 0]) / np.diff(intervals)
        # Calculate the mean slope, ignoring NaN values
        mean_slope = np.nanmean(slopes)
        plt.figure()
        plt.plot(intervals, err[:, 0], label='mean error')
        plt.fill_between(intervals, err[:, 0] - err[:, 1], err[:, 0] + err[:, 1], alpha=0.2, label='std')
        plt.grid(True)
        plt.xlabel(x_label), plt.ylabel(y_label)
        plt.title("Mean Slope:" + str("%.2f" % mean_slope))
        print("Successfully completed compare_carpose_to_gps function")
        return True
    def kuemmerle_error(ref_poses, measured_poses, start_end_idx_pairs, trans_func, rot_func):
        if type(start_end_idx_pairs) is not np.ndarray:
            start_end_idx_pairs = np.array(start_end_idx_pairs).T
        assert start_end_idx_pairs.shape[1] == 2, "pose indexes must come in pairs (start pose idx, end pose idx)"
        selected_relative_ref_poses = ref_poses[start_end_idx_pairs[:, 1]] - ref_poses[start_end_idx_pairs[:, 0]]
        selected_relative_measured_poses = measured_poses[start_end_idx_pairs[:, 1]] - measured_poses[start_end_idx_pairs[:, 0]]
        
        total_translation_error = trans_func(selected_relative_ref_poses, selected_relative_measured_poses)
        total_rotation_error = rot_func(selected_relative_ref_poses - selected_relative_measured_poses)
        
        return (total_translation_error + total_rotation_error) # / deltas_set.shape[1]
    def create_rotation_matrices(angles_rad):
        '''TODO: replace with Functions.Mpsi()'''
        cos_angles = np.cos(angles_rad)
        sin_angles = np.sin(angles_rad)
        
        # Create the rotation matrices using broadcasting
        rotation_matrices = np.array([
            [cos_angles, -sin_angles],
            [sin_angles, cos_angles]
        ]).transpose(2, 0, 1)  # Rearrange dimensions to (n, 2, 2)
        
        return rotation_matrices
    def rotate_trajectory(pts, angle):
        assert pts.shape[1] == 3, "path must be arranged as column (each point X,Y,psi is a row)"
        rot = Rotation.from_euler('ZYX', [angle, 0, 0], degrees=False).as_matrix()  # 3 X 3
        rot = rot[0:2, 0:2]
        rot_traj = pts[:, :2] @ rot.T  # N*2 = N*2 @ 2*2
        rot_headings = pts[:, 2] + angle
        rot_traj = np.hstack((rot_traj, rot_headings[:, np.newaxis]))  # N*2 + N*1
        return rot_traj
    def trajectory_error(est, gt, weights=None):
        err = np.linalg.norm(est[:, :2] - gt[:, :2], axis=1)
        if weights is not None:
            assert weights.shape == err.shape
            err = (err * weights).sum() / weights.sum()
        else:
            err = err.mean()
        return err
    def find_rotation_angle(est, gt, weights=None):
        def minimization_function(angle, traj_gt):
            gt_trans = rotate_trajectory(traj_gt, angle[0])
            return trajectory_error(est=est, gt=gt_trans, weights=weights)

        x0 = np.array([0.0])  # angle
        res = minimize(minimization_function, x0, method='nelder-mead', args=(gt,),
                        options={'xatol': 1e-8, 'disp': False})

        angle = res.x[0]
        return angle
    def continuous_angle(angles, period):
        """
        Convert angles in a specific range to a sequence without boundary discontinuities.
        """
        angles = np.asarray(angles)
        
        # Determine points where the angle jumps due to periodicity and compute the shift
        jumps_up = (np.diff(angles) > 0.8 * period).astype(int)
        jumps_down = (np.diff(angles) < -0.8 * period).astype(int)
        jumps = jumps_up - jumps_down

        # Calculate the total accumulated shift
        steps = np.cumsum(jumps) * period
        continuous_seq = angles.copy()
        continuous_seq[1:] -= steps
        
        return continuous_seq
    def car_pose_path():
        """Return path to car pose file"""
        car_pose_file_container = script_config["aidriver_trip_path"]
        for file in os.listdir(car_pose_file_container):
            if "car_pose" in file:
                res = file
                return Path(car_pose_file_container) / res
        raise FileNotFoundError(f"No car_pose file found in {car_pose_file_container}")
    def interpolate_path_by_arc_length(est, gt):
            """
            assume trajectories are numpy arrays with shape N X 3 (x, y, psi)
            1. segment gt to match trajectory length
            2. calculate arc lengths of both paths
            3. interpolate gt to est arc length
            """
            # Calculate cumulative distance along the trajectory
            
            s_diff_est = np.diff(est[:, :2], axis=0) # (n_est - 1) X 2
            est_diff_non_zero_idx = np.where(np.linalg.norm(s_diff_est[:,:2], axis=1) > 0)[0]
            s_diff_est = s_diff_est[est_diff_non_zero_idx] # n_est_nz X 2 where nz is non zero
            s_est = np.cumsum(np.linalg.norm(s_diff_est, axis=1))
            s_est = np.insert(s_est, 0, 0)
            est_new_idx = np.insert(est_diff_non_zero_idx, 0, 0)
            est_no_zero_diff = est[est_new_idx] # after removal of zero diff indices
            # s_est = np.cumsum(np.sqrt(np.sum(np.diff(est[:, :2], axis=0)**2, axis=1)))
            # s_est = np.insert(s_est, 0, 0)
            s_diff_gt = np.diff(gt[:, :2], axis=0) # (n_gt - 1) X 2
            gt_diff_non_zero_idx = np.where(np.linalg.norm(s_diff_gt[:,:2], axis=1) > 0)
            s_diff_gt = s_diff_gt[gt_diff_non_zero_idx] # n_gt_nz X 2 where nz is non zero
            # s_gt = np.cumsum(np.sqrt(np.sum(s_diff_gt**2, axis=1)))
            s_gt = np.cumsum(np.linalg.norm(s_diff_gt, axis=1))
            s_gt = np.insert(s_gt, 0, 0)
            gt = gt[np.insert(gt_diff_non_zero_idx, 0, 0)]
            # Find the last index in gt that corresponds to the est trajectory length
            gt_last_idx = min(len(s_gt)-1, np.searchsorted(s_gt, s_est[-1], side="left"))
            gt, s_gt = (gt[:gt_last_idx], s_gt[:gt_last_idx])
            gt_interp_x = np.interp(x=s_est, xp=s_gt, fp=gt[:, 0])
            gt_interp_y = np.interp(x=s_est, xp=s_gt, fp=gt[:, 1])
            gt_interp_yaw = np.interp(x=s_est, xp=s_gt, fp=gt[:, 2])
            
            return est_no_zero_diff, est_new_idx, np.column_stack([gt_interp_x, gt_interp_y, gt_interp_yaw]), s_est
    def rotate_traj_to_match_initial_heading(gt, est, plot=False):
        # # --- First Figure: Inputs ---
        # fig1, axs1 = plt.subplots(2, 2, figsize=(12, 8), num='rotate_traj_to_match_initial_heading inputs')
        # # Left column: XY plot
        # axs1[0, 0].plot(gt[:, 0], gt[:, 1], label='gt')
        # axs1[0, 0].plot(est[:, 0], est[:, 1], label='est')
        # axs1[0, 0].set_title('XY Trajectory')
        # axs1[0, 0].set_xlabel('X')
        # axs1[0, 0].set_ylabel('Y')
        # axs1[0, 0].axis('equal')
        # axs1[0, 0].legend()
        # # Right column, upper: X vs arc length
        # arc_gt = np.insert(np.cumsum(np.linalg.norm(np.diff(gt[:, :2], axis=0), axis=1)), 0, 0)
        # arc_est = np.insert(np.cumsum(np.linalg.norm(np.diff(est[:, :2], axis=0), axis=1)), 0, 0)
        # axs1[0, 1].plot(arc_gt, gt[:, 0], label='gt X')
        # axs1[0, 1].plot(arc_est, est[:, 0], '--', label='est X')
        # axs1[0, 1].set_title('X vs Arc Length')
        # axs1[0, 1].set_xlabel('Arc Length [m]')
        # axs1[0, 1].set_ylabel('X')
        # axs1[0, 1].legend()
        # # Right column, lower: Y vs arc length
        # axs1[1, 1].plot(arc_gt, gt[:, 1], label='gt Y')
        # axs1[1, 1].plot(arc_est, est[:, 1], '--', label='est Y')
        # axs1[1, 1].set_title('Y vs Arc Length')
        # axs1[1, 1].set_xlabel('Arc Length [m]')
        # axs1[1, 1].set_ylabel('Y')
        # axs1[1, 1].legend()
        # # Hide lower left subplot (not used)
        # axs1[1, 0].axis('off')
        # plt.tight_layout()
        

        gt_distance = np.linalg.norm(gt[-1, :2] - gt[0, :2])
        est_distance = np.linalg.norm(est[-1, :2] - est[0, :2])
        if gt_distance < 1e-1 or est_distance < 1e-1:
            return est, np.arange(len(est))

        def transform_r(est, gt, weights=None):
            angle = find_rotation_angle(est, gt, weights=weights)
            gt_trans = rotate_trajectory(gt, angle)
            err = trajectory_error(est=est, gt=gt_trans, weights=weights)
            return gt_trans, err, angle

        est_no_zero_diffs, est_new_idx, gt_interp, s_common = interpolate_path_by_arc_length(est, gt)  
        # est is rewritten without zero diffs, gt is interpolated to est arcc length
        

        # # --- Second Figure: Interpolation ---
        # fig2, axs2 = plt.subplots(2, 2, figsize=(12, 8), num='rotate_traj_to_match_initial_heading interpolation')
        # # Left column: XY plot
        # axs2[0, 0].plot(gt_interp[:, 0], gt_interp[:, 1], label='gt_interp')
        # axs2[0, 0].plot(est_no_zero_diffs[:, 0], est_no_zero_diffs[:, 1], label='est')
        # axs2[0, 0].set_title('XY Interpolated')
        # axs2[0, 0].set_xlabel('X')
        # axs2[0, 0].set_ylabel('Y')
        # axs2[0, 0].axis('equal')
        # axs2[0, 0].legend()
        # # Right column, upper: X vs arc length
        # # arc_gt_interp = np.insert(np.cumsum(np.linalg.norm(np.diff(gt_interp[:, :2], axis=0), axis=1)), 0, 0)
        # # arc_est_nz = np.insert(np.cumsum(np.linalg.norm(np.diff(est_no_zero_diffs[:, :2], axis=0), axis=1)), 0, 0)
        # axs2[0, 1].plot(s_common, gt_interp[:, 0], label='gt_interp X')
        # axs2[0, 1].plot(s_common, est_no_zero_diffs[:, 0], '--', label='est X')
        # axs2[0, 1].set_title('X vs Arc Length')
        # axs2[0, 1].set_xlabel('Arc Length [m]')
        # axs2[0, 1].set_ylabel('X')
        # axs2[0, 1].legend()
        # # Right column, lower: Y vs arc length
        # axs2[1, 1].plot(s_common, gt_interp[:, 1], label='gt_interp Y')
        # axs2[1, 1].plot(s_common, est_no_zero_diffs[:, 1], '--', label='est Y')
        # axs2[1, 1].set_title('Y vs Arc Length')
        # axs2[1, 1].set_xlabel('Arc Length [m]')
        # axs2[1, 1].set_ylabel('Y')
        # axs2[1, 1].legend()
        # # Hide lower left subplot (not used)
        # axs2[1, 0].axis('off')
        # plt.tight_layout()
        

        weights = (gt_interp.shape[0] - np.arange(gt_interp.shape[0])) / gt_interp.shape[0]
        power = 4
        weights = weights ** power
        err = trajectory_error(est=est_no_zero_diffs, gt=gt_interp, weights=weights)

        gt_r, err_r, angle_r = transform_r(est_no_zero_diffs, gt_interp, weights=weights)

        out_dict = {'err_r': err_r, 'angle_r': angle_r}
        est_r = rotate_trajectory(est_no_zero_diffs, -angle_r)
        if plot:
            # --- Third Figure: Outputs ---
            fig3, axs3 = plt.subplots(2, 2, figsize=(12, 8), num='rotate_traj_to_match_initial_heading outputs')
            # Left column: XY plot
            axs3[0, 0].plot(gt[:, 0], gt[:, 1], label='gt')
            axs3[0, 0].plot(est_r[:, 0], est_r[:, 1], '--', label=f'est rotated {err:.1f}')
            axs3[0, 0].plot([0], [0], '*k')
            axs3[0, 0].set_title('XY Rotated Output')
            axs3[0, 0].set_xlabel('X')
            axs3[0, 0].set_ylabel('Y')
            axs3[0, 0].axis('equal')
            axs3[0, 0].legend()
            # Right column, upper: X vs arc length
            # arc_gt_out = np.insert(np.cumsum(np.linalg.norm(np.diff(gt[:, :2], axis=0), axis=1)), 0, 0)
            # arc_est_r = np.insert(np.cumsum(np.linalg.norm(np.diff(est_r[:, :2], axis=0)), axis=1), 0, 0)
            axs3[0, 1].plot(s_common, gt[:, 0], label='gt X')
            axs3[0, 1].plot(s_common, est_r[:, 0], '--', label='est_rot X')
            axs3[0, 1].set_title('X vs Arc Length')
            axs3[0, 1].set_xlabel('Arc Length [m]')
            axs3[0, 1].set_ylabel('X')
            axs3[0, 1].legend()
            # Right column, lower: Y vs arc length
            axs3[1, 1].plot(s_common, gt[:, 1], label='gt Y')
            axs3[1, 1].plot(s_common, est_r[:, 1], '--', label='est_rot Y')
            axs3[1, 1].set_title('Y vs Arc Length')
            axs3[1, 1].set_xlabel('Arc Length [m]')
            axs3[1, 1].set_ylabel('Y')
            axs3[1, 1].legend()
            # Hide lower left subplot (not used)
            axs3[1, 0].axis('off')
            plt.tight_layout()
            plt.show()
        return est_r, est_new_idx
    # Set up script configuration from command-line arguments
    script_config = {
        "aidriver_trip_path": args.trip_path,
        "metric_type": args.metric_type
    }
    
    # Initialize only the minimal data needed by compare_carpose_to_gps
    aidriver_trip_path = Path(script_config["aidriver_trip_path"])
    print('Loading trip data from ' + str(aidriver_trip_path))
    trip_obj = Classes.Trip(aidriver_trip_path)
    
    # Run the main function with the specified metric type
    print(f"Running compare_carpose_to_gps function with metric type: {script_config['metric_type']}...")
    result = compare_carpose_to_gps(script_config["metric_type"])
    
    # Display results
    plt.show()
    
    if result:
        print("Script completed successfully!")
    else:
        print("Script encountered an issue.")