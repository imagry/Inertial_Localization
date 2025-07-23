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
from Classes import VehicleDynamicModel, VehicleKinemaicModel
import Functions
from SteeringControlPlotter import SteeringControlPlotter
BUILD_PATH = './Tests/python/python_binding/build'
if BUILD_PATH not in sys.path:
    sys.path.append(BUILD_PATH)
from control_module import ControlAPI

# import & set simulation configurations
with open('vehicle_config.json', "r") as f:
    vehicle_params = json.loads(f.read())
simulation_params = {'dt': 0.01, 't_end': 100, 'animation_dt': 0.1, 'motion_planning_dt': 0.25,
                     'ego_frame_placement': 'front_axle', 'velocity_KPH': 25,
                     "motion_planning_horizon": 20, "motion_planing_path_spacing": 1.0, "control_interp_spacing": 0.1,
                     'velocity_variations': 0.1, 'heading_drift_noise': 1.0,#deg
                     'model': 'Dynamic', #'Kinematic', 'Dynamic'
                     'animate': False, 'plot_results': True, 'save_results': False, 'motion_planning_debug': False,
                     'animation_mode': 'GT_navigation_frame'} # Est_navigation_frame, GT_navigation_frame
t = np.arange(0, simulation_params['t_end'], simulation_params['dt'])
# generate path
scenario = 'random_curvature' # 'sin', 'straight_line', 'square', shiba, random_curvature, eight, ellipse
desired_traj_x, desired_traj_y, desired_traj_psi, desired_traj_k, desired_traj_s = Functions.calc_desired_path(scenario)
desired_traj = np.vstack([desired_traj_x, desired_traj_y]).T
traj_length = desired_traj_s[-1]
# create vehicle agent
error_x = 0.0
error_y = 1.0
if simulation_params['model'] == 'Dynamic':
    vehicle_obj = VehicleDynamicModel(x=desired_traj_x[0] + error_x, y=desired_traj_y[0] + error_y,
                                      psi=desired_traj_psi[0],
                                      vehicle_params=vehicle_params, simulation_params=simulation_params,
                                      steering_uncertainty_factor=1.0, lr_uncertainty_factor=1.0,
                                      WB_uncertainty_factor=1.0,
                                      m_uncertainty_factor=1.0, I_uncertainty_factor=1.0,
                                      C_uncertainty_factor=1.0)
    # vehicle_obj.C = 1e5
elif simulation_params['model'] == 'Kinematic':
    vehicle_obj = VehicleKinemaicModel(vehicle_params=vehicle_params, simulation_params=simulation_params,
                                       steering_uncertainty_factor=1.0, lr_uncertainty_factor=1.0,
                                       WB_uncertainty_factor=1.0)
else:
    raise 'invalid model type'
# initialize control
cpp_control_obj = ControlAPI(float(t[0]))
cpp_control_obj.ResetVehicleState(t[0], [vehicle_obj.x, vehicle_obj.y, vehicle_obj.psi, vehicle_obj.vx])
# measurement errors
speed_variations = simulation_params['velocity_variations'] * (np.random.rand(t.shape[0])-0.5).cumsum() # we add random walk signal to the actual constant velocity
# create a velocity profile that starts from zero
ideal_speed_profile = Functions.regulate_random_signal(np.zeros(t.shape), simulation_params['velocity_KPH'], gain=0.001, plot_res=False)
actual_speed = ideal_speed_profile + speed_variations
# Functions.regulate
odometer_signal = actual_speed.round()
heading_measurement_errors = simulation_params['heading_drift_noise'] * np.pi/180 * (np.random.rand(t.shape[0])-0.5).cumsum() # we add random walk signal to the heading as measurement error
# initialize stored variables
x = []
y = []
x.append(vehicle_obj.x)
y.append(vehicle_obj.y)
psi = []
psi.append(vehicle_obj.psi)
P_est = [copy(cpp_control_obj.GetPosition())]
front_slip_angle = []
rear_slip_angle = []
delta = []
ef = []
psi_traj = []
stop_condition = False
i = 0
if simulation_params['animate']:
    plotter_obj = SteeringControlPlotter(car_params=vehicle_params)
    if simulation_params['animation_mode'] == 'GT_navigation_frame':
        plotter_obj.desired_traj_x = desired_traj_x
        plotter_obj.desired_traj_y = desired_traj_y
ndt_animation = int(simulation_params['animation_dt'] / simulation_params['dt'])
ndt_motion_planning = int(simulation_params['motion_planning_dt'] / simulation_params['dt'])
pbar = tqdm(total=len(t))
desired_traj_s_i = 0
# simulation starts here
while not stop_condition:
    if np.mod(i, ndt_motion_planning) == 0: # motion planning calculation
        idx_start = Functions.project_point_on_path(point=[vehicle_obj.x, vehicle_obj.y],
                                                    path=desired_traj)
        idx_end = np.argmin(
            abs(desired_traj_s - (desired_traj_s[idx_start] + simulation_params["motion_planning_horizon"])))
        # interpulate based on evenly spaced path length
        s_interp = np.arange(desired_traj_s[idx_start], desired_traj_s[idx_end],
                             step=simulation_params["motion_planing_path_spacing"])
        cond4 = s_interp.shape[0] <= 5
        if cond4:
            print('condition 4 is met')
        else:
            desired_traj_x_interp = np.interp(x=s_interp, xp=desired_traj_s, fp=np.array(desired_traj_x))
            desired_traj_y_interp = np.interp(x=s_interp, xp=desired_traj_s, fp=np.array(desired_traj_y))
            horizon_desired_traj = np.vstack([desired_traj_x_interp, desired_traj_y_interp]).T
            # desired path observation in ego frame
            desired_traj_ego_frame_horizon, _ = Functions.project_points_2D(vehicle_obj.x, vehicle_obj.y,
                                                                            vehicle_obj.psi, horizon_desired_traj)
            if simulation_params['animate'] and (simulation_params['animation_mode'] == 'Est_navigation_frame'):
                # calculate the reference path at the estimated navigation frame for animation
                Phat = cpp_control_obj.GetPosition()
                psi_hat = cpp_control_obj.GetVehicleHeading()
                Tnav_hat2ego = Functions.affine_transformation_matrix_2D(Phat[0], Phat[1], psi_hat)
                Tego2navhat = Functions.inv_affine_transformation_matrix_2D(Tnav_hat2ego)
                reference_path_at_nav_hat = Functions.project_points_2D(Tego2navhat, desired_traj_ego_frame_horizon)
            # C++ controller motion planning observation
            motion_planning_y_axis_direction = -1.0 #
            cpp_control_obj.MotionPlanningUpdate(list(desired_traj_ego_frame_horizon[:, 0]),
                                                 list(motion_planning_y_axis_direction *
                                                      desired_traj_ego_frame_horizon[:, 1]),
                                                 t[i],
                                                 "EGO")
            Phat = cpp_control_obj.GetPosition()
            psi_hat = cpp_control_obj.GetVehicleHeading()
            Tnn_hat = Functions.nav_est_2_nav_affine_transformation_2D(Phat[0], Phat[1], psi_hat,
                                                                       vehicle_obj.x, vehicle_obj.y, vehicle_obj.psi)
            ref_traj = cpp_control_obj.GetReferenceTrajectory()
            ref_traj = np.vstack([ref_traj[0], ref_traj[1]]).T
            ref_traj_in_nav_frame = Functions.project_points_2D(Tnn_hat, ref_traj)
            if i == 0:
                delta_i = 0.0
            else:
                delta_i = cpp_control_obj.GetDelta()
            if simulation_params["motion_planning_debug"]:
                plt.figure(figsize=(20,10))
                h1 = plt.subplot(121)
                h1.set_title('Navigation Frame')
                # nav frame
                plt.plot(desired_traj[:, 0], desired_traj[:, 1], linewidth=5, color='gray', linestyle='--', label='GT')
                # plt.scatter(horizon_desired_traj[:, 0], horizon_desired_traj[:, 1], s=30, color='green',
                #             label='observation')
                plt.scatter(ref_traj_in_nav_frame[:, 0], ref_traj_in_nav_frame[:, 1], s=5, color='blue',
                            label='(fixed) smooth reference path, $T^n_{\hat{n}} \cdot T^{\hat{n}}_{EGO} \cdot Path^{EGO}$')
                Functions.draw_car(vehicle_obj.x, vehicle_obj.y, vehicle_obj.psi, steer=delta_i, car_params=vehicle_params,
                                   ax=plt.gca())
                plt.grid(True), plt.legend()
                h1.set_xlim(vehicle_obj.x - 10, vehicle_obj.x + 20)
                h1.set_ylim(vehicle_obj.y - 20, vehicle_obj.y + 20)
                # h1.axis('equal')
                h2 = plt.subplot(122)
                h2.set_title('EGO Frame')
                desired_traj_ego_frame, _ = Functions.project_points_2D(vehicle_obj.x, vehicle_obj.y,
                                                                                vehicle_obj.psi, desired_traj)
                plt.plot(desired_traj_ego_frame[:, 0], desired_traj_ego_frame[:, 1], linestyle='--', linewidth=0.5, color='gray', label='reference')
                plt.scatter(desired_traj_ego_frame_horizon[:, 0], desired_traj_ego_frame_horizon[:, 1], label='observation')
                Functions.draw_car(0, 0, 0, steer=delta_i, car_params=vehicle_params, ax=plt.gca())
                plt.grid(True), plt.legend()
                plt.gca().set_xlim(- 5, + 20)
                plt.gca().set_ylim(- 20, + 20)
                # plt.gca().axis('equal')
                plt.show()
    ti = t[i]
    steering_update_success = cpp_control_obj.CalculateSteeringCommand(t[i])
    # assert steering_update_success
    vehicle_obj.update(a=0, delta=cpp_control_obj.GetDelta())
    vehicle_obj.vx = actual_speed[i] / 3.6
    # localization object is propagated using the same eq as vehicle equations (front axle configuration with delta)
    delta_i = max(cpp_control_obj.GetDelta(), -vehicle_params['MAX_STEER'])
    delta_i = min(delta_i, vehicle_params['MAX_STEER'])
    steering_wheel_angle = cpp_control_obj.GetDelta() * vehicle_params["steering_ratio"]
    cpp_control_obj.UpdateSteeringWheel(steering_wheel_angle, t[i])
    # cpp_control_obj.UpdatePosition(t[i])
    cpp_control_obj.UpdateSpeed(odometer_signal[i] / 3.6, t[i])
    cpp_control_obj.UpdateHeading(vehicle_obj.psi + heading_measurement_errors[i], t[i])
    P_est.append(cpp_control_obj.GetPosition())
    # store values
    x.append(vehicle_obj.x)
    y.append(vehicle_obj.y)
    if simulation_params['model'] == 'Dynamic':
        front_slip_angle.append(vehicle_obj.alpha_f)
        rear_slip_angle.append(vehicle_obj.alpha_r)
    dx = x[-1] - x[-2]
    dy = y[-1] - y[-2]
    desired_traj_s_i += np.linalg.norm(np.array([dx, dy]))
    psi.append(vehicle_obj.psi)
    delta.append(cpp_control_obj.GetDelta())
    ef.append(cpp_control_obj.GetLateralError())
    cond1 = i >= t.shape[0] - 1
    cond2 = np.linalg.norm(np.array([vehicle_obj.x - desired_traj_x[-1], vehicle_obj.y - desired_traj_y[-1]])) < 1.0
    cond3 = desired_traj_s_i >= traj_length * 5.0
    stop_condition = cond1 or cond2 or cond3 or cond4
    if cond1: print('condition 1 is met')
    if cond2: print('condition 2 is met')
    if cond3: print('condition 3 is met')
    i += 1
    pbar.update(1)
    if np.mod(i, ndt_animation) == 0 and simulation_params['animate']:
        if simulation_params['animation_mode'] == 'GT_navigation_frame':
            plotter_obj.update_plot(current_car_position_x=vehicle_obj.x,
                                    current_car_position_y=vehicle_obj.y,
                                    current_car_heading=vehicle_obj.psi,
                                    steering=cpp_control_obj.GetDelta(),
                                    motion_planning_traj_raw=horizon_desired_traj,
                                    motion_planning_traj_processed=ref_traj_in_nav_frame,
                                    lateral_error=ef[-1],
                                    t_i=ti)
        elif simulation_params['animation_mode'] == 'Est_navigation_frame':
            Phat = cpp_control_obj.GetPosition()
            psi_hat = cpp_control_obj.GetVehicleHeading()
            Tnav_hat2ego = Functions.affine_transformation_matrix_2D(Phat[0],Phat[1], psi_hat)
            Tego2navhat= Functions.inv_affine_transformation_matrix_2D(Tnav_hat2ego)
            # Tnn_hat = Functions.nav_est_2_nav_affine_transformation_2D(Phat[0], Phat[1], psi_hat,
            #                                                            vehicle_obj.x, vehicle_obj.y, vehicle_obj.psi)
            ref_traj_in_nav_hat_frame = np.array(cpp_control_obj.GetReferenceTrajectory()).T
            # ref_traj_in_nav_hat_frame = np.vstack([ref_traj_in_nav_hat_frame[0], ref_traj_in_nav_hat_frame[1]]).T
            # ref_traj_in_nav_frame = Functions.project_points_2D(Tnn_hat, ref_traj_in_nav_hat_frame)
            plotter_obj.update_plot(current_car_position_x=Phat[0],
                                    current_car_position_y=Phat[1],
                                    current_car_heading=psi_hat,
                                    steering=cpp_control_obj.GetDelta(),
                                    motion_planning_traj_raw=reference_path_at_nav_hat,
                                    motion_planning_traj_processed=ref_traj_in_nav_hat_frame,
                                    lateral_error=cpp_control_obj.GetLateralError(),
                                    t_i=ti)
        else:
            raise 'invalid animation mode'

pbar.close()
if simulation_params['save_results']:
    vehicle_states_dic = {'x': x, 'y': y, 'psi': psi, 'time': t[:i + 1]}
    control_states_dic = {'delta': delta, 'ef': ef, 'time': t[:i]}
    trajectory_dic = {'traj_x': desired_traj_x, 'traj_y': desired_traj_y, 'traj_psi': desired_traj_psi}
    Functions.save_csv(vehicle_states_dic, path='./data/vehicle_states.csv', print_message=True)
    Functions.save_csv(control_states_dic, path='./data/control_states.csv', print_message=True)
    Functions.save_csv(trajectory_dic, path='./data/desired_traj.csv', print_message=True)
if simulation_params['plot_results']:
    # plot the speed and heading errors
    plt.figure('measurement errors')
    h = plt.subplot(311)
    plt.title('measurement errors')
    plt.plot(t, actual_speed, label='actual_speed')
    plt.plot(t, odometer_signal, label='odometer_signal')
    plt.grid(True), plt.ylabel('speed [KPH]'), plt.legend()
    plt.subplot(312, sharex=h)
    plt.plot(t, actual_speed - odometer_signal, label='odometer error')
    plt.grid(True), plt.ylabel('odometer error')
    plt.subplot(313, sharex=h)
    plt.plot(t, heading_measurement_errors * 180 / np.pi, label='heading measurement errors')
    plt.grid(True), plt.ylabel('heading measurement \n errors [deg]')

    animation_figure = plt.figure('stanley control on dynamic model')
    vehicle_animation_axis = plt.subplot(1, 2, 1)
    plt.title("BEV"), plt.xlabel('[m]'), plt.ylabel('[m]')
    ref_traj_line = vehicle_animation_axis.plot(desired_traj_x, desired_traj_y, color='gray', linewidth=2.0,
                                                label='desired')
    vehicle_traj_line = vehicle_animation_axis.plot(x, y, linewidth=2.0, color='darkviolet', label='actual')
    P_est = np.array(P_est)
    plt.plot(P_est[:, 0], P_est[:, 1], label='P est')
    plt.legend()
    # vehicle_animation_axis.axis("equal")
    vehicle_animation_axis.grid(True)
    lateral_error_axis = plt.subplot(2, 2, 2)
    ef = np.array(ef)
    plt.title('lateral error, STD = ' + str("%.2f" % np.std(ef)) + ' [m]')
    lateral_error_axis.plot(t[:i], ef)
    lateral_error_axis.grid(True)
    plt.xlabel('time [sec]'), plt.ylabel('ef [m]')
    plt.subplot(2, 2, 4)
    plt.plot(t[:i], delta), plt.grid(True), plt.ylabel('steering [rad]')
    plt.figure('track error distribution')
    plt.hist(ef, bins=100)
    plt.grid(True), plt.title('track error distribution')
    if simulation_params['model'] == 'Dynamic':
        plt.figure('slip angles')
        plt.plot(t[:i], front_slip_angle, label='front_slip_angle')
        plt.plot(t[:i], rear_slip_angle, label='rear_slip_angle')
        plt.grid(True), plt.legend()

    plt.show()