import os
import sys
import math
import numpy as np
np.random.seed(0)
import matplotlib.pyplot as plt
tests_python_path = "./Tests/python"
if tests_python_path not in sys.path:
    sys.path.append(tests_python_path)
from Classes import StanleyController, VehicleDynamicModel, ShortTermLocalization, VehicleKinemaicModel, DelayConstant, Delay
from SteeringControlPlotter import SteeringControlPlotter
import Functions
import json
import CurvesGenerator.cubic_spline as cs
from tqdm import tqdm
from copy import copy, deepcopy
BUILD_PATH = './Tests/python/python_binding/build'
if BUILD_PATH not in sys.path:
    sys.path.append(BUILD_PATH)
from control_module import ControlAPI

def calc_straight_line_from_vehicle(x_v, y_v, psi_v, ds, L):
    x_end = x_v + np.cos(psi_v) * L
    y_end = y_v + np.sin(psi_v) * L
    unit_vec = np.array([x_end - x_v, y_end - y_v]) / L
    s = np.arange(0, L, ds)
    p0 = np.array([x_v, y_v])
    p = [p0 + s_i * unit_vec for s_i in s]
    p = np.array(p)
    return p, s


def project_path(path_1, path_2):
    "project path_2 on path_1 return the indices in path_2 that are the nearest to points in path_1"
    idx = []
    for point in path_1:
        dx = [point[0] - x for x in path_2[:, 0]]
        dy = [point[1] - y for y in path_2[:, 1]]
        idx.append(int(np.argmin(np.hypot(dx, dy))))
    return idx


def fuze_paths(path_1, path_2):
    "fuzed path should start from path_1 and converge to path_2"
    idx = project_path(path_1, path_2) #indices in path_2 that are the nearest to points in path_1
    projected_path = [path_2[j] for j in idx]
    n = len(idx)
    alpha = np.linspace(0,1, n)
    fuzed = []
    for i in range(n):
        fuzed.append((1.0 - alpha[i]) * path_1[i] + alpha[i] * projected_path[i])
    fuzed = np.array(fuzed)
    fuzed = np.vstack([fuzed, path_2[idx[-1] + 1:]])
    return fuzed


def motion_planning_emulation(desired_path, desired_path_s, x_v, y_v, psi_v, ds, L, horizon, plot_result = False):
    # fuze paths in ragne [0,L]
    idx_for_car_on_path = Functions.project_point_on_path(point=[x_v, y_v], path=desired_path)
    idx_for_L = np.argmin(
            abs(desired_path_s - (desired_path_s[idx_for_car_on_path] + L)))
    path_temp, _ = calc_straight_line_from_vehicle(x_v, y_v, psi_v, ds, L)
    path_desired = np.array([desired_path[idx_for_car_on_path:idx_for_L, 0], desired_path[idx_for_car_on_path:idx_for_L, 1]]).T
    fuzed_path = fuze_paths(path_temp, path_desired)
    # calc path in range [L,horizon]
    idx_for_horizon = np.argmin(abs(desired_path_s - (desired_path_s[idx_for_car_on_path] + horizon)))
    path_L_to_horizon = np.array([desired_path[idx_for_L:idx_for_horizon, 0], desired_path[idx_for_L:idx_for_horizon, 1]]).T
    fuzed_path = np.vstack([fuzed_path,path_L_to_horizon])
    fuzed_spline_x, fuzed_spline_y, fuzed_spline_psi, _, fuzed_s = cs.calc_spline_course(fuzed_path[:, 0],
                                                                                              fuzed_path[:, 1], ds=ds)
    if plot_result:
        plt.figure()
        plt.plot(desired_path[:, 0], desired_path[:, 1])
        plt.plot(path_temp[:, 0], path_temp[:, 1], color='black', linestyle='--')
        plt.plot(fuzed_path[:, 0], fuzed_path[:, 1])
        plt.grid(True)
        plt.show()
    return fuzed_spline_x, fuzed_spline_y, fuzed_spline_psi, fuzed_s


def run_system_simulation(simulation_params, desired_traj, velocity_profile_KPH, odometer_signal, heading_measurement_errors, delay_fluctuations):
    def observation_model():
        # calculate a horizon of GT trajectory in nav frame
        motion_planning_path_x, motion_planning_path_y, _, motion_planning_path_s = motion_planning_emulation(
            desired_traj, desired_traj_s, vehicle_obj.x, vehicle_obj.y, vehicle_obj.psi,
            simulation_params["motion_planing_path_spacing"],
            simulation_params['L_param_for_motion_planning_emulation'],
            simulation_params['motion_planning_horizon'],
            plot_result=False)
        cond4 = motion_planning_path_s.shape[0] <= 5  # check there's at least 5 points or end simulation
        if cond4:
            print('condition 4 is met')
            observation_is_valid = False
            desired_traj_sparse_nav_delayed = np.array([[0, 0]])
            reference_traj_sparse_ego_delayed = np.array([[0, 0]])
        else:
            # observation model ref_traj_nav -> sparse, finite horizon-> transform to EGO - > delay
            desired_traj_sparse_nav = np.vstack([motion_planning_path_x, motion_planning_path_y]).T
            reference_traj_nav_delay_obj.update(t[i], desired_traj_sparse_nav)
            delay_i = max(simulation_params['time_delay'] + delay_fluctuations[i], 0) # varify delay non negative
            desired_traj_sparse_nav_delayed = reference_traj_nav_delay_obj.get_delayed_value(delayed_clock=t[i] - delay_i)  # used only for animation
            # desired path observation in ego frame
            desired_traj_sparse_ego, trans_nav2ego = Functions.project_points_2D(vehicle_obj.x, vehicle_obj.y,
                                                                                 vehicle_obj.psi,
                                                                                 desired_traj_sparse_nav)
            # delay the motion planning observation
            reference_traj_ego_delay_obj.update(t[i], desired_traj_sparse_ego)
            reference_traj_sparse_ego_delayed = reference_traj_ego_delay_obj.get_delayed_value(delayed_clock=t[i] - delay_i)
            observation_is_valid = np.linalg.norm(
                reference_traj_sparse_ego_delayed[-1] - reference_traj_sparse_ego_delayed[0]) > 2.0
        return cond4, observation_is_valid, desired_traj_sparse_nav_delayed, reference_traj_sparse_ego_delayed

    def path_processing():
        # calculate affine transformation using estimated vehicle state
        # these matrices are used only for animation not part of the algo
        Phat = cpp_control_obj.GetPosition()
        psi_hat = cpp_control_obj.GetVehicleHeading()
        T_nav_est_2_ego = Functions.affine_transformation_matrix_2D(Phat[0],
                                                                    Phat[1],
                                                                    psi_hat)
        T_ego_2_nav_est = Functions.inv_affine_transformation_matrix_2D(T_nav_est_2_ego)
        # fix matrix to GT nav frame used for visualization
        T_nav_est_2_nav_at_MP_time = Functions.nav_est_2_nav_affine_transformation_2D(Phat[0],
                                                                                      Phat[1],
                                                                                      psi_hat,
                                                                                      vehicle_obj.x, vehicle_obj.y,
                                                                                      vehicle_obj.psi)
        localization_delay_obj.update(t[i], deepcopy(localization_obj))
        localization_obj_delayed = localization_delay_obj.get_delayed_value(delayed_clock=t[i] - simulation_params['time_delay'])  # used only for animation
        # transform observation to the nav_est frame using delayed localization
        T_nav_est_2_ego_delayed = Functions.affine_transformation_matrix_2D(localization_obj_delayed.pos[0],
                                                                            localization_obj_delayed.pos[1],
                                                                            localization_obj_delayed.psi)
        T_ego_2_nav_est_delayed = Functions.inv_affine_transformation_matrix_2D(T_nav_est_2_ego_delayed)
        if simulation_params['compensate_for_delay']:
            desired_traj_sparse_nav_est = Functions.project_points_2D(T_ego_2_nav_est_delayed,
                                                                      reference_traj_sparse_ego_delayed)
        else:
            desired_traj_sparse_nav_est = Functions.project_points_2D(T_ego_2_nav_est,
                                                                      reference_traj_sparse_ego_delayed)
        desired_traj_smooth_nav_est_x, desired_traj_smooth_nav_est_y, desired_traj_smooth_nav_est_psi, _, _ = (
            cs.calc_spline_course(desired_traj_sparse_nav_est[:, 0],
                                  desired_traj_sparse_nav_est[:, 1],
                                  ds=simulation_params["control_interp_spacing"]))
        return (T_nav_est_2_ego, T_nav_est_2_nav_at_MP_time,
                desired_traj_smooth_nav_est_x, desired_traj_smooth_nav_est_y, desired_traj_smooth_nav_est_psi)

    with open('vehicle_config.json', "r") as f:
        vehicle_params = json.loads(f.read())
    desired_traj_x, desired_traj_y, desired_traj_psi, desired_traj_k, desired_traj_s = desired_traj
    desired_traj = np.vstack([desired_traj_x, desired_traj_y]).T
    traj_length = desired_traj_s[-1]
    # create vehicle agent
    error_x = 0.0
    error_y = 2.0
    if simulation_params['model'] == 'Dynamic':
        vehicle_obj = VehicleDynamicModel(x=desired_traj_x[0] + error_x, y=desired_traj_y[0] + error_y, psi=desired_traj_psi[0],
                                          vehicle_params=vehicle_params, simulation_params=simulation_params,
                                          steering_uncertainty_factor=1.0, lr_uncertainty_factor=1.0, WB_uncertainty_factor=1.0,
                                          m_uncertainty_factor=1.0, I_uncertainty_factor=1.0, C_uncertainty_factor=1.0)
    elif simulation_params['model'] == 'Kinematic':
        vehicle_obj = VehicleKinemaicModel(vehicle_params=vehicle_params, simulation_params=simulation_params,
                                           steering_uncertainty_factor=1.0, lr_uncertainty_factor=1.0, WB_uncertainty_factor=1.0)
    else:
        raise 'invalid model type'
    # initialize control
    # t = np.arange(0, simulation_params['t_end'], simulation_params['dt'])
    """ attention: variable t is coming from the main function variable scope."""
    cpp_control_obj = ControlAPI(float(t[0]))
    cpp_control_obj.ResetVehicleState(t[0], [vehicle_obj.x, vehicle_obj.y, vehicle_obj.psi, vehicle_obj.vx])
    vehicle_obj.vx = 0.0
    """localization_obj is to be erased take localization solution from the cpp obj"""
    localization_obj = ShortTermLocalization()
    localization_obj.nominal_dt = simulation_params['dt']
    localization_obj.WB = vehicle_params['WB']
    localization_obj.reset_position(clock=t[0], state=[vehicle_obj.x, vehicle_obj.y, vehicle_obj.psi, 0.0])

    desired_traj_ego_frame, Trans = Functions.project_points_2D(vehicle_obj.x, vehicle_obj.y, vehicle_obj.psi, desired_traj)
    desired_psi_ego_frame = list(np.array(desired_traj_psi))# - psi_0)
    """ SC is to be erased take steering solution from the cpp obj"""
    # stanly gain
    Ks = 1.0
    SC = StanleyController(Ks=Ks, desired_traj_x=desired_traj_ego_frame[:, 0],
                           desired_traj_y=desired_traj_ego_frame[:, 1],
                           desired_traj_psi=desired_psi_ego_frame)
    # variable to keep
    x = []
    y = []
    psi = []
    x.append(vehicle_obj.x)
    y.append(vehicle_obj.y)
    psi.append(vehicle_obj.psi)
    P_est_ego = copy(localization_obj.pos)
    Trans_ego2nav = Functions.inv_affine_transformation_matrix_2D(Trans)
    P_est_ego_homo = np.hstack([P_est_ego, 1]).reshape([3,1])
    P_est = [Trans_ego2nav.dot(P_est_ego_homo)[:2].T.squeeze()]
    P_est = [copy(localization_obj.pos)]
    front_slip_angle = []
    rear_slip_angle = []
    delta = []
    ef = []
    curvature = []
    stop_condition = False
    i = 0
    if simulation_params['animate']:
        if simulation_params['animation_mode'] == 'GT_navigation_frame':
            plotter_obj = SteeringControlPlotter(car_params=vehicle_params,
                                                 desired_traj_x=desired_traj_x,
                                                 desired_traj_y=desired_traj_y,
                                                 flip_xy=True)
        else:
            plotter_obj = SteeringControlPlotter(car_params=vehicle_params,
                                                 flip_xy=True)
    animation_dt = 0.1
    motion_planning_dt = 0.1
    ndt_animation = int(animation_dt / simulation_params['dt'])
    ndt_motion_planning = int(motion_planning_dt / simulation_params['dt'])
    """maybe fix the pbar with the completed path length"""
    pbar = tqdm(total=100)
    # initialize delay objects
    traj_0 = np.zeros([50, 2])
    reference_traj_ego_delay_obj = Delay(max_time_delay=1, nominal_dt=simulation_params['dt'],
                                           default_value=traj_0)
    reference_traj_nav_delay_obj = Delay(max_time_delay=1, nominal_dt=simulation_params['dt'],
                                           default_value=traj_0)
    localization_delay_obj = Delay(max_time_delay=1, nominal_dt=simulation_params['dt'],
                                           default_value=copy(localization_obj))
    # simulation starts here
    while not stop_condition:
        if np.mod(i, ndt_motion_planning) == 0: # motion planning observation and path processing
            cond4, observation_is_valid, desired_traj_sparse_nav_delayed, reference_traj_sparse_ego_delayed = \
                observation_model()
            if not cond4:  # there's at least 5 points or end simulation
                """ comment 
                this is the motion planning observation: sparse, delayed and in the ego frame
                from here is the algorithm: 
                    1. transform observation to the nav_est frame using delayed localization  
                    2. smoothen using splines interpolation 
                    3. track using the localization feedback
                """
                if observation_is_valid:
                    """
                    this stage is to be implemented in CPP. for now we disable the splines interpolation in cpp and 
                    insert the path in the navigation frame. 
                    test that it works correctly and then start implementing the delay compensation in CPP. 
                    action items:
                    1. connect the python processed paths in nav frame wo interpolation. verify that is correct.
                    2. connect the unprocessed paths.
                    3. start working on the cpp implementation.
                    """
                    (T_nav_est_2_ego, T_nav_est_2_nav_at_MP_time,
                     desired_traj_smooth_nav_est_x, desired_traj_smooth_nav_est_y,
                     desired_traj_smooth_nav_est_psi) = path_processing()
                    #### python controller
                    SC.desired_traj_x = desired_traj_smooth_nav_est_x
                    SC.desired_traj_y = desired_traj_smooth_nav_est_y
                    SC.desired_traj_psi = desired_traj_smooth_nav_est_psi
                    # C++ controller motion planning observation
                    motion_planning_y_axis_direction = -1.0  #
                    use_python_processed_path = False
                    if use_python_processed_path:
                        cpp_control_obj.MotionPlanningUpdate(list(desired_traj_smooth_nav_est_x),
                                                             list(motion_planning_y_axis_direction *
                                                                  desired_traj_smooth_nav_est_y),
                                                             t[i],
                                                             "NAV",
                                                             0.0)
                    else:
                        cpp_control_obj.MotionPlanningUpdate(list(reference_traj_sparse_ego_delayed[:, 0]),
                                                             list(motion_planning_y_axis_direction *
                                                                  reference_traj_sparse_ego_delayed[:, 1]),
                                                             t[i],
                                                             "EGO",
                                                             simulation_params['time_delay'])
                    control_ref_traj = cpp_control_obj.GetReferenceTrajectory()
                    # control_ref_traj_nav_est = np.vstack([SC.desired_traj_x, SC.desired_traj_y]).T
                    control_ref_traj_nav_est = np.vstack([control_ref_traj[0], control_ref_traj[1]]).T
                    control_ref_traj_fixed_to_nav = Functions.project_points_2D(T_nav_est_2_nav_at_MP_time,
                                                                                control_ref_traj_nav_est)  # used only for ploting on GT frame
                    if i == 0:
                        delta_i = 0.0
                    else:
                        delta_i = cpp_control_obj.GetDelta()
                    if simulation_params["motion_planning_debug"]:  # and reference_path_is_valid:
                        plt.figure("motion_planning_debug", figsize=(20, 10))
                        h1 = plt.subplot(121)
                        h1.set_title('Navigation Frame t = ' + str(t[i]) + ' [sec]')
                        # nav frame
                        plt.plot(desired_traj[:, 0], desired_traj[:, 1], linewidth=2, color='gray', linestyle='--',
                                 label='GT')
                        # plt.scatter(desired_traj_sparse_nav_delayed[:, 0], desired_traj_sparse_nav_delayed[:, 1], s=10, color='red', label='observation')
                        plt.scatter(control_ref_traj_fixed_to_nav[:, 0], control_ref_traj_fixed_to_nav[:, 1], s=5,
                                    color='blue',
                                    label='(fixed) smooth reference path, $T^n_{\hat{n}} \cdot T^{\hat{n}}_{EGO} \cdot Path^{EGO}$')
                        Functions.draw_car(vehicle_obj.x, vehicle_obj.y, vehicle_obj.psi, steer=delta_i,
                                           car_params=vehicle_params, ax=plt.gca())
                        plt.grid(True), plt.gca().axis('equal'), plt.legend()
                        h1.set_xlim(vehicle_obj.x - 20, vehicle_obj.x + 20)
                        h1.set_ylim(vehicle_obj.y - 20, vehicle_obj.y + 20)

                        control_ref_traj_ego = Functions.project_points_2D(T_nav_est_2_ego, control_ref_traj_nav_est)
                        h2 = plt.subplot(122)
                        h2.set_title('EGO Frame')
                        desired_traj_ego, _ = Functions.project_points_2D(vehicle_obj.x, vehicle_obj.y,
                                                                          vehicle_obj.psi, desired_traj)  # GT
                        plt.plot(desired_traj_ego[:, 0], desired_traj_ego[:, 1], linestyle='--', linewidth=2.0,
                                 color='gray', label='reference')
                        plt.scatter(reference_traj_sparse_ego_delayed[:, 0], reference_traj_sparse_ego_delayed[:, 1],
                                    s=10, color='red',
                                    label='observation (without considering delay)')
                        plt.plot(control_ref_traj_ego[:, 0], control_ref_traj_ego[:, 1],
                                 label='processed path in ego frame (considering delay)')
                        Functions.draw_car(0, 0, 0, steer=delta_i, car_params=vehicle_params, ax=plt.gca())
                        plt.grid(True), plt.legend()
                        plt.gca().set_xlim(- 20, + 30)
                        plt.gca().set_ylim(- 20, + 20)
                        # plt.gca().axis('equal')
                        plt.show()
        ti = t[i]
        vehicle_state_i = [localization_obj.pos[0], localization_obj.pos[1], localization_obj.psi, localization_obj.speed]
        traj_idx = Functions.project_point_on_path([vehicle_obj.x, vehicle_obj.y], desired_traj)
        curvature.append(desired_traj_k[traj_idx])
        """
        this block of code is replaced by the cpp module
        """
        
        if observation_is_valid:
            steering_update_success = cpp_control_obj.CalculateSteeringCommand(t[i])
            # assert steering_update_success
            vehicle_obj.update(a=0, delta=cpp_control_obj.GetDelta())
            vehicle_obj.vx = velocity_profile_KPH[i] / 3.6
            odometer_i = odometer_signal[i]
            # localization object is propagated using the same eq as vehicle equations (front axle configuration with delta)
            delta_i = max(cpp_control_obj.GetDelta(), -vehicle_params['MAX_STEER'])
            delta_i = min(delta_i, vehicle_params['MAX_STEER'])
        else:
            delta_i = 0.0
            odometer_i = 0.0
        steering_wheel_angle = cpp_control_obj.GetDelta() * vehicle_params["steering_ratio"]
        cpp_control_obj.UpdateSteeringWheel(steering_wheel_angle, t[i])
        cpp_control_obj.UpdateSpeed(odometer_i / 3.6, t[i])
        cpp_control_obj.UpdateHeading(vehicle_obj.psi + heading_measurement_errors[i], t[i])
        localization_obj.update_front_axle_position(t[i], delta_i)
        if vehicle_obj.vx > 0:
            localization_obj.update_speed(odometer_signal[i] / 3.6)
        # localization_obj.psi = vehicle_obj.psi + heading_measurement_errors[i]
        # localization_obj.update_heading(vehicle_obj.psi + heading_measurement_errors[i])# - psi_0)


        """
        steering_update_success = cpp_control_obj.CalculateSteeringCommand(t[i])
        # assert steering_update_success
        vehicle_obj.update(a=0, delta=cpp_control_obj.GetDelta())
        vehicle_obj.vx = actual_speed[i] / 3.6
        # localization object is propagated using the same eq as vehicle equations (front axle configuration with delta)
        delta_i = max(cpp_control_obj.GetDelta(), -vehicle_params['MAX_STEER'])
        delta_i = min(delta_i, vehicle_params['MAX_STEER'])
        steering_wheel_angle = cpp_control_obj.GetDelta() * vehicle_params["steering_ratio"]
        cpp_control_obj.UpdateSteeringWheel(steering_wheel_angle, t[i])
        cpp_control_obj.UpdateSpeed(odometer_signal[i] / 3.6, t[i])
        cpp_control_obj.UpdateHeading(vehicle_obj.psi + heading_measurement_errors[i], t[i])
        """
        P_est.append(cpp_control_obj.GetPosition())
        # P_est.append(copy(localization_obj.pos))
        # store values
        x.append(vehicle_obj.x)
        y.append(vehicle_obj.y)
        if simulation_params['model'] == 'Dynamic':
            front_slip_angle.append(vehicle_obj.alpha_f)
            rear_slip_angle.append(vehicle_obj.alpha_r)
        idx = Functions.project_point_on_path(point=[vehicle_obj.x, vehicle_obj.y], path=desired_traj)
        desired_traj_s_i = desired_traj_s[idx]
        psi.append(vehicle_obj.psi)
        delta.append(delta_i)
        ef.append(cpp_control_obj.GetLateralError())
        cond1 = i >= t.shape[0] - 1
        cond2 = False
        cond3 = desired_traj_s_i >= traj_length * 5.0
        stop_condition = cond1 or cond2 or cond3 or cond4
        if cond1: print('condition 1 is met')
        if cond2: print('condition 2 is met')
        if cond3: print('condition 3 is met')
        i += 1
        if traj_idx > 0:
            # pbar.update((desired_traj_s[traj_idx] - desired_traj_s[traj_idx - 1]) / traj_length * 100)
            pbar.update(desired_traj_s[traj_idx]/ traj_length * 100 - pbar.n)
        if np.mod(i, ndt_animation) == 0 and simulation_params['animate']:
            if simulation_params['animation_mode'] == 'GT_navigation_frame':
                if not observation_is_valid:
                    control_ref_traj_fixed_to_nav = np.array([[0, 0]])
                    print("observation is not valid")
                plotter_obj.update_plot(current_car_position_x=vehicle_obj.x,
                                        current_car_position_y=vehicle_obj.y,
                                        current_car_heading=vehicle_obj.psi,
                                        steering=delta_i,
                                        motion_planning_traj_raw=desired_traj_sparse_nav_delayed,
                                        motion_planning_traj_processed=control_ref_traj_fixed_to_nav,
                                        lateral_error=ef[-1],
                                        t_i=ti,
                                        steering_cmd=delta_i,
                                        heading_error=cpp_control_obj.GetHeadingError(),
                                        heading_reference=cpp_control_obj.GetHeadingReference()
                                        )
            elif simulation_params['animation_mode'] == 'Est_navigation_frame': ##not checked !!!
                Phat = cpp_control_obj.GetPosition()
                psi_hat = cpp_control_obj.GetVehicleHeading()
                Tnav_hat2ego = Functions.affine_transformation_matrix_2D(Phat[0], Phat[1], psi_hat)
                Tego2navhat = Functions.inv_affine_transformation_matrix_2D(Tnav_hat2ego)
                ref_traj_in_nav_hat_frame = np.array(cpp_control_obj.GetReferenceTrajectory()).T
                plotter_obj.update_plot(current_car_position_x=Phat[0],
                                        current_car_position_y=Phat[1],
                                        current_car_heading=psi_hat,
                                        steering=cpp_control_obj.GetDelta(),
                                        motion_planning_traj_raw=ref_traj_in_nav_hat_frame,
                                        motion_planning_traj_processed=ref_traj_in_nav_hat_frame,
                                        lateral_error=cpp_control_obj.GetLateralError(),
                                        t_i=ti)
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

        plt.figure('BEV')
        vehicle_animation_axis = plt.subplot(1, 2, 1)
        plt.title("BEV"), plt.xlabel('[m]'), plt.ylabel('[m]')
        ref_traj_line = vehicle_animation_axis.plot(desired_traj_x, desired_traj_y, color='gray', linewidth=2.0,
                                                    label='desired')
        vehicle_traj_line = vehicle_animation_axis.plot(x, y, linewidth=2.0, color='darkviolet', label='actual')
        P_est = np.array(P_est)
        plt.plot(P_est[:, 0], P_est[:, 1], label='P est')
        plt.legend()
        vehicle_animation_axis.axis("equal")
        vehicle_animation_axis.grid(True)
        lateral_error_axis = plt.subplot(2, 2, 2)

        ef = np.array(ef)
        idx = np.argwhere(ef != None).tolist()
        ef = ef[idx]
        plt.title('lateral error, STD = ' + str("%.2f" % np.std(ef)) + ' [m]')
        lateral_error_axis.plot(t[:i][idx], ef)
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

        # calculate estimated position error statistics on time interval

        time_interval = max(simulation_params['time_delay'], 0.1)
        dt = simulation_params['dt']
        window_size = round(time_interval / dt)
        n = P_est.shape[0]
        dPest_on_time_inerval = P_est[window_size:n] - P_est[:n - window_size]
        x = np.array(x)
        y = np.array(y)
        X_on_time_inerval = (x[window_size:n] - x[:n - window_size]).reshape([n - window_size, 1])
        Y_on_time_inerval = (y[window_size:n] - y[:n - window_size]).reshape([n - window_size, 1])
        dP_on_time_inerval = np.hstack([X_on_time_inerval, Y_on_time_inerval])
        dP_error = dPest_on_time_inerval - dP_on_time_inerval
        dp_error_norm = np.linalg.norm(dP_error, axis=1)
        plt.figure()
        plt.hist(dp_error_norm, bins=100)
        title = 'short term localization error on ' + str(
            simulation_params['time_delay']) + ' [sec] window, \n  mean = ' + \
                str("%.2f" % dp_error_norm.mean()) + ', STD = ' + \
                str("%.2f" % dp_error_norm.std())
        plt.grid(True), plt.ylabel('[m]'), plt.title(title)
        plt.show()
    ef = np.array(ef)
    idx = np.argwhere(ef != None).tolist()
    ef = ef[idx]
    return ef


if __name__ == '__main__':
    scenario = 'straight_line'  # 'sin', 'straight_line', 'square', shiba, random_curvature, eight, ellipse
    desired_traj = Functions.calc_desired_path(scenario)
    simulation_params = {'dt': 0.01, 't_end': 50, 'ego_frame_placement': 'front_axle', 'velocity_KPH': 25, 'time_delay': 0.25,
                         'compensate_for_delay': False,
                         # this field is connected only to python. enabling delay compensation in the cpp module is in control_config.json
                         'L_param_for_motion_planning_emulation': 8,
                         "motion_planning_horizon": 20, "motion_planing_path_spacing": 1.0, "control_interp_spacing": 0.1,
                         'velocity_variations': 0.1, 'heading_drift_noise': 1.0, 'delay_fluctuations': 0.05,
                         'model': 'Dynamic', #'Kinematic', 'Dynamic'
                         'animate': True, 'plot_results': True, 'save_results': False, 'motion_planning_debug': False,
                         'animation_mode': 'GT_navigation_frame'} # Est_navigation_frame, GT_navigation_frame
    t = np.arange(0, simulation_params['t_end'], simulation_params['dt'])
    # measurement errors
    speed_variations = simulation_params['velocity_variations'] * (
                np.random.rand(t.shape[0]) - 0.5).cumsum()  # we add random walk signal to the actual constant velocity
    # create a velocity profile that starts from zero
    if False:
        ideal_speed_profile = Functions.regulate_random_signal(np.zeros(t.shape), simulation_params['velocity_KPH'],
                                                               gain=0.001, plot_res=False)
        actual_speed = ideal_speed_profile + speed_variations
    else:
        actual_speed = np.ones(t.shape) * simulation_params['velocity_KPH'] + speed_variations
    if np.any(actual_speed <= 0):
        velocity_profile = Functions.regulate_random_signal(actual_speed, set_point=simulation_params["velocity_KPH"],
                                                                  gain=0.0002, plot_res=False)
    else:
        velocity_profile = actual_speed
    odometer_measurement = velocity_profile.round()
    heading_measurement_errors = simulation_params['heading_drift_noise'] * np.pi/180 * (np.random.rand(t.shape[0])-0.5).cumsum() # we add random walk signal to the heading as measurement error
    delay_fluctuations = simulation_params['delay_fluctuations'] * (np.random.rand(t.shape[0])-0.5)
    i = 0
    j = 0
    ef = run_system_simulation(simulation_params, desired_traj,
                               velocity_profile,
                               odometer_measurement,
                               heading_measurement_errors,
                               delay_fluctuations)
