import sys
import os
tests_python_path = "./Tests/python"
if tests_python_path not in sys.path:
    sys.path.append(tests_python_path)
BUILD_PATH = './Tests/python/python_binding/build'
if BUILD_PATH not in sys.path:
    sys.path.append(BUILD_PATH)
simulation_data_path = os.path.abspath("./Tests/python/longitudinal_simulation_data")
velocity_profiles_path = os.path.abspath("./Tests/velocity_profiles")

sys.path.append('/opt/imagry/VehicleLongitudinalControlSimulation')
import controllers
import ev_model
import trajectories
import simulator_for_cpp_testing as simulator
import numpy as np
from tqdm import tqdm

from control_module import ControlAPI
from pandas import read_csv

if __name__ == '__main__':
    # Create CPP ControlAPI instance
    cpp_control_obj = ControlAPI(float(0.0))
    cpp_control_obj.ResetVehicleState(0.0, [0.0, 0.0, 0.0, 0.0])
    fs_slow = 1e2  # Sampling rate for slow components [Hz]
    fs_fast = 1e4  # Sampling rate for fast components [Hz]
    duration = 50  # Simulation maximal duration [s]
    controller = controllers.PIDBasedLongitudinalControllerWrapper(
        cpp_control_obj=cpp_control_obj
    )
    model = ev_model.ElectricVehicle()
    path_length = 1500  # [m]
    ref_trajectory = trajectories.SyntheticTrajectory(path_length=path_length,
                                                      time_length=duration,
                                                      dx=5.0,
                                                      dt=1 / fs_slow,
                                                      min_angle=0.0,
                                                      max_angle=5 * np.pi / 180,
                                                      min_velocity=0/3.6,
                                                      max_velocity=50/3.6,
                                                      angle_profile_mode='zero',
                                                      velocity_profile_mode='inverse_step',
                                                      velocity_filter=False)    # Filter inside controller
    ref_trajectory.save_velocity_profile_to_csv(folder_path=velocity_profiles_path,
                                                profile_name="blah")
    # real_data = read_csv('./dynamics/data/zickel/first_segment.csv', index_col=0)
    # controller = controllers.RealDataFollower(real_data)
    # ref_trajectory = trajectories.RealTrajectory(real_data)
    # # ref_trajectory.visualize()
    # TODO: check braking distance starting with 50 KMH - checked, 9 meters as expected
    # TODO: add rate limiter with max jerk - which one is it?
    simulation = simulator.Simulation(model=model, controller=controller, ref_trajectory=ref_trajectory,
                                      duration=duration, fs_slow=fs_slow, fs_fast=fs_fast,
                                      disable_brake=False, disable_throttle=False,
                                      initial_velocity=0.0,
                                      measurements_folder=simulation_data_path,
                                      simulation_name='blah')
    simulation.simulate()
    # measurements_folder = os.path.join(simulation_data_path, 'blah')
    # simulation.load_simulation_from_folder(measurements_folder)

    subplots_list = [
        {
            'title': 'Vehicle Velocity',
            'vehicle_state_variables': ['v'],
            'controller_state_variables': [],
            'dc_controller_state_variables': [],
            'brake_controller_state_variables': [],
            'trajectory_state_variables': ['velocity', 'filtered_velocity'],
            'components_dict': {

            }
        },
        # {
        #     'title': 'Motor Speed',
        #     'vehicle_state_variables': [],
        #     'controller_state_variables': [],
        #     'dc_controller_state_variables': [],
        #     'brake_controller_state_variables': [],
        #     'trajectory_state_variables': [],
        #     'components_dict': {
        #         'motor': ['w_m']
        #     }
        # },
        # {
        #     'title': 'Vehicle Acceleration',
        #     'vehicle_state_variables': ['a'],
        #     'controller_state_variables': [],
        #     'dc_controller_state_variables': [],
        #     'brake_controller_state_variables': [],
        #     'trajectory_state_variables': [],
        #     'components_dict': {
        #
        #     }
        # },
        # {
        #     'title': 'Vehicle Position',
        #     'vehicle_state_variables': ['x'],
        #     'controller_state_variables': [],
        #     'dc_controller_state_variables': [],
        #     'brake_controller_state_variables': [],
        #     'trajectory_state_variables': [],
        #     'components_dict': {
        #
        #     }
        # },
        # {
        #     'title': 'Vehicle Pitch',
        #     'vehicle_state_variables': ['alpha'],
        #     'controller_state_variables': [],
        #     'dc_controller_state_variables': [],
        #     'brake_controller_state_variables': [],
        #     'trajectory_state_variables': [],
        #     'components_dict': {
        #
        #     }
        # },
        {
            'title': 'Throttle Command and Throttle Mode',
            'vehicle_state_variables': [],
            'controller_state_variables': ['command_throttle', 'throttle_mode'],
            'dc_controller_state_variables': [],
            'brake_controller_state_variables': [],
            'trajectory_state_variables': [],
            'components_dict': {

            }
        },
        {
            'title': 'Throttle PID Terms',
            'vehicle_state_variables': [],
            'controller_state_variables': ['p_term_throttle', 'i_term_throttle', 'd_term_throttle', 'ii_term_throttle'],
            'dc_controller_state_variables': [],
            'brake_controller_state_variables': [],
            'trajectory_state_variables': [],
            'components_dict': {

            }
        },
        {
            'title': 'Brake Command and Braking Mode',
            'vehicle_state_variables': [],
            'controller_state_variables': ['command_brake', 'brake_mode'],
            'dc_controller_state_variables': [],
            'brake_controller_state_variables': [],
            'trajectory_state_variables': [],
            'components_dict': {

            }
        },
        {
            'title': 'Brake PID Terms',
            'vehicle_state_variables': [],
            'controller_state_variables': ['p_term_brake', 'i_term_brake', 'd_term_brake', 'ii_term_brake'],
            'dc_controller_state_variables': [],
            'brake_controller_state_variables': [],
            'trajectory_state_variables': [],
            'components_dict': {

            }
        },
        # {
        #     'title': 'Throttle PID Terms',
        #     'vehicle_state_variables': [],
        #     'controller_state_variables': ['p_term_throttle', 'i_term_throttle', 'd_term_throttle'],
        #     'dc_controller_state_variables': [],
        #     'brake_controller_state_variables': [],
        #     'trajectory_state_variables': [],
        #     'components_dict': {
        #
        #     }
        # },
        {
            'title': 'Global Error Integrator',
            'vehicle_state_variables': [],
            'controller_state_variables': ['velocity_error_integral_global'],
            'dc_controller_state_variables': [],
            'brake_controller_state_variables': [],
            'trajectory_state_variables': [],
            'components_dict': {

            }
        },
        {
            'title': 'Velocity Error',
            'vehicle_state_variables': [],
            'controller_state_variables': ['velocity_error'],
            'dc_controller_state_variables': [],
            'brake_controller_state_variables': [],
            'trajectory_state_variables': [],
            'components_dict': {

            }
        },
        # {
        #     'title': 'Velocity Error Derivative',
        #     'vehicle_state_variables': [],
        #     'controller_state_variables': ['velocity_error_derivative'],
        #     'dc_controller_state_variables': [],
        #     'brake_controller_state_variables': [],
        #     'trajectory_state_variables': [],
        #     'components_dict': {
        #
        #     }
        # },
        # {
        #     'title': 'Error in Velocity',
        #     'vehicle_state_variables': [],
        #     'controller_state_variables': ['velocity_error'],
        #     'dc_controller_state_variables': [],
        #     'brake_controller_state_variables': [],
        #     'trajectory_state_variables': [],
        #     'components_dict': {
        
        #     }
        # },
        # {
        #     'title': 'Motor, Wheel and Wheel-load Torques',
        #     'vehicle_state_variables': [],
        #     'controller_state_variables': [],
        #     'dc_controller_state_variables': [],
        #     'brake_controller_state_variables': [],
        #     'trajectory_state_variables': [],
        #     'components_dict': {
        #         'wheel': ['T_load_on_wheel'],
        #         'gear': [],
        #         'load_on_wheel': [],
        #         'motor': ['T_m'],
        #         'spring': ['T_spring_output_total']
        #     }
        # },
        # {
        #     'title': 'Input Voltage vs. Back-emf',
        #     'vehicle_state_variables': [],
        #     'controller_state_variables': [],
        #     'dc_controller_state_variables': [],
        #     'brake_controller_state_variables': [],
        #     'trajectory_state_variables': [],
        #     'components_dict': {
        #         'motor': ['V_a', 'back_emf']
        #     }
        # }
    ]

    simulation.plot(subplots_list)
