import numpy as np
np.random.seed(0)
import sys
import os
tests_python_path = "./Tests"
if tests_python_path not in sys.path:
    sys.path.append(tests_python_path)
print("CWD = " + str(os.getcwd()) + "\n")
from Classes import VehicleDynamicModel
import json
BUILD_PATH = './Tests/python/python_binding/build'
if BUILD_PATH not in sys.path:
    sys.path.append(BUILD_PATH)
from control_module import StanleyController

with open('vehicle_config.json', "r") as f:
    vehicle_params = json.loads(f.read())
simulation_params = {'dt': 0.01, 't_end': 5*60, 'ego_frame_placement': 'front_axle', 'velocity_KPH': 10,
                     'animate': True, 'plot_results': True}
vehicle = VehicleDynamicModel(x=0.0, y=1.0, psi=0.0,
                              vehicle_params=vehicle_params, simulation_params=simulation_params,
                              steering_uncertainty_factor=1.0, lr_uncertainty_factor=1.0, WB_uncertainty_factor=1.0,
                              m_uncertainty_factor=1.0, I_uncertainty_factor=1.0, C_uncertainty_factor=1.0)
t = np.arange(0, simulation_params['t_end'], simulation_params['dt'])
vehicle.v = simulation_params['velocity_KPH'] / 3.6
# stanly gain
Ks = 1.0
SC = StanleyController(Ks, [0.0, 1.0, 2.0], [0.0, 0.0, 0.0], 1.0)
SC.calc_steering_command(vehicle.x, vehicle.y, vehicle.psi, vehicle.vx)
print('delta = ' + str(SC.get_delta()))
