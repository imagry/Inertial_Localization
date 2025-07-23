import numpy as np
np.random.seed(0)
import sys
tests_python_path = "./Tests/python"
if tests_python_path not in sys.path:
    sys.path.append(tests_python_path)
from Classes import StanleyController, VehicleDynamicModel
import json

with open('vehicle_config.json', "r") as f:
    vehicle_params = json.loads(f.read())
simulation_params = {'dt': 0.01, 't_end': 5*60, 'ego_frame_placement': 'front_axle', 'velocity_KPH': 10,
                     'animate': True, 'plot_results': True}
vehicle_obj = VehicleDynamicModel(x=0.0, y=1.0, psi=0.0,
                                  vehicle_params=vehicle_params, simulation_params=simulation_params,
                                  steering_uncertainty_factor=1.0, lr_uncertainty_factor=1.0,
                                  WB_uncertainty_factor=1.0,
                                  m_uncertainty_factor=1.0, I_uncertainty_factor=1.0,
                                  C_uncertainty_factor=1.0)
t = np.arange(0, simulation_params['t_end'], simulation_params['dt'])
vehicle_obj.v = simulation_params['velocity_KPH'] / 3.6
# stanly gain
Ks = 0.3
SC = StanleyController(Ks=Ks, desired_traj_x=[0.0, 1.0, 2.0], 
                       desired_traj_y=[0.0, 0.0, 0.0], 
                       desired_traj_psi=[0.0, 0.0, 0.0])

SC.calc_steering_command(vehicle_obj)
print('delta = ' + str(SC.delta))
