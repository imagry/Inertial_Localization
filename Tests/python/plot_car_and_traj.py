import Functions
import matplotlib.pyplot as plt
import json

def convert_string_float_list(string: str):
    strings = string.split(' ')
    strings = [ x for x in strings if len(x)>0]
    return [float(x) for x in strings]

#pasete the print from the teraminal here
print_from_terminal = r"""traj_.x: 0 1.24931 2.44302 3.59129 4.70465 5.7919 6.85991 7.91401 8.95826 9.99574 
traj_.y: -2.42446e-38 0.0611557 0.143211 0.240701 0.34981 0.46794 0.593455 0.725606 0.864478 1.01079 
vehicle_pos_x: 16.5859
vehicle_pos_y: 3.34699
state.psi_: 0.254965
"""
print_from_terminal = print_from_terminal.split('\n')

traj_x = print_from_terminal[0].split('traj_.x: ')[1]
traj_y = print_from_terminal[1].split('traj_.y: ')[1]
traj_x = convert_string_float_list(traj_x)
traj_y = convert_string_float_list(traj_y)
vehicle_pos_x = float(print_from_terminal[2].split('vehicle_pos_x: ')[1])
vehicle_pos_y = float(print_from_terminal[3].split('vehicle_pos_y: ')[1])
psi = float(print_from_terminal[4].split('state.psi_: ')[1])
animation_figure = plt.figure()
vehicle_animation_axis = plt.subplot(1, 1, 1)
ref_traj_line = vehicle_animation_axis.plot(traj_x, traj_y, color='gray', linewidth=2.0)
with open('../../vehicle_config.json', "r") as f:
    vehicle_params = json.loads(f.read())
vehicle_line = Functions.draw_car(vehicle_pos_x, vehicle_pos_y, psi, steer=0, car_params=vehicle_params, ax=vehicle_animation_axis)
vehicle_animation_axis.axis("equal")
vehicle_animation_axis.grid(True)
vehicle_animation_axis.set_xlabel('x [m]')
vehicle_animation_axis.set_ylabel('y [m]')
plt.show()