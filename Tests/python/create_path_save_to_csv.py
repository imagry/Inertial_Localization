import Functions
import numpy as np

scenario = 'straight_line'  # 'sin', 'straight_line', 'turn', shiba, random_curvature, eight, ellipse
desired_traj = Functions.calc_desired_path(scenario, plot_results=True, ds=1)
desired_traj_x, desired_traj_y, desired_traj_psi, _, _ = desired_traj
timestamps = np.arange(len(desired_traj_x), dtype=float) * 0.01  # Assuming a time step of 0.01 seconds

dic = {'x': desired_traj_x, 'y': desired_traj_y}
Functions.save_csv(dic, path='/opt/imagry/aidriver_new/config/mp_data/example_path.csv', print_message=True)

