import os
import Functions
import pandas as pd
import matplotlib.pyplot as plt
python_results_file_path = '../../data/short_term_localization_results_python.csv'
cpp_results_file_path = '../../data/localization_test_output_cpp.csv'
python_res = pd.read_csv(python_results_file_path).to_dict('list')
cpp_res = pd.read_csv(cpp_results_file_path).to_dict('list')
plt.figure()
plt.plot(python_res['P_est_x'], python_res['P_est_y'], label='python')
plt.plot(cpp_res['P_est_x'], cpp_res['P_est_y'], linestyle='--', label='cpp')
plt.grid(True), plt.legend()
plt.show()
