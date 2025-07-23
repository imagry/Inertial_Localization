import os
import Functions
import pandas as pd
import matplotlib.pyplot as plt
python_results_file_path = '../../data/control_states.csv'
cpp_results_file_path = '../../data/CPP_results.csv'
python_res = pd.read_csv(python_results_file_path).to_dict('list')
cpp_res = pd.read_csv(cpp_results_file_path).to_dict('list')
plt.figure()
plt.subplot(211)
plt.title('delta')
plt.plot(python_res['delta'], label='python')
plt.plot(cpp_res['delta'], label='cpp')
plt.grid(True), plt.legend()
plt.subplot(212)
plt.title('ef')
plt.plot(python_res['ef'], label='python')
plt.plot(cpp_res['ef'], label='cpp')
plt.grid(True), plt.legend()

plt.show()

