import argparse
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import Functions
parser = argparse.ArgumentParser()
parser.add_argument('--path_estimated', type=str, required=True)
parser.add_argument('--path_reference', type=str, required=True)
args = parser.parse_args()

data_estimated = pd.read_csv(args.path_estimated)
data_reference = pd.read_csv(args.path_reference)

Functions.PlotEulerAngles(Functions.continuous_angle(data_reference.roll * np.pi / 180),
                          Functions.continuous_angle(data_estimated.phi_hat),
                          Functions.continuous_angle(data_reference.pitch * np.pi / 180),
                          Functions.continuous_angle(data_estimated.theta_hat),
                          Functions.continuous_angle((data_reference.yaw - data_reference.yaw[0]) * np.pi / 180),
                          Functions.continuous_angle(data_estimated.psi_hat - data_estimated.psi_hat[0]),
                          np.array(data_estimated.time_IMU),
                          np.array(data_reference.time_stamp - data_reference.time_stamp[0]))