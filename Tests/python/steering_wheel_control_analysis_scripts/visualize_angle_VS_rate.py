import pandas as pd
import matplotlib.pyplot as plt
import os
from datetime import datetime
import numpy as np
eps = 25 # dps
def calculate_signal_derivative(u,t):
    du_dt =  np.diff(u) / np.diff(t)
    return du_dt
def sign(u):
    if u < eps and u > -eps:
        y = 0
    elif u > eps:
        y = 1
    else:
        y = -1
    return y
def sign_of_signal(u):
    y = []
    for ui in u:
        y.append(sign(ui))
    return np.array(y)
def align_time_vectors(time_vectors):
    """
    export a common time vector with the smallest dt, maximal t0, minimal tend
    time_vectors: list where each element  is a np.array 1D vector
    """
    n = len(time_vectors)
    dt_list = []
    t0_list = []
    tend_list = []
    for time_vector in time_vectors:
        dt_list.append(np.mean(np.diff(time_vector)))
        t0_list.append(time_vector[0])
        tend_list.append(time_vector[-1])
    dt = min(dt_list)
    t0 = max(t0_list)
    tend = min(tend_list)
    t = np.arange(start=t0,stop=tend,step=dt)
    return t
def estimate_lsb(signal):
    # Calculate the differences between consecutive values
    differences = np.diff(signal)

    # Find the smallest non-zero difference
    non_zero_differences = differences[differences != 0]
    if len(non_zero_differences) == 0:
        return 0  # If all differences are zero, return 0
    lsb = np.min(np.abs(non_zero_differences))

    return lsb
path_angle = "data/Re_ Steering resolution19_12/steering.csv"
print("analyzing " + path_angle)
steering_data = pd.read_csv(path_angle)
path_rate = "data/Re_ Steering resolution19_12/steering_rate.csv"
print("analyzing " + path_angle)
rate_data = pd.read_csv(path_rate)

common_time = align_time_vectors([steering_data.time_stamp.values.tolist(), rate_data.time_stamp.values.tolist()])
steering_interp = np.interp(common_time, steering_data.time_stamp, steering_data.data_value)
rate_interp = np.interp(common_time, rate_data.time_stamp, rate_data.data_value)
common_time -= common_time[0]

plt.figure()
h = plt.subplot(211)
plt.plot(common_time, steering_interp, label="angle")
plt.legend()
plt.grid(True)
plt.subplot(212, sharex=h)
dtheta_dt = np.hstack([0,calculate_signal_derivative(steering_interp, common_time)])
rate_degrees = 4.0*rate_interp * sign_of_signal(dtheta_dt)

plt.plot(common_time, rate_degrees, label="rate")
plt.plot(common_time, dtheta_dt, label=r"$\frac{d\theta}{dt}$")
print("estimated LSB = " + str("%.5f" % estimate_lsb(rate_degrees)) + " [deg]")
plt.legend()
plt.grid(True)
plt.show()
