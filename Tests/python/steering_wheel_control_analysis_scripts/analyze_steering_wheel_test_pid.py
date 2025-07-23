import argparse
import pandas as pd
import matplotlib.pyplot as plt
import os
from datetime import datetime
import numpy as np
def get_latest_file_by_name(path):
    # Initialize the latest date to the smallest possible date
    latest_date = datetime.min
    latest_dir = None

    # Iterate over all items in the given path
    for item in os.listdir(path):
        # Construct the full path of the item
        item_path = os.path.join(path, item)


        try:
            # Try to parse the file name as a date
            item_date = datetime.strptime(item.split('.')[0], '%Y-%m-%dT%H_%M_%S')

            # If this directory's date is later than the latest date we've seen so far
            if item_date > latest_date:
                # Update the latest date and directory
                latest_date = item_date
                latest_dir = item
        except ValueError:
            # If the directory name couldn't be parsed as a date, ignore it
            pass

    # Return the name of the directory with the latest date
    return latest_dir

container_folder = "/opt/imagry/vehicle_response_tests/"
file_name = get_latest_file_by_name(container_folder)
# file_name = "2024-11-28T14_56_33.csv"
path = os.path.join(container_folder, file_name)
print("analyzing " + path)
data = pd.read_csv(path)
steering_torque = np.array(data.steering_torque)
steering_proportional = np.array(data.steering_proportional)
steering_integral = np.array(data.steering_integral)
steering_derivative = np.array(data.steering_derivative)
kp = np.array(data.kp)
ki = np.array(data.ki)
kd = np.array(data.kd)
t0 = data.time_stamp[0]
plt.figure()
h = plt.subplot(421)
plt.plot(data.time_stamp - t0, data.target_steering, label="target_steering")
plt.plot(data.time_stamp - t0, data.steering, label="steering")
plt.legend()
plt.grid(True)
plt.subplot(423, sharex=h)
plt.plot(data.time_stamp - t0,data.steering_proportional, label='steering_proportional')
plt.legend()
plt.grid(True)
plt.subplot(425, sharex=h)
plt.plot(data.time_stamp - t0,data.steering_integral, label='steering_integral')
plt.legend()
plt.grid(True)
plt.subplot(427, sharex=h)
plt.plot(data.time_stamp - t0,data.steering_derivative, label='steering_derivative')
plt.legend()
plt.grid(True)
plt.subplot(122, sharex=h)
plt.plot(data.time_stamp - t0,kp * steering_proportional, label='steering_proportional')
plt.plot(data.time_stamp - t0,ki * steering_integral, label='steering_integral')
plt.plot(data.time_stamp - t0,kd * steering_derivative, label='steering_derivative')
plt.plot(data.time_stamp - t0,data.steering_torque, label='steering_torque', linestyle='--')
plt.legend()
plt.grid(True)
plt.legend()
plt.grid(True)
plt.show()
