import argparse
import pandas as pd
import matplotlib.pyplot as plt
import os
from datetime import datetime
import numpy as np
import cmath
import math

def save_csv(dic, path, print_message=False):
    df = pd.DataFrame(data=dic)
    df.to_csv(path,index=False)
    if print_message:
        print('results saved to ' + path)
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
def segment_signals_according_to_frequency(freq_vec, time, input, output, plot_res=False):
    signals_sorted_according_to_frequency = []
    # keys = ['frequency', 'start_idx', 'stop_idx', "input",  "output"]
    # Create a dictionary with keys and values set to None
    # signals_sorted_according_to_frequency.append({key: None for key in keys})
    signals_sorted_according_to_frequency.append({})
    signals_sorted_according_to_frequency[0]["frequency"] = freq_vec[0]
    signals_sorted_according_to_frequency[0]["start_idx"] = 0
    last_freq = freq_vec[0]
    for idx, f in enumerate(freq_vec):
        if f != last_freq:
            signals_sorted_according_to_frequency[-1]["stop_idx"] = idx - 1
            idx_range = range(signals_sorted_according_to_frequency[-1]["start_idx"],signals_sorted_according_to_frequency[-1]["stop_idx"])
            signals_sorted_according_to_frequency[-1]["time"] = time[idx_range]
            signals_sorted_according_to_frequency[-1]["input"] = input[idx_range]
            t = signals_sorted_according_to_frequency[-1]["time"] - signals_sorted_according_to_frequency[-1]["time"][0]
            input_restored = 0.4 * np.sin(2 * np.pi * last_freq * t)
            signals_sorted_according_to_frequency[-1]["synthetic_input"] = input_restored
            signals_sorted_according_to_frequency[-1]["output"] = output[idx_range]
            signals_sorted_according_to_frequency.append({"frequency": f, "start_idx":idx})
            last_freq = f
            if plot_res:
                plt.figure()
                plt.plot(signals_sorted_according_to_frequency[-1]["time"],
                         signals_sorted_according_to_frequency[-1]["input"])
                plt.plot(signals_sorted_according_to_frequency[-1]["time"],
                         signals_sorted_according_to_frequency[-1]["synthetic_input"])
                plt.plot(signals_sorted_according_to_frequency[-1]["time"],
                         signals_sorted_according_to_frequency[-1]["output"])
                plt.title("frequency = " + str(signals_sorted_according_to_frequency[-1]["frequency"]))
    signals_sorted_according_to_frequency[-1]["stop_idx"] = idx
    idx_range = range(signals_sorted_according_to_frequency[-1]["start_idx"],
                      signals_sorted_according_to_frequency[-1]["stop_idx"])
    signals_sorted_according_to_frequency[-1]["time"] = time[idx_range]
    signals_sorted_according_to_frequency[-1]["input"] = input[idx_range]
    t = signals_sorted_according_to_frequency[-1]["time"] - signals_sorted_according_to_frequency[-1]["time"][0]
    input_restored = 0.4 * np.sin(2 * np.pi * last_freq * t)
    signals_sorted_according_to_frequency[-1]["synthetic_input"] = input_restored
    signals_sorted_according_to_frequency[-1]["output"] = output[idx_range]
    if plot_res:
        plt.figure()
        plt.plot(signals_sorted_according_to_frequency[-1]["time"],
                 signals_sorted_according_to_frequency[-1]["input"])
        plt.plot(signals_sorted_according_to_frequency[-1]["time"],
                 signals_sorted_according_to_frequency[-1]["synthetic_input"])
        plt.plot(signals_sorted_according_to_frequency[-1]["time"],
                 signals_sorted_according_to_frequency[-1]["output"])
        plt.title("frequency = " + str(signals_sorted_according_to_frequency[-1]["frequency"]))
    return signals_sorted_according_to_frequency
def segment_signals_according_to_frequency_map(frequency_map, input, output):
    signal_sorted_according_to_frequency = {}
    signal_sorted_according_to_frequency["frequency"] = []
    signal_sorted_according_to_frequency["input"] = []
    signal_sorted_according_to_frequency["output"] = []
    for index, (frequency, idx) in enumerate(frequency_map.items()):
        signal_sorted_according_to_frequency["frequency"].append(frequency)
        signal_sorted_according_to_frequency["input"].append(input[idx])
        signal_sorted_according_to_frequency["output"].append(frequency)
def search_sorted_vector(t_vec, ti, side):
    insertion_idx = np.searchsorted(t_vec, ti) # where to insert t_i to keep t_vec sorted
    if side=='left':
        idx = insertion_idx - 1
    elif side == 'right':
        idx = insertion_idx
    elif side == 'closest_value':
        idx_left = insertion_idx - 1
        t_left = t_vec[idx_left]
        t_right = t_vec[idx_left + 1]
        if ti - t_left < t_right - ti:
            idx = idx_left
        else:
            idx = idx_left + 1
    else:
        raise "Functions.search_time_vector: side is unrecognised"
    if idx == -1:
        idx = 0
    return idx
def calc_Fourier_ransform(t,s, plot_res=False):
    """
    t is time
    s is signal
    """
    if type(t) is not np.array:
        t = np.array(t)
    dt = np.mean(np.diff(t))
    # Compute the Fourier Transform of the signal
    fft_result = np.fft.fft(s)
    # Compute the frequencies corresponding to the FFT result
    frequencies = np.fft.fftfreq(len(fft_result), dt)
    idx_of_positive_freq = np.argwhere(frequencies > 0).squeeze()
    frequencies = frequencies[idx_of_positive_freq]
    fft_result = fft_result[idx_of_positive_freq]
    fft_result = fft_result / len(frequencies)
    if plot_res:
        # Plot the original signal
        plt.figure(figsize=(12, 6))
        plt.subplot(2, 1, 1)
        plt.plot(t, s)
        plt.title('Original Signal')
        plt.xlabel('Time [s]')
        plt.ylabel('Amplitude')

        # Plot the magnitude of the Fourier Transform
        plt.subplot(2, 1, 2)
        plt.plot(frequencies, np.abs(fft_result))
        plt.title('Fourier Transform')
        plt.xlabel('Frequency [Hz]')
        plt.ylabel('Magnitude')
        # plt.xlim(0, 50)  # Limit the x-axis to show relevant frequencies

        plt.tight_layout()
    return frequencies, fft_result
def continuous_angle(U, units='rad'):
    n = len(U)
    Y = U.copy()
    Counter = 0
    if units == 'deg':
        Cycle = 360
    elif units == 'rad':
        Cycle = 2 * np.pi
    else:
        print("ContinuousAngle:wrong units")
    for i in range(1, n):
        # if abs(U[i] - U[i - 1]) > Cycle / 10:
        #     print(U[i] - U[i - 1])
        if (U[i] - U[i - 1]) > Cycle / 2:
            Counter = Counter - 1
        elif (U[i] - U[i - 1]) < - Cycle / 2:
            Counter = Counter + 1
        Y[i] = U[i] + Cycle * Counter
    return Y
# container_folder = "/opt/imagry/vehicle_response_tests/"
# file_name = get_latest_file_by_name(container_folder)

container_folder = os.getcwd()
file_name = "../../../git/SteeringWheelControlDesign/data/sin_test_3_12.csv"
path = os.path.join(container_folder, file_name)
print("analyzing " + path)
data = pd.read_csv(path)
frequency_vector = np.array(data.target_brake)
input = np.array(data.target_steering)
output = np.array(data.steering)
t0 = data.time_stamp[0]
time = np.array(data.time_stamp) - t0
annotated_signals = segment_signals_according_to_frequency(frequency_vector, time, input, output)
plant = {"freq": [], "magnitude": [], "phase": [], "frequency_response": []}
for idx, item in enumerate(annotated_signals):
    freq_u, u_f = calc_Fourier_ransform(item["time"], item["synthetic_input"], False)
    freq_idx = search_sorted_vector(freq_u, item["frequency"], "closest_value")
    annotated_signals[idx]["u_complex"] = u_f[freq_idx]
    frequency_y, y_f = calc_Fourier_ransform(item["time"], item["output"], False)
    freq_idx = search_sorted_vector(frequency_y, item["frequency"], "closest_value")
    annotated_signals[idx]["y_complex"] = y_f[freq_idx]
    annotated_signals[idx]["plant_complex"] = item["y_complex"] / item["u_complex"]
    annotated_signals[idx]["plant_gain"] = 20 * np.log10(np.abs(item["plant_complex"]))
    annotated_signals[idx]["plant_phase"] = math.degrees(cmath.phase(item["plant_complex"]))
    plant["freq"].append(item["frequency"])
    plant["magnitude"].append(item["plant_gain"])
    plant["phase"].append(item["plant_phase"])
    plant["frequency_response"].append(item["plant_complex"])

plant["phase"] =  continuous_angle(plant["phase"], units='deg')
save_csv(plant, "../../../git/SteeringWheelControlDesign/data/plant.csv", True)


plt.figure('plant')
magnitude_ax = plt.subplot(2,1,1)
plt.plot(plant["freq"], plant["magnitude"])
plt.grid(True)
plt.ylabel("gain [db]")
# plt.yscale('log')
phase_ax = plt.subplot(2,1,2, sharex=magnitude_ax)
plt.plot(plant["freq"], plant["phase"])
plt.axhline(y=-180, color='k', linestyle='--')
plt.ylabel("phase [deg]")
plt.grid(True)
plt.show()