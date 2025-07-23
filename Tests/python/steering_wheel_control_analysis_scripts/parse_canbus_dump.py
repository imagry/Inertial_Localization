import pandas as pd
import matplotlib.pyplot as plt
import os
from datetime import datetime
import numpy as np


def estimate_lsb(signal):
    # Calculate the differences between consecutive values
    differences = np.diff(signal)

    # Find the smallest non-zero difference
    non_zero_differences = differences[differences != 0]
    if len(non_zero_differences) == 0:
        return 0  # If all differences are zero, return 0
    lsb = np.min(np.abs(non_zero_differences))

    return lsb
def parse_line(line: str):
    line = line.split('\n')[0]
    items = line.split(' ')
    # Remove empty strings using a list comprehension
    items = [s for s in items if s]
    id = items[1]
    ch = []
    for item in items[3:8]:
        ch.append(int(item, 16))
    decimal_value = int(items[4] + items[3], 16)
    # Calculate the maximum positive value for the given bit length
    bit_length = 16
    max_positive = 2 ** (bit_length - 1) - 1
    # Calculate the minimum negative value for the given bit length
    min_negative = -2 ** (bit_length - 1)

    # Check if the value is greater than the max positive value
    if decimal_value > max_positive:
        # Convert to negative using two's complement
        decimal_value -= 2 ** bit_length

    ch.append(decimal_value/10)
    return np.array(ch)
path = "data/can files 2b0 can1/new1.txt"
print("analyzing " + path)
with open(path, "r") as f:
    lines = f.readlines()
canbus_channels = None
for line in lines:
    if canbus_channels is None:
        canbus_channels = parse_line(line)
    else:
        canbus_channels = np.vstack([canbus_channels, parse_line(line)])
plt.figure()
h = plt.subplot(511)
plt.plot(canbus_channels[:,0]), plt.grid(True)
plt.subplot(512, sharex=h)
plt.plot(canbus_channels[:,1]), plt.grid(True)
plt.subplot(513, sharex=h)
plt.plot(canbus_channels[:,2]), plt.grid(True)
plt.subplot(514, sharex=h)
plt.plot(canbus_channels[:,3]), plt.grid(True)
plt.subplot(515, sharex=h)
plt.plot(canbus_channels[:,4]), plt.grid(True)

plt.figure()
plt.subplot(111, sharex=h)
plt.plot(canbus_channels[:,5]), plt.grid(True)
print("estimated LSB = " + str(estimate_lsb(canbus_channels[:,5])) )
plt.show()

