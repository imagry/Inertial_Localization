import argparse
import pandas as pd
import matplotlib.pyplot as plt
import os
from datetime import datetime
import numpy as np
import cmath
import math
import control
import pandas

plant = pandas.read_csv('../../../git/SteeringWheelControlDesign/data/plant.csv')
######## system identification ##########
# Define the Laplace variable s
s = control.TransferFunction.s
# Define a transfer function
controller = {}
controller["transfer_function"] = 0.02 * 1 / s
controller["freq"] = np.array(plant.freq)
mag, phase, omega = control.bode(controller["transfer_function"], 2 * np.pi * controller["freq"], plot=False)
controller["phase"] = np.degrees(phase)
controller["magnitude_db"] = 20 * np.log10(mag)

open_loop = {}
open_loop["freq"] = np.array(plant.freq)
open_loop["phase"] = controller["phase"] + plant.phase
open_loop["magnitude_db"] = controller["magnitude_db"] + plant.magnitude



plt.figure('plant')
magnitude_ax = plt.subplot(2,1,1)
plt.plot(plant.freq, plant.magnitude, label="plant")
plt.plot(controller["freq"], controller["magnitude_db"], label="C")
plt.plot(open_loop["freq"], open_loop["magnitude_db"], label="open_loop")
plt.legend()
plt.grid(True)
plt.ylabel("gain [db]")
phase_ax = plt.subplot(2,1,2, sharex=magnitude_ax)
plt.plot(plant.freq, plant.phase)
plt.plot(controller["freq"], controller["phase"], label="C")
plt.plot(open_loop["freq"], open_loop["phase"], label="open_loop")
plt.axhline(y=-180, color='k', linestyle='--')
plt.ylabel("phase [deg]")
plt.grid(True)
plt.show()
