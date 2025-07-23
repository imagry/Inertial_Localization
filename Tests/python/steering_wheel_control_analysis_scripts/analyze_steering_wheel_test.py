import argparse
import pandas as pd
import matplotlib.pyplot as plt
import os
parser = argparse.ArgumentParser()
parser.add_argument('--path', type=str, required=True)
args = parser.parse_args()
print("analyzing " + args.path)
path, file_name = os.path.split(args.path)
file_type = file_name.split(sep='.')[1]
if file_type == 'csv':
    data = pd.read_csv(args.path)
elif file_type == 'xlsx':
    data = pd.read_excel(args.path)
else:
    raise 'unsupported file type'

plt.figure()
plt.plot(data.time_stamp - data.time_stamp[0], data.target_steering, label="target_steering")
plt.plot(data.time_stamp - data.time_stamp[0], data.steering, label="steering")
plt.legend()
plt.grid(True)
plt.show()
