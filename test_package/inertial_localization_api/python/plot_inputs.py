import argparse
import pandas as pd
import matplotlib.pyplot as plt
parser = argparse.ArgumentParser()
parser.add_argument('--path', type=str, required=True)
parser.add_argument('--x', nargs='+')
parser.add_argument('--y', nargs='+')
args = parser.parse_args()

data = pd.read_csv(args.path)
assert len(args.x) == len(args.y)
n = len(args.x)
plt.figure()
plt.title(args.path)
for i in range(n):
    plt.plot(data[args.x[i]], data[args.y[i]], label=args.y[i])
# to run code from terminal -> python plot_input.py --path input.csv
plt.grid(True), plt.legend()
plt.axis('equal')
plt.show()
