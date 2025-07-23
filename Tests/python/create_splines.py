import argparse
import pandas as pd
import matplotlib.pyplot as plt
import CurvesGenerator.cubic_spline as cs
import os
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str, required=True)
    parser.add_argument('--plot_res', action='store_true',
                        help='a boolean flag')
    parser.set_defaults(flag=False)
    parser.add_argument('--out_path', type=str, default=None)

    args = parser.parse_args()
    data = pd.read_csv(args.path, index_col=None)
    traj_spline_x, traj_spline_y, traj_spline_psi, traj_spline_cur, _ = cs.calc_spline_course(data.x, data.y ,ds=0.1)
    df = pd.DataFrame(data={'x': traj_spline_x, 'y': traj_spline_y})

    if args.out_path is not None:
        df.to_csv(args.out_path, index=False)
        print( __file__ + ": results saved to " + args.out_path + "\n")
    else:
        df.to_csv('splines_python.csv', index=False)
        print(__file__ + ": results saved to " + os.getcwd() + "/splines_python.csv \n")
    if args.plot_res:
        plt.figure()
        plt.plot(data['x'], data['y'], label='raw data point')
        plt.plot(traj_spline_x, traj_spline_y, label='splines')
        plt.grid(True), plt.legend(), #plt.axis('equal')
        plt.show()
