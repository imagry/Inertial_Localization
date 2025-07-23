import json
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, CheckButtons, Button
from os import path
import glob
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime

vehicle_control_path = '/opt/imagry/aidriver_new/modules/control_api/VehicleControl'
tests_python_path = path.join(vehicle_control_path, "Tests/python")
config_path = path.join(tests_python_path, "longitudinal_plots_config")
debug_data_path = path.join(vehicle_control_path, "data/temp_results")
aidriver_trips_path = "/opt/imagry/trips"

def find_latest_trip(directory):
    pattern = path.join(directory, "*/")
    folders = glob.glob(pattern)
    folders = [f for f in folders if path.basename(path.normpath(f))[-1].isdigit()]
    folders.sort(key=path.getmtime, reverse=True)
    return path.basename(path.normpath(folders[0])) if folders else None

class Plotter:
    def __init__(self, control_logger_trip_name=None,
                 aidriver_trip_name=None,
                 display_aidriver_trip_data=False):
        self.control_logger_trip_name = control_logger_trip_name if control_logger_trip_name is not None else find_latest_trip(debug_data_path)
        self.aidriver_trip_name = aidriver_trip_name if aidriver_trip_name is not None else find_latest_trip(aidriver_trips_path)
        self.display_aidriver = display_aidriver_trip_data
        filename = path.join(debug_data_path, self.control_logger_trip_name, "debug_longitudinal_control.csv")
        self.menu_fig = None  # Initialize menu figure
        self.load_json_files()
        self.load_data_from_csv(filename)
        self.setup_plots()

    def load_json_files(self):
        with open(path.join(config_path, 'default_plots.json')) as f:
            self.default_plots = json.load(f)
        with open(path.join(config_path, 'custom_plots.json')) as f:
            self.additional_plots = json.load(f)
        with open(path.join(config_path, 'vehicle_states.json')) as f:
            self.vehicle_states = json.load(f)
        with open(path.join(config_path, 'controller_states.json')) as f:
            self.controller_states = json.load(f)

    def load_data_from_csv(self, filename):
        data = pd.read_csv(filename)
        data.columns = data.columns.str.replace(r' \[.*\]', '', regex=True)
        self.data = data
        self.min_time = data['time'][1]
        self.max_time = data['time'].max()

        self.vehicle_data = {key: list(zip(data['time'], data[key])) for key in self.vehicle_states.keys() if key in data.columns}
        self.controller_data = {key: list(zip(data['time'], data[key])) for key in self.controller_states.keys() if key in data.columns}

        vehicle_acceleration = data['vehicle_velocity_filtered'].diff() / data['dt']
        vehicle_acceleration = vehicle_acceleration.rolling(window=100, center=True, min_periods=1).mean().fillna(0)
        self.vehicle_data['vehicle_acceleration'] = list(zip(data['time'], vehicle_acceleration))

        vehicle_jerk = vehicle_acceleration.diff() / data['dt']
        vehicle_jerk = vehicle_jerk.rolling(window=100, center=True, min_periods=1).mean().fillna(0)
        self.vehicle_data['vehicle_jerk'] = list(zip(data['time'], vehicle_jerk))

    def setup_plots(self):
        self.fig, self.axs = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
        self.line_styles = ['-', '--', '-.', ':']
        self.fig.text(0.95, 0.95, f'Trip Name (control logger): {self.control_logger_trip_name}\n\nTrip Name (aidriver): {self.aidriver_trip_name if self.display_aidriver else None}', ha='right', va='top', fontsize=8)
        
        # self.slider_ax = self.fig.add_axes([0.1, 0.01, 0.8, 0.03])
        # self.time_slider = Slider(self.slider_ax, 'Time', self.min_time, self.max_time, valinit=self.max_time)
        # self.time_slider.on_changed(self.update_time_selection)

        self.button_ax = self.fig.add_axes([0.1, 0.93, 0.3, 0.05])  
        self.open_menu_button = Button(self.button_ax, 'Open Custom Plots', color='lightblue', hovercolor='deepskyblue')
        self.open_menu_button.on_clicked(self.open_additional_plots_menu)
        # Format the x-axis to show time of day
        # Convert Unix timestamp to matplotlib's internal date format
        # timestamps_num = [mdates.epoch2num(ts) for ts in timestamps]
        # Convert 'time' values to matplotlib's internal date format
        # Convert 'time' values to matplotlib's internal date format (numeric)
        time_values = self.data['time']
        time_values_num = mdates.date2num(time_values)  # Convert datetime to matplotlib's numeric format

        for ax in self.axs:
            ax.plot_date(time_values_num, time_values, '-')  # Plot the time values against the data
            ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))  # Time format (HH:MM:SS)
            ax.xaxis.set_major_locator(mdates.HourLocator(interval=1))  # Hour locator, adjust as needed
        self.update_time_selection(self.max_time)
        plt.show()

    def update_time_selection(self, value):
        for i, (plot_name, plot_info) in enumerate(self.default_plots.items()):
            ax = self.axs[i]
            ax.clear()
            ax.set_title(plot_name)
            ax.set_xlim(self.min_time, value)
            ax.grid(True)

            self.style_index = 0
            for state in plot_info.get('vehicle_states', []):
                if state in self.vehicle_data:
                    filtered_data = [v for v in self.vehicle_data[state] if v[0] <= value]
                    if filtered_data:
                        display_name = self.vehicle_states[state]['display_name']
                        unit = self.vehicle_states[state]['unit']
                        ax.plot([v[0] for v in filtered_data], [v[1] for v in filtered_data],
                                label=f"{display_name} [{unit}]",
                                linestyle=self.line_styles[self.style_index % len(self.line_styles)])
                        self.style_index += 1

            for state in plot_info.get('controller_states', []):
                if state in self.controller_data:
                    filtered_data = [v for v in self.controller_data[state] if v[0] <= value]
                    if filtered_data:
                        display_name = self.controller_states[state]['display_name']
                        unit = self.controller_states[state]['unit']
                        ax.plot([v[0] for v in filtered_data], [v[1] for v in filtered_data],
                                label=f"{display_name} [{unit}]",
                                linestyle=self.line_styles[self.style_index % len(self.line_styles)])
                        self.style_index += 1

            ax.legend(loc='upper left')

        self.fig.canvas.draw_idle()

    def open_additional_plots_menu(self, event):
        if self.menu_fig is not None:
            plt.close(self.menu_fig)

        self.menu_fig, self.menu_ax = plt.subplots(figsize=(8, 5))
        self.menu_ax.set_title('Select Additional Plots')

        self.check = CheckButtons(self.menu_ax, list(self.additional_plots.keys()), [False] * len(self.additional_plots))

        self.confirm_button_ax = self.menu_fig.add_axes([0.1, 0.01, 0.8, 0.05])
        self.confirm_button = Button(self.confirm_button_ax, 'Open Selected Plots')
        self.confirm_button.on_clicked(self.open_selected_plots)

        plt.show()

    def open_selected_plots(self, event):
        selected_plots = self.check.get_status()
        print("Selected plots:", selected_plots)

        for i, (plot_name, plot_info) in enumerate(self.additional_plots.items()):
            if selected_plots[i]:
                print(f"Opening plot: {plot_name}")

                if not isinstance(plot_info, dict) or 'subplots' not in plot_info:
                    print(f"Unexpected structure for {plot_name}: {plot_info}")
                    continue
                
                new_fig, new_axs = plt.subplots(len(plot_info['subplots']), 1, figsize=(8, 6), sharex=self.axs[0])
                new_fig.suptitle(plot_name, fontweight='bold')

                for j, (subplot_title, subplot) in enumerate(plot_info['subplots'].items()):
                    ax = new_axs[j]
                    if not isinstance(subplot, dict):
                        print(f"Unexpected structure for subplot in {plot_name}: {subplot}")
                        continue
                    
                    style_index = 0
                    
                    for state in subplot.get('vehicle_states', []):
                        if state in self.vehicle_data:
                            filtered_data = self.vehicle_data[state]
                            if filtered_data:
                                display_name = self.vehicle_states[state]['display_name']
                                unit = self.vehicle_states[state]['unit']
                                ax.plot([v[0] for v in filtered_data], [v[1] for v in filtered_data],
                                         label=f"{display_name} [{unit}]",
                                         linestyle=self.line_styles[style_index % len(self.line_styles)])
                                style_index += 1

                    for state in subplot.get('controller_states', []):
                        if state in self.controller_data:
                            filtered_data = self.controller_data[state]
                            if filtered_data:
                                display_name = self.controller_states[state]['display_name']
                                unit = self.controller_states[state]['unit']
                                ax.plot([v[0] for v in filtered_data], [v[1] for v in filtered_data],
                                         label=f"{display_name} [{unit}]",
                                         linestyle=self.line_styles[style_index % len(self.line_styles)])
                                style_index += 1

                    ax.legend(loc='upper left')
                    ax.set_title(subplot_title)
                    ax.set_xlim(self.min_time, self.max_time)
                    ax.grid(True)

                    # ax_slider = new_fig.add_axes([0.1, 0.01, 0.8, 0.03])
                    # slider = Slider(ax_slider, 'Time', self.min_time, self.max_time, valinit=self.max_time)

                    # def update(val, ax=ax):
                    #     current_time = slider.val
                    #     ax.set_xlim(self.min_time, current_time)
                    #     new_fig.canvas.draw_idle()

                    # slider.on_changed(lambda val, ax=ax: update(val, ax))
                new_fig.text(0.95, 0.95, f'Trip Name (control logger): {self.control_logger_trip_name}\n\nTrip Name (aidriver): {self.aidriver_trip_name if self.display_aidriver else None}', ha='right', va='top', fontsize=8)
                plt.show()

        plt.close(self.menu_fig)
        self.menu_fig = None

if __name__ == "__main__":
    Plotter(control_logger_trip_name=None,
            aidriver_trip_name=None,
            display_aidriver_trip_data=False)
