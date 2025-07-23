import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt

velocity_profiles_path = "/opt/imagry/aidriver_new/modules/control_api/VehicleControl/data/velocity_profiles"

def generate_segmented_profile(speeds, segments, transition_time, initial_speed=0, step=0.01, padding_time=5):
    time_list = []
    speed_list = []

    # Add initial transition to the first speed
    initial_times = np.arange(0, transition_time, step)
    initial_speeds = np.linspace(initial_speed, speeds[0], len(initial_times))
    time_list.extend(initial_times)
    speed_list.extend(initial_speeds)

    # Generate constant-speed segments and transitions
    current_time = initial_times[-1] + step
    for i in range(len(speeds) - 1):
        # Constant-speed segment
        segment_times = np.arange(current_time, current_time + segments[i], step)
        segment_speeds = np.full(len(segment_times), speeds[i])
        time_list.extend(segment_times)
        speed_list.extend(segment_speeds)

        # Transition to next speed
        current_time = segment_times[-1] + step
        transition_times = np.arange(current_time, current_time + transition_time, step)
        transition_speeds = np.linspace(speeds[i], speeds[i + 1], len(transition_times))
        time_list.extend(transition_times)
        speed_list.extend(transition_speeds)

        current_time = transition_times[-1] + step

    # Final constant-speed segment
    segment_times = np.arange(current_time, current_time + segments[-1], step)
    segment_speeds = np.full(len(segment_times), speeds[-1])
    time_list.extend(segment_times)
    speed_list.extend(segment_speeds)

    # Add final transition to zero speed
    final_transition_times = np.arange(segment_times[-1] + step, segment_times[-1] + step + transition_time, step)
    final_transition_speeds = np.linspace(speeds[-1], 0, len(final_transition_times))
    time_list.extend(final_transition_times)
    speed_list.extend(final_transition_speeds)

    # Add padding with zeros at the end
    padding_times = np.arange(final_transition_times[-1] + step, final_transition_times[-1] + step + padding_time, step)
    padding_speeds = np.zeros(len(padding_times))
    time_list.extend(padding_times)
    speed_list.extend(padding_speeds)

    return np.array(time_list), np.array(speed_list)

def generate_sine_profile(dc_level, amplitude, frequency, duration, initial_speed=0, transition_time=2, step=0.01, padding_time=5):
    time_list = []
    speed_list = []

    # Add initial transition to the sine DC level
    initial_times = np.arange(0, transition_time, step)
    initial_speeds = np.linspace(initial_speed, dc_level, len(initial_times))
    time_list.extend(initial_times)
    speed_list.extend(initial_speeds)

    # Generate sine wave profile
    sine_times = np.arange(initial_times[-1] + step, initial_times[-1] + step + duration, step)
    sine_speeds = dc_level + amplitude * np.sin(2 * np.pi * frequency * (sine_times - sine_times[0]))
    time_list.extend(sine_times)
    speed_list.extend(sine_speeds)

    # Add final transition to zero speed
    final_transition_times = np.arange(sine_times[-1] + step, sine_times[-1] + step + transition_time, step)
    final_transition_speeds = np.linspace(dc_level, 0, len(final_transition_times))
    time_list.extend(final_transition_times)
    speed_list.extend(final_transition_speeds)

    # Add padding with zeros at the end
    padding_times = np.arange(final_transition_times[-1] + step, final_transition_times[-1] + step + padding_time, step)
    padding_speeds = np.zeros(len(padding_times))
    time_list.extend(padding_times)
    speed_list.extend(padding_speeds)

    return np.array(time_list), np.array(speed_list)

def save_profile_and_plot(time, speed, folder_name, filename="profile.csv"):
    # Create directory for the profile
    os.makedirs(folder_name, exist_ok=True)

    # Calculate acceleration, jerk, and distance
    acceleration = np.gradient(speed, time)
    jerk = np.gradient(acceleration, time)
    distance = np.cumsum(speed) * (time[1] - time[0])

    # Save profile to CSV
    data = pd.DataFrame({
        "time": time,
        "target_speed": speed,
        "acceleration": acceleration,
        "jerk": jerk,
        "distance": distance
    })
    csv_path = os.path.join(folder_name, filename)
    data.to_csv(csv_path, index=False)
    print(f"Saved speed profile to {csv_path}")

    # Plot the profile with subplots
    plot_path = os.path.join(folder_name, "profile.png")
    plot_profile(time, speed, acceleration, jerk, distance, plot_path)
    print(f"Saved plot to {plot_path}")

def plot_profile(time, speed, acceleration, jerk, distance, plot_path):
    plt.figure(figsize=(12, 10))

    # Subplot 1: Target Speed
    plt.subplot(4, 1, 1)
    plt.plot(time, speed, label='Target Speed (m/s)', color='blue')
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.grid(True)
    plt.legend()

    # Subplot 2: Acceleration
    plt.subplot(4, 1, 2)
    plt.plot(time, acceleration, label='Acceleration (m/s²)', color='orange')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s²)')
    plt.grid(True)
    plt.legend()

    # Subplot 3: Jerk
    plt.subplot(4, 1, 3)
    plt.plot(time, jerk, label='Jerk (m/s³)', color='green')
    plt.xlabel('Time (s)')
    plt.ylabel('Jerk (m/s³)')
    plt.grid(True)
    plt.legend()

    # Subplot 4: Distance
    plt.subplot(4, 1, 4)
    plt.plot(time, distance, label='Distance (m)', color='red')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.savefig(plot_path)
    plt.close()

# Example usage:
# Generate and save segmented profile
time1, speed1 = generate_segmented_profile(
    speeds=[0, 5, 10, 5, 0],    # Target speeds for each segment
    segments=[10, 15, 20, 10],     # Durations of constant-speed segments [zeros, 1st, 2nd, 3rd]
    transition_time=5,             # Time for transitions between segments
    initial_speed=3,               # Initial speed before the profile starts
    padding_time=5                 # Time to pad with zeros at the end
)
save_path = os.path.join(velocity_profiles_path, "segmented_profile")
os.makedirs(save_path, exist_ok=True)
save_profile_and_plot(time1, speed1, save_path)

# Generate and save sine profile
time2, speed2 = generate_sine_profile(
    dc_level=10,                   # DC level of the sine wave
    amplitude=5,                   # Amplitude of the sine wave
    frequency=0.1,                 # Frequency of the sine wave (Hz)
    duration=60,                   # Duration of the sine wave (seconds)
    initial_speed=5,               # Initial speed before the profile starts
    transition_time=3,             # Time to transition to the sine DC level
    padding_time=5                 # Time to pad with zeros at the end
)
save_path = os.path.join(velocity_profiles_path, "sine_profile")
os.makedirs(save_path, exist_ok=True)
save_profile_and_plot(time1, speed1, save_path)