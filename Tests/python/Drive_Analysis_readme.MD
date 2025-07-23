# Enable configurations to save data
## in planner.conf:
SHOULD_DUMP_CAR_POSE_OFFLINE=1
## in vehicle conf under aidriver_new/config/vehicle_config/ (e.g. NIROEV_HA_02.conf or ARIYA_TO_01.conf) or BASE.conf:
PATH_TRAJECTORY_LOGGING_ENABLED=1
PATH_EXTRACTION_LOGGING_ENABLED=1
PATH_ADJUSTMENT_LOGGING_ENABLED=1
PATH_TRAJECTORY_LOGGING_DEFAULT_PATH="/opt/imagry/aidriver_new/logs/"

saves path_extraction.csv, path_adjustment.csv, path_trajectory.csv in logs directory.

USE_OFFLINE_CAR_POSE_READER=0


ENABLE_SN_SAVE_HEATMAP_DEBUG=1
to save road layer, spacenet and vehicles under tmp directory
if tmp doesn't exist under aidriver_new create it manualy

## in vehicle conf under aidriver_new/config/vehicle_config/
LATERAL_CONTROLLER=ControlAPI
LATERAL_CONTROLLER_ENABLED=1 

to run new lateral control
## in  aidriver_new/modules/control_api/VehicleControl/control_config.json (open through running code in VehicleControl repo)
verify "debug_mode":true
and LATERAL_CONTROLLER_ENABLED = 1 in vehicle conf

# run aidriver_new/modules/control_api/VehicleControl/Tests/python/analyze_cpp_debug_files.py
in the python file set script_config["space_net_dump_directory"] to /opt/imagry/aidriver_new/tmp/
where you choose to copy the image files.
# python environment
Install venv:
sudo apt-get install python3-venv
python3 -m venv vehicle_control_env
source vehicle_control_env/bin/activate
pip install -r requirements.txt 
# to install tkiner 
sudo apt-get install python3-tk

# Additional dependencies
Some system dependencies are required for building the Python bindings:

```bash
# Install Eigen3 (required for matrix operations)
sudo apt-get install libeigen3-dev

# Install CMake (if not already installed)
sudo apt-get install cmake
```

# Building the Python bindings
After setting up the environment, build the Python bindings for the localization module:

```bash
# Navigate to the python_binding directory
cd python_binding

# Run the rebuild script
./rebuild.sh
```

This will create a `control_module.so` file in the `build` directory that can be imported in Python.

# Using the Python bindings
Import the control_module in your Python scripts:

```python
from control_module import AHRSLocHandler, ControlAPI

# Create a localization handler with configuration files
loc_handler = AHRSLocHandler("vehicle_config.json", "control_config.json")

# Update with IMU data
# imu_sample = ... # Create IMU sample
# timestamp = ... # Current timestamp
# loc_handler.UpdateIMU(imu_sample, timestamp)

# Update with speed data
# loc_handler.UpdateSpeed(speed, timestamp)

# Update with steering wheel angle
# loc_handler.UpdateSteeringWheel(steering_wheel_angle, timestamp)

# Update with heading
# loc_handler.UpdateHeading(heading, timestamp)

# Get position information
# position = loc_handler.GetPosition()

# Get vehicle heading
# heading = loc_handler.GetVehicleHeading()
```

## Available Classes and Methods

The following classes and methods are available in the Python bindings:

1. **AHRSLocHandler**: Main localization handler
   - `__init__(vehicle_config_path, control_config_path)`: Constructor with config file paths
   - `UpdateIMU(imu_sample, timestamp)`: Update with IMU data
   - `UpdateSpeed(speed, timestamp)`: Update vehicle speed
   - `UpdateSteeringWheel(steering_wheel_angle, timestamp)`: Update steering wheel angle
   - `UpdateHeading(heading, timestamp)`: Update heading information
   - `GetPosition()`: Get current position
   - `GetVehicleHeading()`: Get current heading
   - `ResetVehicleState()`: Reset vehicle state

2. **ControlAPI**: Main control API with localization integration
   - `__init__(vehicle_config_path, control_config_path)`: Constructor with config files
   - Various control-related methods that use localization data

See example scripts in the `python_binding` directory for more usage examples.