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