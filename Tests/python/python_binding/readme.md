to run the pybind simulation:
1. install python virtual env:
    ```
    conda env create --file ../environment.yaml
    ```
2. install pybind cmake 
    ``` 
    conda activate auto_vehicle_cont
    conda install -c conda-forge pybind11
    conda install -c conda-forge cmake
    ```
3. copy files to 3rdParty in the main directory (todo: put in some      common place in the server)
4. set local path variables in root/control_config.json
    ```
    "control_modul_dir": "root/folder/path",
    "visualization_data_path": "root/folder/path/data/temp_results/visualization_data"
    ```
