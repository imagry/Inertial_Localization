cmake_minimum_required(VERSION 3.10)

project(localization_pybind_module)

# Set the build type to Debug (enables debug information)
set(CMAKE_BUILD_TYPE Debug)

# Set the path to the virtual environment where pybind11 is installed
set(VENV_PATH "../vehicle_control_env")

set(THIRD_PARTY_SOURCE_DIR "../../../3rdParty")

# We'll need to install pybind11 in our virtual environment
execute_process(
    COMMAND ${VENV_PATH}/bin/pip install pybind11
    RESULT_VARIABLE PYBIND11_INSTALL_RESULT
)

# Get Python include directory from the virtual environment
execute_process(
    COMMAND ${VENV_PATH}/bin/python -c "import sysconfig; print(sysconfig.get_path('include'))"
    OUTPUT_VARIABLE PYTHON_INCLUDE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Add Python include directory
include_directories(${PYTHON_INCLUDE_DIR})

# Specify Python version (optional)
# set(PYBIND11_PYTHON_VERSION 3.10.8)

# Get the path to pybind11 cmake config
execute_process(
    COMMAND ${VENV_PATH}/bin/python -c "import pybind11; print(pybind11.get_cmake_dir())"
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Add the pybind11 directory to CMAKE_PREFIX_PATH
list(APPEND CMAKE_PREFIX_PATH "${PYBIND11_CMAKE_DIR}")

# Find pybind11 package
find_package(pybind11 REQUIRED)

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

# THOSE LINES ADDED BY DOR DUE TO EIGEN BEING INSTALLED IN A DIFFERENT DIR
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


# Add the directory with the Eigen library to Utils
include_directories(${THIRD_PARTY_SOURCE_DIR}/eigen)

# add_subdirectory(Utils)

# Specify the directory where your source files are located
# set(SOURCE_DIR "cpp")
set(SOURCE_DIR "../../../Utils")
# set(MODULE_DIR "")
# Find all source files in the specified directory and its subdirectories
file(GLOB_RECURSE SOURCE_FILES "${SOURCE_DIR}/*.cpp")
# Append the pybind module source file to the SOURCE_FILES list
list(APPEND SOURCE_FILES "localization_pybind_module.cpp")
# Append API source file to the SOURCE_FILES list
list(APPEND SOURCE_FILES 
                         "../../../ahrs_loc_handler.cpp"
                         "../../../wrapper/localization_api_wrapper.cpp")
# B10: Removed LocalizationAPI.cpp as part of control code removal


# file(GLOB_RECURSE MAIN_MODULE "${MODULE_DIR}/localization_pybind_module.cpp")

# Add the source files to your library
add_library(localization_pybind_module MODULE ${SOURCE_FILES} ${MAIN_MODULE})


# Link the library with pybind11
target_link_libraries(localization_pybind_module PRIVATE pybind11::module nlohmann_json::nlohmann_json)


# Add this line to specify the directory containing the shared library
link_directories("${PROJECT_SOURCE_DIR}/build")

# Specify include directories for the target
target_include_directories(localization_pybind_module PRIVATE 
        ${PROJECT_SOURCE_DIR}/cpp
        ${THIRD_PARTY_SOURCE_DIR}/eigen
        "../../../"  # Add the path to your LocalizationAPI.hpp and other headers
)