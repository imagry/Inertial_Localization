cmake_minimum_required(VERSION 3.10)
project(VehicleControlOperational)
include_directories(${CMAKE_SOURCE_DIR})

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)

# Add the directory with the Eigen library to Utils
include_directories(${CMAKE_SOURCE_DIR}/3rdParty/eigen)

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

add_subdirectory(Utils)
add_subdirectory(Tests)
add_subdirectory(wrapper)

add_library(ControlAPI
    ahrs_loc_handler.cpp
)
target_link_libraries(ControlAPI PUBLIC Utils nlohmann_json::nlohmann_json)
add_executable(VehicleControlOperational Main.cpp)
target_link_libraries(VehicleControlOperational PUBLIC Tests Utils ControlAPI)