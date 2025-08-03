cmake_minimum_required(VERSION 3.10)
project(InertialLocalization)
include_directories(${CMAKE_SOURCE_DIR})

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)

# Add the directory with the Eigen library to Utils
include_directories(${CMAKE_SOURCE_DIR}/3rdParty/eigen)

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)

# Add Google Test
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(json googletest)

add_subdirectory(Utils)
add_subdirectory(Tests)
add_subdirectory(wrapper)

add_library(ControlAPI
    ahrs_loc_handler.cpp
)
target_link_libraries(ControlAPI PUBLIC Utils nlohmann_json::nlohmann_json)
add_executable(VehicleControlOperational Main.cpp)
# Remove the Tests library from the link list to avoid dependency on the broken Tests
target_link_libraries(VehicleControlOperational PUBLIC Utils ControlAPI)